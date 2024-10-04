#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <chrono>
#include <thread>

using namespace std;

// SessionType to distinguish between frames and coordinates sessions
enum class SessionType {
    Frames,
    Coordinates
};

class session_manager;

class session : public std::enable_shared_from_this<session> {
    boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
    boost::beast::flat_buffer buffer_;
    std::string client_id_;
    session_manager& manager_;
    SessionType session_type_;

    // Image data
    cv::Mat frame_;

   public:
    explicit session(boost::asio::ip::tcp::socket&& socket, session_manager& manager, SessionType session_type)
        : ws_(std::move(socket)),
          manager_(manager),
          session_type_(session_type) {
        // Generate a unique client ID
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        client_id_ = boost::uuids::to_string(uuid);

        // Set TCP no-delay option
        boost::asio::ip::tcp::no_delay option(true);
        ws_.next_layer().socket().set_option(option);
    }

    void run() {
        ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
        auto self = shared_from_this();
        ws_.async_accept(boost::beast::bind_front_handler(&session::on_accept, self));
    }

    void handle_close(const std::string& reason);
    std::string get_client_id() const { return client_id_; }

   private:
    void on_accept(boost::beast::error_code ec);
    void do_read();
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
    void send_ack_signal();
    void process_data_to_frame(boost::beast::flat_buffer& buffer, cv::Mat& frame);
    void close_connection();
};

class session_manager {
    std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<session>> sessions_;

   public:
    void add_session(const std::string& client_id, std::shared_ptr<session> ses) {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_[client_id] = ses;
        std::cout << "Client added with ID: " << client_id << std::endl;
    }

    void remove_session(const std::string& client_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions_.erase(client_id);
        std::cout << "Client removed with ID: " << client_id << std::endl;
    }
};

// Session member function implementations

void session::on_accept(boost::beast::error_code ec) {
    if (ec) {
        std::cerr << "Accept error: " << ec.message() << std::endl;
        return;
    }
    std::cout << "Client connected with ID: " << client_id_ << std::endl;

    auto self = shared_from_this();
    manager_.add_session(client_id_, self);

    if (session_type_ == SessionType::Frames) {
        send_ack_signal();
    }

    do_read();
}

void session::do_read() {
    auto self = shared_from_this();

    ws_.async_read(buffer_, [self](boost::beast::error_code ec, size_t bytes_transferred) {
        self->on_read(ec, bytes_transferred);
    });
}

void session::on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    if (ec == boost::beast::websocket::error::closed) {
        handle_close("Client disconnected: ");
        return;
    }

    if (ec) {
        std::cerr << "Read error: " << ec.message() << std::endl;
        handle_close("Closing due to read error: ");
        return;
    }

    try {
        if (session_type_ == SessionType::Frames) {
            if (ws_.got_binary()) {
                // Process frame data
                process_data_to_frame(buffer_, frame_);

                // Display the frame
                static int frame_count = 0;
                std::ostringstream filename;
                filename << "received_frame_" << client_id_ << "_" << frame_count++ << ".jpg";

                // Save the frame to a folder
                try {
                    bool success = cv::imwrite("../../../src/test_imgs/" + filename.str(), frame_);
                    if (!success) {
                        std::cerr << "Failed to save image: " << filename.str() << std::endl;
                    } else {
                        std::cout << "Image saved successfully: " << filename.str() << std::endl;
                    }
                } catch (const cv::Exception& e) {
                    std::cerr << "OpenCV exception: " << e.what() << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Standard exception: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "Unknown exception occurred while saving image." << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                // Send ACK to client
                send_ack_signal();
            } else {
                std::cerr << "Expected binary message for frame data but received text." << std::endl;
                handle_close("Closing due to incorrect message type.");
                return;
            }
        } else if (session_type_ == SessionType::Coordinates) {
            // Process coordinates data
            auto data = boost::asio::buffer_cast<const uint8_t*>(buffer_.data());
            size_t data_size = buffer_.size();

            if (data_size != 8) {
                std::cerr << "Invalid coordinates data size: " << data_size << std::endl;
            } else {
                int32_t net_x = 0, net_y = 0;
                std::memcpy(&net_x, data, 4);
                std::memcpy(&net_y, data + 4, 4);

                int32_t x = ntohl(net_x);
                int32_t y = ntohl(net_y);

                std::cout << "Received coordinates: (" << x << ", " << y << ")" << std::endl;
            }

            buffer_.consume(buffer_.size());
        }

        // Continue reading
        do_read();

    } catch (const std::exception& e) {
        std::cerr << "Processing error: " << e.what() << std::endl;
        handle_close("Closing due to processing error: ");
    }
}

void session::send_ack_signal() {
    auto self = shared_from_this();
    std::string msg = "ACK";
    ws_.text(true);  // Ensure we're sending a text message
    ws_.async_write(boost::asio::buffer(msg), [self](boost::beast::error_code ec, std::size_t) {
        if (ec) {
            std::cerr << "Error sending ACK signal: " << ec.message() << std::endl;
            return;
        }

        // Now wait for the next frame from the client
        self->do_read();
    });
}

void session::process_data_to_frame(boost::beast::flat_buffer& buffer, cv::Mat& frame) {
    // Get a const_buffer pointing to the buffer's data
    auto data = buffer.data();
    size_t data_size = buffer.size();

    // Create a vector of unsigned char from the buffer
    std::vector<uchar> frame_data(boost::asio::buffer_cast<const uchar*>(data),
                                  boost::asio::buffer_cast<const uchar*>(data) + data_size);

    // Decode the JPEG image from the raw bytes
    frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);

    if (frame.empty()) {
        throw std::runtime_error("Failed to decode the frame.");
    }

    // Consume the buffer data
    buffer.consume(buffer.size());
}

void session::close_connection() {
    if (!ws_.is_open()) {
        std::cout << "WebSocket is already closed for client: " << client_id_ << std::endl;
        return;
    }

    std::cout << "Starting Server Close of Client: " << client_id_ << std::endl;

    auto self = shared_from_this();
    ws_.async_write(boost::asio::buffer(""), [self](boost::beast::error_code ec, std::size_t) {
        if (ec) {
            std::cerr << "Error flushing data: " << ec.message() << std::endl;
            return;
        }

        self->ws_.async_close(boost::beast::websocket::close_code::normal,
                              [self](boost::beast::error_code close_ec) {
                                  if (close_ec) {
                                      std::cerr << "Close error: " << close_ec.message() << std::endl;
                                  } else {
                                      std::cout << "Server closed Client with Id: " << self->client_id_ << std::endl;
                                  }
                                  self->handle_close("Clean up after async_close: ");
                              });
    });
}

void session::handle_close(const std::string& reason) {
    std::cout << reason << client_id_ << std::endl;

    // cv::destroyWindow(client_id_);
    // cv::waitKey(1);  // Needed to close window

    manager_.remove_session(client_id_);

    ws_.next_layer().close();
}

// Listener class

class listener : public std::enable_shared_from_this<listener> {
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    session_manager& manager_;
    SessionType session_type_;

   public:
    listener(boost::asio::io_context& ioc, boost::asio::ip::tcp::endpoint endpoint, session_manager& manager, SessionType session_type)
        : acceptor_(ioc), socket_(ioc), manager_(manager), session_type_(session_type) {
        boost::beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            std::cerr << "Open error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
        if (ec) {
            std::cerr << "Set option error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.bind(endpoint, ec);
        if (ec) {
            std::cerr << "Bind error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
        if (ec) {
            std::cerr << "Listen error: " << ec.message() << std::endl;
            return;
        }
    }

    void do_accept();

   private:
};

void listener::do_accept() {
    std::cout << "Listening on port " << acceptor_.local_endpoint().port() << "..." << std::endl;

    auto self = shared_from_this();
    acceptor_.async_accept(socket_, [self](boost::beast::error_code ec) {
        if (ec) {
            std::cerr << "Accept error: " << ec.message() << std::endl;
        } else {
            std::cout << "Connection accepted on port " << self->acceptor_.local_endpoint().port() << "..." << std::endl;
            std::make_shared<session>(std::move(self->socket_), self->manager_, self->session_type_)->run();
        }

        // Continue accepting new connections
        self->do_accept();
    });
}

int main() {
    try {
        // IP address and ports
        const std::string ip_address = "0.0.0.0";  // Listen on all interfaces
        const unsigned short frames_port = 8080;
        const unsigned short coords_port = 8081;
        std::cout << "Starting Server..." << std::endl;

        boost::asio::io_context ioc{1};

        boost::asio::ip::tcp::endpoint frames_endpoint{boost::asio::ip::make_address(ip_address), frames_port};
        boost::asio::ip::tcp::endpoint coords_endpoint{boost::asio::ip::make_address(ip_address), coords_port};

        session_manager manager;

        auto listener_frames = std::make_shared<listener>(ioc, frames_endpoint, manager, SessionType::Frames);
        auto listener_coords = std::make_shared<listener>(ioc, coords_endpoint, manager, SessionType::Coordinates);

        listener_frames->do_accept();
        listener_coords->do_accept();

        ioc.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
