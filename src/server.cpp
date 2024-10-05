#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/version.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

// SessionType to distinguish between frames and coordinates sessions
enum class SessionType {
    Frames,
    Coordinates
};

class session : public std::enable_shared_from_this<session> {
    boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
    boost::beast::flat_buffer buffer_;
    std::string client_id_;
    SessionType session_type_;

    cv::Mat frame_;

public:
    session(boost::asio::ip::tcp::socket socket, SessionType session_type)
        : ws_(std::move(socket)),
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
        ws_.async_accept([self](boost::beast::error_code ec) {
            self->on_accept(ec);
        });
    }

    void handle_close(const std::string& reason) {
        std::cout << reason << client_id_ << std::endl;

        if (ws_.is_open()) {
            boost::beast::error_code ec;
            ws_.close(boost::beast::websocket::close_code::normal, ec);
            if (ec) {
                std::cerr << "Error closing WebSocket: " << ec.message() << std::endl;
            }
        }
    }

    std::string get_client_id() const { return client_id_; }

private:
    void on_accept(boost::beast::error_code ec);
    void do_read();
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
    void send_ack_signal();
    void process_data_to_frame(boost::beast::flat_buffer& buffer, cv::Mat& frame);
    void close_connection();
};

// Session member function implementations
void session::on_accept(boost::beast::error_code ec) {
    if (ec) {
        std::cerr << "Accept error: " << ec.message() << std::endl;
        return;
    }
    std::cout << "Client connected with ID: " << client_id_ << std::endl;

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

                // Save the frame
                static int frame_count = 0;
                std::ostringstream filename;
                filename << "received_frame_" << client_id_ << "_" << frame_count++ << ".jpg";
                cv::imwrite("../../../src/test_imgs/" + filename.str(), frame_);

                // Simulate blocking operation
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

        // Continue reading
        // self->do_read();  // Already called in on_read
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
    buffer_.consume(buffer_.size());
}

void session::close_connection() {
    if (!ws_.is_open()) {
        std::cout << "WebSocket is already closed for client: " << client_id_ << std::endl;
        return;
    }

    std::cout << "Starting Server Close of Client: " << client_id_ << std::endl;

    auto self = shared_from_this();
    ws_.async_close(boost::beast::websocket::close_code::normal, [self](boost::beast::error_code ec) {
        if (ec) {
            std::cerr << "Close error: " << ec.message() << std::endl;
        } else {
            std::cout << "Server closed Client with Id: " << self->client_id_ << std::endl;
        }
        self->handle_close("Clean up after async_close: ");
    });
}

// Listener class
class listener : public std::enable_shared_from_this<listener> {
    boost::asio::ip::tcp::acceptor acceptor_;
    SessionType session_type_;

public:
    listener(boost::asio::io_context& ioc, boost::asio::ip::tcp::endpoint endpoint, SessionType session_type)
        : acceptor_(ioc), session_type_(session_type) {
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

    void do_accept() {
        acceptor_.async_accept([self = shared_from_this()](boost::beast::error_code ec, boost::asio::ip::tcp::socket socket) mutable {
            if (ec) {
                std::cerr << "Accept error: " << ec.message() << std::endl;
            } else {
                std::cout << "Connection accepted on port " << self->acceptor_.local_endpoint().port() << "..." << std::endl;

                // Create a new session
                auto new_session = std::make_shared<session>(std::move(socket), self->session_type_);

                // Start the session
                new_session->run();
            }

            // Accept the next connection
            self->do_accept();
        });
    }
};

int main() {
    try {
        // IP address and ports
        const std::string ip_address = "0.0.0.0";  // Listen on all interfaces
        const unsigned short frames_port = 8080;
        const unsigned short coords_port = 8081;

        std::cout << "Starting Server..." << std::endl;

        // Create io_context for the server
        boost::asio::io_context ioc;

        // Create endpoints for frames and coordinates
        boost::asio::ip::tcp::endpoint frames_endpoint{boost::asio::ip::make_address(ip_address), frames_port};
        boost::asio::ip::tcp::endpoint coords_endpoint{boost::asio::ip::make_address(ip_address), coords_port};

        // Create listeners for frames and coordinates
        auto listener_frames = std::make_shared<listener>(ioc, frames_endpoint, SessionType::Frames);
        auto listener_coords = std::make_shared<listener>(ioc, coords_endpoint, SessionType::Coordinates);

        // Start accepting connections
        listener_frames->do_accept();
        listener_coords->do_accept();

        // Run the io_context in multiple threads
        std::vector<std::thread> threads;
        unsigned int thread_count = std::thread::hardware_concurrency();
        if (thread_count == 0) {
            thread_count = 4;  // Default to 4 threads if hardware_concurrency can't determine
        }

        for (unsigned int i = 0; i < thread_count; ++i) {
            threads.emplace_back([&ioc]() {
                ioc.run();
            });
        }

        // Wait for all threads to finish
        for (auto& t : threads) {
            t.join();
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}