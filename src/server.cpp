#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>

using namespace std;

class session : public std::enable_shared_from_this<session> {
    boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
    boost::beast::flat_buffer buffer_;
    string client_id_;

    public:
        explicit session(boost::asio::ip::tcp::socket &&socket): ws_(std::move(socket)) {
            boost::uuids::uuid uuid = boost::uuids::random_generator()();
            client_id_ = boost::uuids::to_string(uuid);

            // Set TCP_NODELAY on the underlying TCP socket
            boost::asio::ip::tcp::no_delay option(true);
            ws_.next_layer().socket().set_option(option);
        }

        void run() {
            ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
            auto self = shared_from_this();
            ws_.async_accept(boost::beast::bind_front_handler(&session::on_accept, self));
        }

        void handle_close(const std::string& reason) {
            cout << reason << client_id_ << endl;

            cv::destroyWindow(client_id_);
            cv::waitKey(1);  // Neded to close window

            ws_.next_layer().close();
        }

    private:
        void on_accept(boost::beast::error_code ec) {
            if (ec) {
                cerr << "Accept error: " << ec.message() << endl;
                return;
            }
            cout << "Client connected with ID: " << client_id_ << endl;

            do_read();
        }


        // Forms recursive loop with on_read()
        void do_read() {
            auto self = shared_from_this();

            ws_.async_read(buffer_, [self](boost::beast::error_code ec, size_t bytes_transferred) {
                self->on_read(ec, bytes_transferred);
            });
        }

        // processes raw binary into cv::Mat
        cv::Mat process_data_to_frame(boost::beast::flat_buffer &buffer){
            auto data = boost::asio::buffer_cast<const uint8_t*>(buffer.data());
            size_t data_size = buffer.size();

            vector<uint8_t> frame_data(data, data + data_size);

            cv::Mat frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);
            
            if (frame.empty()) {
                throw runtime_error("Failed to decode the frame.");
            }

            buffer.consume(buffer.size());

            return frame;
        }

        // Main event function this is where the action happens
        void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
            boost::ignore_unused(bytes_transferred);

            if (ec == boost::beast::websocket::error::closed) {
                handle_close("Client disconnected: ");
                return;
            }

            if (ec) {
                cerr << "Read error: " << ec.message() << endl;
                handle_close("Closing due to read error: ");
                return;
            }

            if (!ws_.got_text()) {
                try {
                    cv::Mat frame = process_data_to_frame(buffer_);
                    cv::imshow(client_id_, frame);

                    if (cv::waitKey(1) == 'q') {
                        close_connection();
                        return;
                    }

                } catch (const std::exception& e) {
                    cerr << "Frame processing error: " << e.what() << endl;
                    handle_close("Closing due to frame processing error: ");
                }
            }

            // Continue to next frame
            do_read();
        }

        void close_connection() {
            if (!ws_.is_open()) {
                cout << "WebSocket is already closed for client: " << client_id_ << endl;
                return;
            }

            cout << "Starting Server Close of Client: " << client_id_ << endl;

            // Step 1: Ensure any pending writes are complete
            auto self = shared_from_this();
            ws_.async_write(boost::asio::buffer(""), [self](boost::beast::error_code ec, std::size_t) {
                if (ec) {
                    cerr << "Error flushing data: " << ec.message() << endl;
                    return;
                }

                // Step 2: After flushing, initiate the WebSocket close handshake
                self->ws_.async_close(boost::beast::websocket::close_code::normal,
                    [self](boost::beast::error_code close_ec) {
                        if (close_ec) {
                            cerr << "Close error: " << close_ec.message() << endl;
                        } else {
                            cout << "Server closed Client with Id: " << self->client_id_ << endl;
                        }
                        self->handle_close("Clean up after async_close: ");
                    });
            });
        }

};

class listener : public std::enable_shared_from_this<listener> {
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;

public:
    listener(boost::asio::io_context &ioc, boost::asio::ip::tcp::endpoint endpoint) : acceptor_(ioc), socket_(ioc) {
        boost::beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            cerr << "Open error: " << ec.message() << endl;
            return;
        }

        acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
        if (ec) {
            cerr << "Set option error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.bind(endpoint, ec);
        if (ec) {
            cerr << "Bind error: " << ec.message() << endl;
            return;
        }

        acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
        if (ec) {
            cerr << "Listen error: " << ec.message() << endl;
            return;
        }
    }

    void do_accept() {
        std::cout << "Listening..." << std::endl;

        // Use shared_from_this() only after the object is managed by a shared_ptr
        auto self = shared_from_this();
        acceptor_.async_accept(socket_, [self](boost::beast::error_code ec) {
                if (ec) {
                    std::cerr << "Accept error: " << ec.message() << std::endl;
                } else {
                    std::cout << "Connection accepted..." << std::endl;
                    std::make_shared<session>(std::move(self->socket_))->run();
                }

                // Continue accepting new connections
                self->do_accept();
            });
    }
};

int main() {
    try {
        // Hard-coded IP address and port
        const string ip_address = "0.0.0.0";  // Use "0.0.0.0" to listen on all interfaces
        const unsigned short port = 8080;
        cout << "Starting Server..." << endl;

        boost::asio::io_context ioc{1};

        boost::asio::ip::tcp::endpoint endpoint{ boost::asio::ip::make_address(ip_address), port};

        auto listener_ptr = make_shared<listener>(ioc, endpoint);

        listener_ptr->do_accept();

        ioc.run();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}