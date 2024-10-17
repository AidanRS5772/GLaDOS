#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <iostream>
#include <memory>
#include <string>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
    websocket::stream<tcp::socket> ws_;
    beast::flat_buffer buffer_;

   public:
    explicit WebSocketSession(tcp::socket socket)
        : ws_(std::move(socket)) {}

    void start() {
        ws_.async_accept(
            beast::bind_front_handler(
                &WebSocketSession::on_accept,
                shared_from_this()));
    }

   private:
    void on_accept(beast::error_code ec) {
        if (ec) {
            std::cerr << "Error during WebSocket handshake: " << ec.message() << std::endl;
            return;
        }
        do_read();
    }

    void do_read() {
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &WebSocketSession::on_read,
                shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred) {
        if (ec == websocket::error::closed) {
            return;
        }
        if (ec) {
            std::cerr << "Error during WebSocket read: " << ec.message() << std::endl;
            return;
        }

        std::string tag;
        if (buffer_.size() >= 4) {
            auto data = static_cast<const char*>(buffer_.data().data());
            tag = std::string(data, 4);
            buffer_.consume(4);
        } else {
            std::cerr << "Received message without a valid tag" << std::endl;
            buffer_.consume(buffer_.size());
            do_read();
            return;
        }

        if (tag == "CORD") {
            handle_cords();
        } else {
            std::cerr << "Unknown tag: " << tag << std::endl;
        }

        do_read();
    }

    void handle_cords() {
        if (buffer_.size() >= 8) {
            int int1, int2;
            const char* data = static_cast<const char*>(buffer_.data().data());
            std::memcpy(&int1, data, sizeof(int));
            std::memcpy(&int2, data + sizeof(int), sizeof(int));

            std::cout << "Received Coordinates: (" << int1 << ", " << int2 << ")" << std::endl;

            buffer_.consume(8);

            send_message("CORD");
        } else {
            std::cerr << "Not enough data for coordinates (expected 8 bytes)" << std::endl;
            buffer_.consume(buffer_.size());
        }
    }

    void send_message(const std::string& message) {
        ws_.async_write(
            net::buffer(message),
            [self = shared_from_this(), message](beast::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error sending message: " << ec.message() << std::endl;
                }
            });
    }
};

class WebSocketServer {
    tcp::acceptor acceptor_;

   public:
    WebSocketServer(net::io_context& ioc, tcp::endpoint endpoint)
        : acceptor_(ioc) {
        acceptor_.open(endpoint.protocol());
        acceptor_.set_option(net::socket_base::reuse_address(true));
        acceptor_.bind(endpoint);
        acceptor_.listen();

        do_accept();
    }

   private:
    void do_accept() {
        acceptor_.async_accept(
            [this](beast::error_code ec, tcp::socket socket) {
                if (!ec) {
                    std::make_shared<WebSocketSession>(std::move(socket))->start();
                } else {
                    std::cerr << "Error accepting connection: " << ec.message() << std::endl;
                }
                do_accept();
            });
    }
};

// Main entry point for the server
int main() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        auto const port = static_cast<unsigned short>(std::atoi("8080"));

        net::io_context ioc{std::thread::hardware_concurrency()};

        auto server = std::make_shared<WebSocketServer>(ioc, tcp::endpoint{address, port});

        // Run the I/O context in multiple threads
        std::vector<std::thread> threads;
        for (std::size_t i = 0; i < std::thread::hardware_concurrency(); ++i) {
            threads.emplace_back([&ioc] { ioc.run(); });
        }

        for (auto& t : threads) {
            t.join();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}
