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
            // Assuming the first 4 bytes represent the tag (could be more or less)
            auto data = static_cast<const char*>(buffer_.data().data());
            tag = std::string(data, 4);
            buffer_.consume(4);  // Consume the tag part
        } else {
            std::cerr << "Received message without a valid tag" << std::endl;
            buffer_.consume(buffer_.size());
            do_read();
            return;
        }

        if (tag == "CORD") {
        } else {
            std::cerr << "Unknown tag: " << tag << std::endl;
        }

        do_read();
    }

    void do_write(const std::string& message) {
        ws_.async_write(
            net::buffer(message),
            beast::bind_front_handler(
                &WebSocketSession::on_write,
                shared_from_this()));
    }

    void on_write(beast::error_code ec, std::size_t bytes_transferred) {
        if (ec) {
            std::cerr << "Error during WebSocket write: " << ec.message() << std::endl;
        }

        do_read();
    }
};

class WebSocketServer {
    tcp::acceptor acceptor_;
    tcp::socket socket_;

   public:
    WebSocketServer(net::io_context& ioc, tcp::endpoint endpoint)
        : acceptor_(ioc), socket_(ioc) {
        acceptor_.open(endpoint.protocol());
        acceptor_.set_option(net::socket_base::reuse_address(true));
        acceptor_.bind(endpoint);
        acceptor_.listen();

        do_accept();
    }

   private:
    void do_accept() {
        acceptor_.async_accept(
            socket_,
            [this](beast::error_code ec) {
                if (!ec) {
                    std::make_shared<WebSocketSession>(std::move(socket_))->start();
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

        net::io_context ioc{1};

        // Create and run the WebSocket server
        tcp::endpoint endpoint{address, port};
        std::make_shared<WebSocketServer>(ioc, endpoint);

        // Run the I/O context to start the event loop
        ioc.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
