#include <array>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class Server;

class Session : public std::enable_shared_from_this<Session> {
    websocket::stream<tcp::socket> ws_;
    beast::flat_buffer buffer_;
    std::string pair_name_;
    std::shared_ptr<Server> server_;

   public:
    explicit Session(tcp::socket socket, std::shared_ptr<Server> server)
        : ws_(std::move(socket)), server_(std::move(server)) {}

    void start() {
        ws_.async_accept(beast::bind_front_handler(&Session::on_accept, shared_from_this()));
    }

    void send_raw_data(const void* data, std::size_t size) {
        ws_.async_write(
            net::buffer(data, size),
            [self = shared_from_this(), size](beast::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error sending raw data: " << ec.message() << std::endl;
                } else {
                    std::cout << "Sent " << bytes_transferred << " bytes." << std::endl;
                }
            });
    }

   private:
    void on_accept(beast::error_code ec);

    void do_read() {
        ws_.async_read(buffer_, beast::bind_front_handler(&Session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred) {
        if (ec == websocket::error::closed) {
            return;
        }
        if (ec) {
            std::cerr << "Error during read: " << ec.message() << std::endl;
            return;
        }

        std::string tag;
        if (buffer_.size() >= 4) {
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
            handle_cords();
        } else {
            std::cerr << "Unknown tag: " << tag << std::endl;
        }

        do_read();
    }

    void handle_cords() {
        if (buffer_.size() >= 8) {
            int32_t int1, int2;
            const char* data = static_cast<const char*>(buffer_.data().data());

            int1 = ntohl(*reinterpret_cast<const uint32_t*>(data));
            int2 = ntohl(*reinterpret_cast<const uint32_t*>(data + sizeof(int32_t)));

            std::cout << "Received Coordinates: (" << int1 << ", " << int2 << ")" << std::endl;

            buffer_.consume(8);

            std::string message = "CACK";
            send_raw_data(message.data(), message.size());
        } else {
            std::cerr << "Not enough data for coordinates (expected 8 bytes)" << std::endl;
            buffer_.consume(buffer_.size());
        }
    }
};

class Pair {
    std::mutex mutex_;
    std::shared_ptr<Session> primary_, secondary_;
    std::optional<std::array<int, 2>> primary_cord_, secondary_cord_;

   public:
    Pair() : primary_(nullptr), secondary_(nullptr) {}

    void add_session(std::shared_ptr<Session> sess, int type) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (type == 1) {
            if (!primary_) {
                primary_ = std::move(sess);
                std::cout << "Primary session added to pair.\n";
            } else {
                std::cerr << "This Pair already has a primary session.\n";
            }
        } else if (type == 2) {
            if (!secondary_) {
                secondary_ = std::move(sess);
                std::cout << "Secondary session added to pair.\n";
            } else {
                std::cerr << "This Pair already has a secondary session.\n";
            }
        } else {
            std::cerr << "Invalid session type, expected 1 or 2.\n";
        }
    }

    void update_cord(std::shared_ptr<Session> sess, std::array<int, 2> cord) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (sess == primary_) {
            if (!primary_cord_.has_value()) {
                primary_cord_ = cord;
            }
        } else if (sess == secondary_) {
            if (!secondary_cord_.has_value()) {
                secondary_cord_ = cord;
            }
        } else {
            std::cerr << "Acsessed Pair not with non-member session" << std::endl;
        }

        if (primary_cord_.has_value() && secondary_cord_.has_value()) {
            std::array<float, 3> comb_cord = {1.5f, 1.5f, 1.5f};
            std::vector<char> buffer(4 + sizeof(float) * 3);

            std::string tag = "CORD";
            std::memcpy(buffer.data(), tag.data(), 4);

            std::memcpy(buffer.data() + 4, &comb_cord[0], sizeof(float));
            std::memcpy(buffer.data() + 4 + sizeof(float), &comb_cord[1], sizeof(float));
            std::memcpy(buffer.data() + 4 + 2 * sizeof(float), &comb_cord[2], sizeof(float));

            primary_->send_raw_data(buffer.data(), buffer.size());

            primary_cord_.reset();
            secondary_cord_.reset();
        }
    }
};

class Server : public std::enable_shared_from_this<Server> {
    tcp::acceptor acceptor_;
    std::mutex mutex_;
    std::unordered_map<std::string, Pair> pairs_;

   public:
    Server(net::io_context& ioc, tcp::endpoint endpoint)
        : acceptor_(ioc) {
        acceptor_.open(endpoint.protocol());
        acceptor_.set_option(net::socket_base::reuse_address(true));
        acceptor_.bind(endpoint);
        acceptor_.listen();

        do_accept();
    }

    void add_session(std::shared_ptr<Session> sess, const std::string& name, int type) {
        std::lock_guard<std::mutex> lock(mutex_);

        Pair& pair = pairs_[name];
        pair.add_session(std::move(sess), type);
    }

   private:
    void do_accept() {
        acceptor_.async_accept(
            [this](beast::error_code ec, tcp::socket socket) {
                if (!ec) {
                    std::make_shared<Session>(std::move(socket), shared_from_this())->start();
                } else {
                    std::cerr << "Error accepting connection: " << ec.message() << std::endl;
                }
                do_accept();
            });
    }
};

void Session::on_accept(beast::error_code ec) {
    if (ec) {
        std::cerr << "Error during handshake: " << ec.message() << std::endl;
        return;
    }

    std::cout << "Input pair name: ";
    std::cin >> pair_name_;
    std::cout << std::endl;

    int type;
    std::cout << "Input type of session (1 for primary, 2 for secondary): ";
    std::cin >> type;
    std::cout << std::endl;

    server_->add_session(shared_from_this(), pair_name_, type);

    do_read();
}

// Main entry point for the server
int main() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        auto const port = static_cast<unsigned short>(std::atoi("8080"));

        net::io_context ioc{std::thread::hardware_concurrency()};

        auto server = std::make_shared<Server>(ioc, tcp::endpoint{address, port});

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