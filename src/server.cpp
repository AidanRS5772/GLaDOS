#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rapidcsv.h"

namespace beast = boost::beast;
namespace net = boost::asio;
namespace websocket = beast::websocket;
using tcp = boost::asio::ip::tcp;

// Forward declaration
class Session;

// Pair class declaration
class Pair : public std::enable_shared_from_this<Pair> {
    std::weak_ptr<Session> primary_, secondary_;
    std::optional<std::array<int32_t, 2>> cord1, cord2;

   public:
    void add_session(std::shared_ptr<Session> sess, int type);

    void update_cord(std::shared_ptr<Session> sess, const std::array<int32_t, 2>& cord);

   private:
    std::optional<std::array<double, 2>> find_lsr_cords(const std::array<int32_t, 2>& px1, const std::array<int32_t, 2>& px2) {
        // temporary math for finding the point in 3d space and then converting to servo cordinate system
        double x = (static_cast<double>(px1[0]) + static_cast<double>(px1[1])) / 2;
        double y = (static_cast<double>(px2[0]) + static_cast<double>(px2[1])) / 2;

        std::optional<std::array<double, 2>> lsr_cords;
        lsr_cords = {x, y};
        return lsr_cords;
    }
};

// Session class definition
class Session : public std::enable_shared_from_this<Session> {
    websocket::stream<tcp::socket> ws_;
    std::weak_ptr<Pair> pair_;

    beast::flat_buffer buffer_;

    std::deque<std::vector<uint8_t>> write_queue_;
    bool write_in_progress_ = false;

   public:
    explicit Session(tcp::socket socket)
        : ws_(std::move(socket)) {}

    void start() {
        ws_.async_accept(beast::bind_front_handler(&Session::on_accept, shared_from_this()));
    }

    void send_message(const std::string& tag, const std::vector<uint8_t>& body);

    void set_pair(std::weak_ptr<Pair> pair) {
        pair_ = pair;
    }

   private:
    void on_accept(beast::error_code ec) {
        if (ec) {
            std::cerr << "WebSocket accept failed: " << ec.message() << std::endl;
            return;
        }
        do_read();
    }

    void do_read() {
        ws_.async_read(buffer_, beast::bind_front_handler(&Session::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t);

    void do_write();

    void on_write(beast::error_code ec);
};

void Pair::add_session(std::shared_ptr<Session> sess, int type) {
    if (type == 1) {
        if (primary_.expired()) {
            primary_ = sess;
            std::cout << "Session added to pair as primary." << std::endl;
        } else {
            std::cerr << "This pair already has a primary session.";
        }
    } else if (type == 2) {
        if (secondary_.expired()) {
            secondary_ = sess;
            std::cout << "Session added to pair as secondary." << std::endl;
        } else {
            std::cerr << "This pair already has a secondary session.";
        }
    } else {
        std::cerr << "Given type is not either 1 or 2.";
    }
}

void Pair::update_cord(std::shared_ptr<Session> sess, const std::array<int32_t, 2>& cord) {
    auto primary = primary_.lock();
    auto secondary = secondary_.lock();

    if (!primary || !secondary) {
        std::cerr << "One of the sessions has expired." << std::endl;
        return;
    }

    if (sess == primary && !cord1.has_value()) {
        cord1 = cord;
    } else if (sess == secondary && !cord2.has_value()) {
        cord2 = cord;
    } else {
        std::cerr << "Session not recognized." << std::endl;
        return;
    }

    if (cord1.has_value() && cord2.has_value()) {
        std::optional<std::array<double, 2>> opt_lsr_cords = find_lsr_cords(cord1.value(), cord2.value());
        cord1.reset();
        cord2.reset();
        if (opt_lsr_cords.has_value()) {
            std::array<double, 2> lsr_cords = opt_lsr_cords.value();

            std::vector<uint8_t> byte_vector(sizeof(double) * lsr_cords.size());
            std::memcpy(byte_vector.data(), lsr_cords.data(), sizeof(double) * lsr_cords.size());

            primary->send_message("LSRC", byte_vector);
        }
    }
}

void Session::on_read(beast::error_code ec, std::size_t) {
    if (ec == websocket::error::closed) {
        return;
    }

    if (ec) {
        std::cerr << "Read error: " << ec.message() << std::endl;
        return;
    }

    while (true) {
        if (buffer_.size() < 8) {
            break;
        }

        auto data = buffer_.data();
        auto buf_iter = boost::asio::buffers_begin(data);

        std::string tag(buf_iter, buf_iter + 4);
        buf_iter += 4;

        uint32_t body_length = 0;
        std::memcpy(&body_length, &(*buf_iter), 4);
        body_length = ntohl(body_length);
        buf_iter += 4;

        if (buffer_.size() < 8 + body_length) {
            break;
        }

        std::vector<uint8_t> body(body_length);
        std::copy_n(buf_iter, body_length, body.begin());
        buf_iter += body_length;

        buffer_.consume(8 + body_length);

        if (tag == "CORD") {
            if (body.size() >= 8) {
                int32_t x = 0, y = 0;
                std::memcpy(&x, body.data(), 4);
                std::memcpy(&y, body.data() + 4, 4);
                x = ntohl(x);
                y = ntohl(y);
                std::array<int32_t, 2> cord = {x, y};

                if (auto pair = pair_.lock()) {
                    pair->update_cord(shared_from_this(), cord);
                }

                send_message("CORD", {});
            } else {
                std::cerr << "Invalid coordinate data size." << std::endl;
            }
        } else {
            std::cerr << "Received unknown tag: " << tag << std::endl;
        }
    }

    do_read();
}

void Session::send_message(const std::string& tag, const std::vector<uint8_t>& body) {
    if (tag.size() != 4) {
        std::cerr << "Tag must be exactly 4 characters." << std::endl;
        return;
    }

    std::vector<uint8_t> message;

    message.insert(message.end(), tag.begin(), tag.end());

    uint32_t body_length = htonl(static_cast<uint32_t>(body.size()));
    uint8_t length_bytes[4];
    std::memcpy(length_bytes, &body_length, 4);
    message.insert(message.end(), length_bytes, length_bytes + 4);

    message.insert(message.end(), body.begin(), body.end());

    net::post(ws_.get_executor(),
              [self = shared_from_this(), msg = std::move(message)]() mutable {
                  bool write_in_progress = !self->write_queue_.empty();
                  self->write_queue_.push_back(std::move(msg));
                  if (!write_in_progress) {
                      self->do_write();
                  }
              });
}

void Session::do_write() {
    if (write_queue_.empty()) {
        return;
    }
    ws_.binary(true);
    auto& msg = write_queue_.front();
    ws_.async_write(
        net::buffer(msg),
        [self = shared_from_this()](beast::error_code ec, std::size_t) {
            self->on_write(ec);
        });
}

void Session::on_write(beast::error_code ec) {
    if (ec) {
        std::cerr << "Write error: " << ec.message() << std::endl;
        return;
    }
    write_queue_.pop_front();
    if (!write_queue_.empty()) {
        do_write();
    }
}

// Server class
class Server {
    tcp::acceptor acceptor_;
    std::unordered_map<std::string, std::shared_ptr<Pair>> pairs_;

   public:
    Server(net::io_context& ioc, tcp::endpoint endpoint)
        : acceptor_(ioc) {
        beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            std::cerr << "Open error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec) {
            std::cerr << "Set option error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.bind(endpoint, ec);
        if (ec) {
            std::cerr << "Bind error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.listen(net::socket_base::max_listen_connections, ec);
        if (ec) {
            std::cerr << "Listen error: " << ec.message() << std::endl;
            return;
        }

        do_accept();
    }

   private:
    void do_accept() {
        acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
            if (!ec) {
                auto session = std::make_shared<Session>(std::move(socket));

                std::string pair_name;
                std::cout << "Input pair name: ";
                std::cin >> pair_name;

                auto it = pairs_.find(pair_name);
                std::shared_ptr<Pair> pair;

                if (it == pairs_.end()) {
                    pair = std::make_shared<Pair>();
                    pairs_[pair_name] = pair;
                } else {
                    pair = it->second;
                }

                session->set_pair(pair);

                int type;
                std::cout << "Input type of session (1 for primary, 2 for secondary): ";
                std::cin >> type;
                pair->add_session(session, type);

                session->start();
            } else {
                std::cerr << "Error accepting connection: " << ec.message() << std::endl;
            }
            do_accept();  // Keep accepting new connections
        });
    }
};

int main() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        auto const port = static_cast<unsigned short>(8080);

        int thread_count = static_cast<int>(std::thread::hardware_concurrency());
        net::io_context ioc{thread_count > 0 ? thread_count : 1};

        auto server = std::make_shared<Server>(ioc, tcp::endpoint{address, port});

        std::vector<std::thread> threads;
        threads.reserve(thread_count);
        for (int i = 0; i < thread_count; ++i) {
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
