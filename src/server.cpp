#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/dnn.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_io.h>
#include <nlohmann/json.hpp>
#include <fstream>
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

// Forward declaration of classes
class Session;
class Pair;

/// @brief Server class: manages GLaDOS pairs, stroed in the hash map, and listens for additional clients to come in.
class Server : public std::enable_shared_from_this<Server> {
  tcp::acceptor acceptor_; // tcp acceptor
  std::unordered_map<std::string, std::shared_ptr<Pair>> pairs_; // hash map of pairs of clients

public:

  /// @brief Constructor for Server does TCP handshake
  /// @param ioc : input output context for the server.
  /// @param endpoint : makes this computer the endpoint for the tcp connection
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

  /// @brief Adds the session to the proper pair with the proper type
  /// @param sess : shared pointer temporarily owned by the function so it is not degraded to a weak pointer
  /// @param name : hash map key of the GLaDOS pair
  /// @param type : type of the client in the pair
  void config_session(std::shared_ptr<Session> sess, const std::string& name, uint32_t type);

private:

  /// @brief creates session instance and runs the session in an async io context.
  void do_accept();
};

/// @brief Pair Class: the go between for any data that needs to be passed from 
//         one client onto the other or needs to be processed together in some way
class Pair : public std::enable_shared_from_this<Pair> {
  std::weak_ptr<Session> primary_, secondary_; // session pointers need to be upgraded to shared pointers before use in pair.
  std::optional<std::array<uint32_t, 2>> cord1, cord2; // pixel cordinates from the sessions
  bool frame_flag = true; // flag to process frames for facial recognition

public:

  /// @brief Adds session to pair with the proper type
  /// @param sess : session shared pointer
  /// @param type : Either 1 or 2 for primary or secondary respectively
  void add_session(std::shared_ptr<Session> sess, int type);

  /// @brief updates the cordinates if they need to be updated and if cords for both 1 and 2 are present process them to send out
  /// @param sess 
  /// @param cord 
  void update_cord(std::shared_ptr<Session> sess, const std::array<uint32_t, 2>& cord);

  /// @brief gets frame flag from Pair class used to determine if Piar is already processing frame: true means go head to process another frame false means dont process a frame
  /// @return frame flag
  bool get_frame_flag() {
    return frame_flag;
  }

  /// @brief Used to update the frame flag on the pair and to update the frame flags on the server so that they dont continue to send frames
  /// @param flag_type : boolean the frame flag is set to.
  void update_frame_flag(bool flag_type);

private:
  /// @brief process the pixel cords to find the point in 3d space and converts that to angles to send to the laser.
  /// @param px1 : pixel cord from primary client
  /// @param px2 : pixel cord from secondary client
  /// @return return the lsr angles in the modified spherical cordinates coresponding to the servo configuration
  std::optional<std::array<double, 2>> find_lsr_cords(const std::array<uint32_t, 2>& px1, const std::array<uint32_t, 2>& px2) {
    // temporary math for finding the point in 3d space and then converting to servo cordinate system
    double x = (static_cast<double>(px1[0]) + static_cast<double>(px1[1])) / 2;
    double y = (static_cast<double>(px2[0]) + static_cast<double>(px2[1])) / 2;

    std::optional<std::array<double, 2>> lsr_cords;
    lsr_cords = { x, y };
    return lsr_cords;
  }
};

/// @brief Session Class: Handles all data coming from a specific client
class Session : public std::enable_shared_from_this<Session> {
  websocket::stream<tcp::socket> ws_; // websocket connection
  std::weak_ptr<Pair> pair_; // unowned pointer to the pair class that the client is apart of
  std::weak_ptr<Server> server_; // unowned pointer to the server

  beast::flat_buffer buffer_; // contigous memory that the server and client right too 

  std::deque<std::vector<uint8_t>> write_queue_; // queue for managing messages being sent to the client as only one message can be sent at a time in various asychronous settings
public:
  explicit Session(tcp::socket socket, std::shared_ptr<Server> server)
    : ws_(std::move(socket)), server_(server) {
  }

  /// @brief starts the session on the server
  void start() {
    ws_.async_accept(beast::bind_front_handler(&Session::on_accept, shared_from_this()));
  }

  /// @brief Sends a message identifying the type of message and the content
  /// @param tag : 4 charachter classifier of message type
  /// @param body : content of the message in raw byte form
  void send_message(const std::string& tag, const std::vector<uint8_t>& body);

  /// @brief Sets the pair of the session
  /// @param pair : pointer to the pair class
  void set_pair(std::shared_ptr<Pair> pair) {
    pair_ = pair;
  }

private:

  /// @brief accepts connection to that specific client
  /// @param ec 
  void on_accept(beast::error_code ec) {
    if (ec) {
      std::cerr << "WebSocket accept failed: " << ec.message() << std::endl;
      return;
    }
    do_read();
  }

  /// @brief does an asycrounous read of the shared buffer in the threaded io context then passes that along to on_read
  void do_read() {
    ws_.async_read(buffer_, beast::bind_front_handler(&Session::on_read, shared_from_this()));
  }

  /// @brief process data from the buffer
  /// @param ec 
  /// @param  
  void on_read(beast::error_code ec, std::size_t);

  /// @brief Does a write to the shared buffer 
  void do_write();

  /// @brief manages the further messages after the write is compleated
  /// @param ec 
  void on_write(beast::error_code ec);
};

void Pair::add_session(std::shared_ptr<Session> sess, int type) {
  if (type == 1) {
    if (primary_.expired()) {
      primary_ = sess;
      std::cout << "Session added to pair as primary." << std::endl;
    }
    else {
      std::cerr << "This pair already has a primary session.";
    }
  }
  else if (type == 2) {
    if (secondary_.expired()) {
      secondary_ = sess;
      std::cout << "Session added to pair as secondary." << std::endl;
    }
    else {
      std::cerr << "This pair already has a secondary session.";
    }
  }
  else {
    std::cerr << "Given type is not either 1 or 2.";
  }
}

void Pair::update_cord(std::shared_ptr<Session> sess, const std::array<uint32_t, 2>& cord) {
  if (auto primary = primary_.lock()) {
    if (auto secondary = secondary_.lock()) {
      if (sess == primary) {
        if (!cord1.has_value()) {
          cord1 = cord;
        }
      }
      else if (sess == secondary) {
        if (!cord2.has_value()) {
          cord2 = cord;
        }
      }
      else {
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
  }
}

void Pair::update_frame_flag(bool flag_type) {
  frame_flag = flag_type;
  std::string tag = flag_type ? "YFRM" : "NFMR";

  if (auto primary = primary_.lock()) {
    if (auto secondary = secondary_.lock()) {
      primary->send_message(tag, {});
      secondary->send_message(tag, {});
    }
  }
}

void Server::do_accept() {
  acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
    if (!ec) {
      auto session = std::make_shared<Session>(std::move(socket), shared_from_this());
      session->start();
    }
    else {
      std::cerr << "Error accepting connection: " << ec.message() << std::endl;
    }
    do_accept();
    });
}

void Server::config_session(std::shared_ptr<Session> sess, const std::string& name, uint32_t type) {
  std::shared_ptr<Pair> pair;

  auto it = pairs_.find(name);
  if (it == pairs_.end()) {
    pair = std::make_shared<Pair>();
    pairs_[name] = pair;
  }
  else {
    pair = it->second;
  }

  sess->set_pair(pair);
  pair->add_session(sess, type);
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

    if (tag == "CONF") {
      std::string name(body.begin() + 4, body.end());

      uint32_t type = 0;
      std::memcpy(&type, body.data(), 4);
      type = ntohl(type);

      if (auto server = server_.lock()) {
        server->config_session(shared_from_this(), name, type);
      }
    }
    else if (tag == "CORD") {
      if (body.size() >= 8) {
        uint32_t x = 0, y = 0;
        std::memcpy(&x, body.data(), 4);
        std::memcpy(&y, body.data() + 4, 4);
        x = ntohl(x);
        y = ntohl(y);
        std::array<uint32_t, 2> cord = { x, y };

        if (auto pair = pair_.lock()) {
          pair->update_cord(shared_from_this(), cord);
        }

        send_message("CORD", {});
      }
      else {
        std::cerr << "Invalid coordinate data size." << std::endl;
      }
    }
    else if (tag == "FRAM") {
      if (auto pair = pair_.lock()) {
        if (pair->get_frame_flag()) {
          cv::Mat img = cv::imdecode(body, cv::IMREAD_COLOR);
          if (img.empty()) {
            std::cerr << "Failed to decode image" << std::endl;
          }
          cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
          dlib::cv_image<dlib::rgb_pixel> dlib_img(img);
          dlib::matrix<dlib::rgb_pixel> frame;
          dlib::assign_image(frame, dlib_img);

          pair->update_frame_flag(false);

          // do facial detection on asychronously in a non-blocking manner
        }
      }
    }
    else {
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

int main() {
  try {
    auto const address = net::ip::make_address("0.0.0.0");
    auto const port = static_cast<unsigned short>(8080);

    int thread_count = static_cast<int>(std::thread::hardware_concurrency());
    net::io_context ioc{ thread_count > 0 ? thread_count : 1 };

    auto server = std::make_shared<Server>(ioc, tcp::endpoint{ address, port });

    std::vector<std::thread> threads;
    threads.reserve(thread_count);
    for (int i = 0; i < thread_count; ++i) {
      threads.emplace_back([&ioc] { ioc.run(); });
    }

    for (auto& t : threads) {
      t.join();
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
