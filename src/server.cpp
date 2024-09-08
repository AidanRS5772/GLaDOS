#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

class session_manager;

// Adjustment Numbers
const int PRE_THRESH = 400;    // Threshhold for KNN background subtractor
const int FRAME_HIST = 120;    // Length of frame history for KNN background subtractor
const int L_KERNAL_SZ = 7;     // Size of convolution kernal for large morphologies
const int S_KERNAL_SZ = 3;     // Size of convolution kernal for small morphologies
const int POST_THRESH = 50;    // Gray scale limit for post thresholding of mask
const int AREA_THRESH = 5000;  // Minimum threshold for identifying object

class session : public std::enable_shared_from_this<session> {
    boost::beast::websocket::stream<boost::beast::tcp_stream> ws_;
    boost::beast::flat_buffer buffer_;
    string client_id_;
    session_manager &manager_;

    // Motion Detection
    cv::Ptr<cv::BackgroundSubtractorKNN> KNN;
    cv::Mat frame_, fg_mask_, clean_fg_mask_;

    // Image Clean Up
    cv::Mat kernal_L;
    cv::Mat kernal_S;

    vector<vector<cv::Point>> contours_;

   public:
    explicit session(boost::asio::ip::tcp::socket &&socket, session_manager &manager) : ws_(std::move(socket)),
                                                                                        manager_(manager),
                                                                                        KNN(cv::createBackgroundSubtractorKNN(FRAME_HIST, PRE_THRESH, true)),
                                                                                        kernal_L(cv::getStructuringElement(cv::MORPH_RECT, cv::Size(L_KERNAL_SZ, L_KERNAL_SZ))),
                                                                                        kernal_S(cv::getStructuringElement(cv::MORPH_RECT, cv::Size(S_KERNAL_SZ, S_KERNAL_SZ))) {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        client_id_ = boost::uuids::to_string(uuid);

        boost::asio::ip::tcp::no_delay option(true);
        ws_.next_layer().socket().set_option(option);
    }

    void run() {
        ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
        auto self = shared_from_this();
        ws_.async_accept(boost::beast::bind_front_handler(&session::on_accept, self));
    }

    void handle_close(const std::string &reason);

    string get_client_id() {
        return client_id_;
    }

   private:
    void on_accept(boost::beast::error_code ec);

    // Forms recursive loop with on_read()
    void do_read() {
        auto self = shared_from_this();

        ws_.async_read(buffer_, [self](boost::beast::error_code ec, size_t bytes_transferred) {
            self->on_read(ec, bytes_transferred);
        });
    }

    // Main event function this is where the action happens
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);

    void send_ack_signal() {
        auto self = shared_from_this();
        string msg = "ACK";
        ws_.text(true);  // Ensure we're sending a text message
        ws_.async_write(boost::asio::buffer(msg), [self](boost::beast::error_code ec, std::size_t) {
            if (ec) {
                cerr << "Error sending ready signal: " << ec.message() << endl;
                return;
            }

            // Now wait for the next frame from the client
            self->do_read();
        });
    }

    // processes raw binary into cv::Mat
    void process_data_to_frame(boost::beast::flat_buffer &buffer, cv::Mat &frame) {
        auto data = boost::asio::buffer_cast<const uint8_t *>(buffer.data());
        size_t data_size = buffer.size();

        vector<uint8_t> frame_data(data, data + data_size);

        frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);

        if (frame.empty()) {
            throw runtime_error("Failed to decode the frame.");
        }

        buffer.consume(buffer.size());
    }

    // find largest countour and draw rect to frame
    optional<cv::Rect> find_motion(cv::Mat &frame) {
        KNN->apply(frame_, fg_mask_);
        cv::threshold(fg_mask_, clean_fg_mask_, POST_THRESH, 255, cv::THRESH_BINARY);
        cv::morphologyEx(clean_fg_mask_, clean_fg_mask_, cv::MORPH_OPEN, kernal_S);
        cv::morphologyEx(clean_fg_mask_, clean_fg_mask_, cv::MORPH_CLOSE, kernal_L);

        cv::findContours(clean_fg_mask_, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int max_area = 0;
        cv::Rect bounding_rect, max_bounding_rect;

        for (long unsigned int i = 0; i < contours_.size(); i++) {
            bounding_rect = cv::boundingRect(contours_[i]);
            int area = bounding_rect.area();
            if (max_area < area) {
                max_bounding_rect = bounding_rect;
                max_area = area;
            }
        }

        if (max_area > AREA_THRESH) {
            cv::rectangle(frame, max_bounding_rect, cv::Scalar(0, 255, 0), 2);
            return max_bounding_rect;
        }
        return nullopt;
    }

    void close_connection() {
        if (!ws_.is_open()) {
            cout << "WebSocket is already closed for client: " << client_id_ << endl;
            return;
        }

        cout << "Starting Server Close of Client: " << client_id_ << endl;

        auto self = shared_from_this();
        ws_.async_write(boost::asio::buffer(""), [self](boost::beast::error_code ec, std::size_t) {
            if (ec) {
                cerr << "Error flushing data: " << ec.message() << endl;
                return;
            }

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

struct client_pair {
    shared_ptr<session> primary;
    shared_ptr<session> secondary;

    optional<cv::Rect> primary_motion;
    optional<cv::Rect> secondary_motion;

    client_pair(std::shared_ptr<session> primary_client = nullptr, std::shared_ptr<session> secondary_client = nullptr)
        : primary(primary_client), secondary(secondary_client) {}

    void add_primary(std::shared_ptr<session> primary_client) {
        if (primary == nullptr) {
            primary = primary_client;
            std::cout << "Primary client added." << std::endl;
        } else {
            std::cerr << "Primary client already exists!" << std::endl;
        }
    }

    void add_secondary(std::shared_ptr<session> secondary_client) {
        if (secondary == nullptr) {
            secondary = secondary_client;
            std::cout << "Secondary client added." << std::endl;
        } else {
            std::cerr << "Secondary client already exists!" << std::endl;
        }
    }
};

class session_manager {
    mutex mutex_;
    unordered_map<string, client_pair> pairs_;

   public:
    void add_session(const string &client_id, shared_ptr<session> ses) {
        lock_guard<mutex> lock(mutex_);

        string client_tag;
        cout << "Add Client with Id:" << client_id << "to pair (pair_name-1or2): " << endl;
        cin >> client_tag;

        size_t pos = client_tag.find('-');

        if (pos != std::string::npos) {
            string name = client_tag.substr(0, pos);
            int num = stoi(client_tag.substr(pos + 1));

            if ((num != 1) && (num != 2)) {
                cerr << "Invalid input format! Number is not 1 or 2." << endl;
            }

            auto &pair = pairs_[name];

            if (num == 1) {
                pair.add_primary(ses);
            } else {
                pair.add_secondary(ses);
            }
        } else {
            cerr << "Invalid input format!" << endl;
        }
    }

    void remove_session(const std::string &client_id) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Iterate over all pairs and check for the session to remove
        for (auto &pair_entry : pairs_) {
            auto &pair = pair_entry.second;

            if (pair.primary && pair.primary->get_client_id() == client_id) {
                std::cout << "Removing primary client: " << client_id << " from pair: " << pair_entry.first << std::endl;
                pair.primary.reset();
            } else if (pair.secondary && pair.secondary->get_client_id() == client_id) {
                std::cout << "Removing secondary client: " << client_id << " from pair: " << pair_entry.first << std::endl;
                pair.secondary.reset();
            }
        }
    }

    void motion_detected(const string &client_id, optional<cv::Rect> motion){
        lock_guard<mutex> lock(mutex_);

        for (auto &pair_entry : pairs_) {
            auto &pair = pair_entry.second;

            if (pair.primary && pair.primary->get_client_id() == client_id) {
                pair.primary_motion = motion;
            } else if (pair.secondary && pair.secondary->get_client_id() == client_id) {
                pair.secondary_motion = motion;
            }

            if (pair.primary_motion.has_value() && pair.secondary_motion.has_value()) {
                double primary_cx = static_cast<double>(pair.primary_motion->x) + static_cast<double>(pair.primary_motion->width/2);
                double primary_cy = static_cast<double>(pair.primary_motion->y) + static_cast<double>(pair.primary_motion->height/2);

                cout << "Primary motion detection ~ (x: " << primary_cx << " , y: " << primary_cy << ")" << endl;

                double secondary_cx = static_cast<double>(pair.secondary_motion->x) + static_cast<double>(pair.secondary_motion->width/2);
                double secondary_cy = static_cast<double>(pair.secondary_motion->y) + static_cast<double>(pair.secondary_motion->height/2);

                cout << "Primary motion detection ~ (x: " << secondary_cx << " , y: " << secondary_cy << ")" << endl;

                
                pair.primary_motion.reset();
                pair.secondary_motion.reset();
            }
        }
    }
};

//These need to be right here for procedural compiling of the interdependent classes session and session manager

void session::on_accept(boost::beast::error_code ec) {
    if (ec) {
        cerr << "Accept error: " << ec.message() << endl;
        return;
    }
    cout << "Client connected with ID: " << client_id_ << endl;

    auto self = shared_from_this();
    manager_.add_session(client_id_, self);

    do_read();
}

void session::handle_close(const string &reason) {
    cout << reason << client_id_ << endl;

    cv::destroyWindow(client_id_);
    cv::waitKey(1);  // Neded to close window

    manager_.remove_session(client_id_);

    ws_.next_layer().close();
}

void session::on_read(boost::beast::error_code ec, std::size_t bytes_transferred){
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
            process_data_to_frame(buffer_, frame_);
            auto motion = find_motion(frame_);
            if (motion.has_value()) {
            } else {
            }

            cv::imshow(client_id_, frame_);

            if (cv::waitKey(1) == 'q') {
                close_connection();
                return;
            }

            send_ack_signal();
        } catch (const std::exception &e) {
            cerr << "Frame processing error: " << e.what() << endl;
            handle_close("Closing due to frame processing error: ");
        }
    }

    // Continue to next frame
    do_read();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

class listener : public std::enable_shared_from_this<listener> {
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    session_manager &manager_;

   public:
    listener(boost::asio::io_context &ioc, boost::asio::ip::tcp::endpoint endpoint, session_manager &manager) : acceptor_(ioc), socket_(ioc), manager_(manager) {
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
                std::make_shared<session>(std::move(self->socket_), self->manager_)->run();
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

        boost::asio::ip::tcp::endpoint endpoint{boost::asio::ip::make_address(ip_address), port};

        session_manager manager;

        auto listener_ptr = make_shared<listener>(ioc, endpoint, manager);

        listener_ptr->do_accept();

        ioc.run();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}