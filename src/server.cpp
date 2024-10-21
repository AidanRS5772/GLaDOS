#include <dlib/dnn.h>
#include <dlib/image_io.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>

#include <Eigen/Dense>
#include <array>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rapidcsv.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;
using json = nlohmann::json;

template <template <int, template <typename> class, int, typename> class block, int N, template <typename> class BN, typename SUBNET>
using residual = add_prev1<block<N, BN, 1, tag1<SUBNET>>>;

template <template <int, template <typename> class, int, typename> class block, int N, template <typename> class BN, typename SUBNET>
using residual_down = add_prev2<avg_pool<2, 2, 2, 2, skip1<tag2<block<N, BN, 2, tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET>
using block = BN<con<N, 3, 3, 1, 1, relu<BN<con<N, 3, 3, stride, stride, SUBNET>>>>>;

template <int N, typename SUBNET>
using ares = relu<residual<block, N, affine, SUBNET>>;
template <int N, typename SUBNET>
using ares_down = relu<residual_down<block, N, affine, SUBNET>>;

template <typename SUBNET>
using alevel0 = ares_down<256, SUBNET>;
template <typename SUBNET>
using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template <typename SUBNET>
using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template <typename SUBNET>
using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template <typename SUBNET>
using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = loss_metric<fc_no_bias<128, avg_pool_everything<
                                                  alevel0<
                                                      alevel1<
                                                          alevel2<
                                                              alevel3<
                                                                  alevel4<
                                                                      max_pool<3, 3, 2, 2, relu<affine<con<32, 7, 7, 2, 2, input_rgb_image_sized<150>>>>>>>>>>>>>;

class Server;

class Session : public std::enable_shared_from_this<Session> {
    websocket::stream<tcp::socket> ws_;
    beast::flat_buffer buffer_;
    std::shared_ptr<Pair> pair_;

   public:
    explicit Session(tcp::socket socket, std::shared_ptr<Pair> pair)
        : ws_(std::move(socket)), pair_(std::move(pair)) {}

    void start() {
        ws_.async_accept(beast::bind_front_handler(&Session::on_accept, shared_from_this()));
    }

    void send_raw_data(const void* data, std::size_t size) {
        ws_.async_write(
            net::buffer(data, size),
            [self = shared_from_this(), size](beast::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error sending raw data: " << ec.message() << std::endl;
                }
            });
    }

   private:
    void on_accept(beast::error_code ec) {
        if (ec) {
            std::cerr << "Error during handshake: " << ec.message() << std::endl;
            return;
        }

        do_read();
    }

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
        } else if (tag == "FRAM" && pair_.get_frame_flag()) {
            handle_frame();
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
            buffer_.consume(8);

            std::cout << "Received Coordinates: (" << int1 << ", " << int2 << ")" << std::endl;

            std::array<int, 2> cords = {int1, int2};
            pair_->update_cord(shared_from_this(), cords);

            std::string msg = "CACK";
            send_raw_data(msg.data(), msg.size());
        } else {
            std::cerr << "Not enough data for coordinates (expected 8 bytes)" << std::endl;
            buffer_.consume(buffer_.size());
        }
    }

    void handle_frame() {
        if (buffer_.size() > 0) {
            std::vector<uchar> jpeg_data(static_cast<const uchar*>(buffer_.data().data()),
                                         static_cast<const uchar*>(buffer_.data().data()) + buffer_.size());
            cv::Mat cvFrame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if (!img.empty()) {
                cv::Mat cvFrameRGB;
                cv::cvtColor(cvFrame, cvFrameRGB, cv::COLOR_BGR2RGB);
                dlib::cv_image<dlib::rgb_pixel> dlibImage(cvFrameRGB);
                dlib::matrix<dlib::rgb_pixel> dlibMatrix;
                dlib::assign_image(dlibMatrix, dlibImage);

                pair_.update_frame_flag(false);

                // do facial recognition
            } else {
                std::cerr << "Failed to decode the image!" << std::endl;
            }
            buffer_.consume(buffer_.size());
        } else {
            std::cerr << "No image data received!" << std::endl;
        }
    }
};

class Pair {
    cv::Mat C1, C2;

    const float lsr_angl = M_PI / 4;
    const float tol = 16.0;

    dlib::frontal_face_detector dectector;
    dlib::shape_predictor sp;
    anet_type net;

    std::vector<std::string> names;
    std::vector<Eigen::VectorXd> mean_vecs;
    std::vector<Eigen::MatrixXd> inv_conv_mats;

    std::mutex mutex_;
    std::shared_ptr<Session> primary_, secondary_;
    std::optional<std::array<int, 2>> primary_cord_, secondary_cord_;
    bool frame_flag;

   public:
    Pair() : primary_(nullptr), secondary_(nullptr), frame_flag(true) {
        rapidcsv::Document doc("../../../src/calibrations.csv");

        std::vector<float> combined;
        std::vector<std::string> columns = {"col1", "col2", "col3", "col4"};
        for (const auto& col : columns) {
            const auto& colData = doc.GetColumn<float>(col);
            combined.insert(combined.end(), colData.begin(), colData.end());
        }

        C1 = cv::Mat(combined).rowRange(0, 12).reshape(1, 3);
        C2 = cv::Mat(combined).rowRange(12, 24).reshape(1, 3);

        detector = dlib::get_frontal_face_detector();
        dlib::deserialize("../../../src/face_recog_DNN/shape_predictor_68_face_landmarks.dat") >> sp;
        dlib::deserialize("../../../src/face_recog_DNN/dlib_face_recognition_resnet_model_v1.dat") >> net;

        std::ifstream input_file("data.json");
        if (!input_file.is_open()) {
            std::cerr << "Error opening file data.json" << std::endl;
        }

        json j;
        input_file >> j;
        for (auto& [name, vectors_json] : j.items()) {
            names.push_back(name);

            std::vector<Eigen::VectorXd> embedding_vecs;
            for (const auto& vector_json : vectors_json) {
                std::vector<double> vec = vector_json.get<std::vector<double>>();
                Eigen::VectorXd eigen_vec = Eigen::VectorXd::Map(vec.data(), vec.size());
                embedding_vecs.push_back(eigen_vec);
            }

            int D = embedding_vecs[0].size();

            Eigen::VectorXd mean_vec = Eigen::VectorXd::Zero(D);
            for (const auto& vec : embedding_vecs) {
                mean_vec += vec;
            }
            mean_vec /= static_cast<double>(embedding_vecs.size());
            mean_vecs.push_back(mean_vec);

            Eigen::MatrixXd cov_mat = Eigen::MatrixXd::Zero(D, D);
            for (const auto& vec : embedding_vecs) {
                Eigen::VectorXd diff = vec - mean_vec;
                cov_mat += diff * diff.transpose();
            }
            cov_mat /= static_cast<double>(embedding_vecs.size() - 1);

            double ep = 1e-5 * cov_mat.trace() / D;
            cov_mat += ep * Eigen::MatrixXd::Identity(D, D);

            Eigen::MatrixXd inv_cov_mat = cov_mat.inverse();
            inv_cov_mats.push_back(inv_cov_mat);
        }
    }

    void add_session(std::shared_ptr<Session> sess, int type) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (type == 1) {
            if (!primary_) {
                primary_ = std::move(sess);
                std::cout << "Primary session added to pair." << std::endl;
            } else {
                std::cerr << "This Pair already has a primary session." << std::endl;
            }
        } else if (type == 2) {
            if (!secondary_) {
                secondary_ = std::move(sess);
                std::cout << "Secondary session added to pair." << std::endl;
            } else {
                std::cerr << "This Pair already has a secondary session." << std::endl;
            }
        } else {
            std::cerr << "Invalid session type, expected 1 or 2." << std::endl;
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
            std::optional<std::array<float, 2>> opt_servo_cords = find_servo_cords(primary_cord_.value(), secondary_cord_.value());
            if (opt_servo_cords.has_value()) {
                std::array<float, 2> servo_cords = opt_servo_cords.value();
                std::vector<char> buffer(4 + sizeof(float) * 2);

                std::string tag = "CORD";
                std::memcpy(buffer.data(), tag.data(), 4);

                std::memcpy(buffer.data() + 4, &servo_cords[0], sizeof(float));
                std::memcpy(buffer.data() + 4 + sizeof(float), &servo_cords[1], sizeof(float));

                primary_->send_raw_data(buffer.data(), buffer.size());
            }

            primary_cord_.reset();
            secondary_cord_.reset();
        }
    }

    bool get_frame_flag() {
        std::lock_guard<std::mutex> lock(mutex_);
        return frame_flag;
    }

    void update_frame_flag(bool flag) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_flag = flag;
        std::string tag = flag ? "YFRM" : "NFRM";

        primary_->send_raw_data(tag.data(), tag.size());
        secondary_->send_raw_data(tag.data(), tag.size());
    }

    void face_recog(dlib::matrix<dlib::rgb_pixel> mat) {
        std::vector<dlib::rectangle> face_dets = detector(mat);

        if (face_dets.size() == 0) {
            update_frame_flag(true);
            return;
        }

        std::vector<dlib::matrix<dlib::rgb_pixel>> face_chips;
        for (const auto& rect : face_dets) {
            auto shape = sp(mat, *rect);
            dlib::matrix<dlib::rgb_pixel> face_chip;
            dlib::extract_image_chip(mat, dlib::get_face_chip_details(shape, 150, 0.25), face_chip);
            face_chips.push_back(std::move(face_chip));
        }

        std::vector<dlib::matrix<float, 0, 1>> face_decs = net(face_chips);
        std::vector<Eigen::VectorXd> eigen_vecs;

        for (const auto& vec: face_decs) {
            Eigen::Map<const Eigen::VectorXf> eigen_vec_f(dlib_vec.begin(), vec.size());
            Eigen::VectorXd eigen_vec_d = eigen_vec_f.cast<double>();
            eigen_vectors.push_back(eigen_vec_d);
        }

        int max_name_idx = 0;
        double max_mdist = 0;
        for(int i=0; i < names.size(); i++){
            for(int j=0; j < eigen_vecs.size(); j++){
                Eigen::VectorXd diff = eigen_vecs[j] - mean_vecs[j];
            }
        }
    }

   private:
    std::optional<std::array<float, 2>> find_servo_cords(std::array<int, 2> p1, std::array<int, 2> p2) {
        cv::Point2f point1(static_cast<float>(p1[0]), static_cast<float>(p1[1]));
        cv::Point2f point2(static_cast<float>(p2[0]), static_cast<float>(p2[1]));

        cv::Mat points4D;
        std::vector<cv::Point2f> points1 = {point1};
        std::vector<cv::Point2f> points2 = {point2};
        cv::triangulatePoints(C1, C2, points1, points2, points4D);

        points4D /= points4D.at<float>(3, 0);
        cv::Point3f point3d(points4D.at<float>(0, 0), points4D.at<float>(1, 0), points4D.at<float>(2, 0));

        cv::Mat reprojected1 = C1 * points4D;
        cv::Mat reprojected2 = C2 * points4D;

        reprojected1 /= reprojected1.at<float>(2, 0);
        reprojected2 /= reprojected2.at<float>(2, 0);

        cv::Point2f reprojectedPt1(reprojected1.at<float>(0, 0), reprojected1.at<float>(1, 0));
        cv::Point2f reprojectedPt2(reprojected2.at<float>(0, 0), reprojected2.at<float>(1, 0));

        float err1 = cv::norm(reprojectedPt1 - point1);
        float err2 = cv::norm(reprojectedPt2 - point2);

        if (err1 < tol && err2 < tol) {
            point3d /= cv::norm(point3d);
            float x_norm = std::sqrt(1 - point3d.x * point3d.x);

            // Correct way to return an optional array
            return std::make_optional<std::array<float, 2>>(
                {std::asin(point3d.x / std::cos(lsr_angl)),
                 std::asin(point3d.y / x_norm) + std::acos(std::sin(lsr_angl) / x_norm)});
        } else {
            return std::nullopt;
        }
    }
};

class Server : public std::enable_shared_from_this<Server> {
    tcp::acceptor acceptor_;
    std::unordered_map<std::string, std::shared_ptr<Pair>> pairs_;

   public:
    Server(net::io_context& ioc, tcp::endpoint endpoint)
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
                    std::cout << "Input pair name: ";
                    std::cin >> pair_name_;

                    std::shared_ptr<Pair> pair;
                    auto it = pairs_.find(name);
                    if (it == pairs_.end()) {
                        pair = std::make_shared<Pair>();
                        pairs_[name] = pair;
                    } else {
                        pair = it->second;
                    }
                    std::shared_ptr<Session> sess = std::make_shared<Session>(std::move(socket), pair);

                    int type;
                    std::cout << "Input type of session (1 for primary, 2 for secondary): ";
                    std::cin >> type;
                    pair.add_session(sess, type);

                    sess->start();
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