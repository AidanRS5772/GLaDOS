#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <vector>

using boost::asio::ip::tcp;
using namespace std;

cv::Mat process_data_to_frame(tcp::socket& socket) {
    uint64_t frame_size;
    boost::asio::read(socket, boost::asio::buffer(&frame_size, sizeof(frame_size)));

    std::vector<uint8_t> buffer(frame_size);
    boost::asio::read(socket, boost::asio::buffer(buffer.data(), buffer.size()));

    // Decode the received JPG image to an OpenCV Mat
    cv::Mat frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (frame.empty()) {
        throw runtime_error("Failed to decode the frame.");
    }

    return frame;
}

void send_acknowledgment(tcp::socket& socket) {
    const std::string ack_msg = "ACK";
    boost::asio::write(socket, boost::asio::buffer(ack_msg));
}

array<int, 2> find_center_of_max_contour(const vector<std::vector<cv::Point>> &contours, cv::Mat &frame){
    int max_area = 0;
    cv::Rect bounding_rect, max_bounding_rect;
    for (long unsigned int i = 0; i < contours.size(); i++) {
        bounding_rect = cv::boundingRect(contours[i]);
        int area = bounding_rect.area();
        if (max_area < area){
            max_bounding_rect = bounding_rect;
            max_area = area;
        }
    }

    int Laser_X = max_bounding_rect.x + max_bounding_rect.width/2;
    int Laser_Y = max_bounding_rect.y + max_bounding_rect.height/2;

    array<int, 2> px_cords = {Laser_X, Laser_Y};

    cv::circle(frame, cv::Point(Laser_X, Laser_Y), 5, cv::Scalar(255, 0, 0), -1);
    return px_cords;
}

const string IP = "10.0.0.235";
const string PORT = "12345";

const int PRE_THRESH = 400; //Threshold for KNN background subtractor
const int FRAME_HIST = 100; //Frame History of KNN background subtractor

const int L_KERNAL_SZ = 7; //Size of convolution kernal for large morphology processing
const int S_KERNAL_SZ = 7; //Size of convolution kernal for large morphology processing

const int POST_THRESH = 50; //Gray scale limit for post thresh


int main() {
    int frame_cnt = 0;

    cv::Ptr<cv::BackgroundSubtractorKNN> KNN = cv::createBackgroundSubtractorKNN(FRAME_HIST, PRE_THRESH, true);
    cv::Mat frame, fg_mask, clean_fg_mask;
    vector<std::vector<cv::Point>> contours;
    cv::Mat kernel_L = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(L_KERNAL_SZ, L_KERNAL_SZ));
    cv::Mat kernel_S = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(S_KERNAL_SZ, S_KERNAL_SZ));



    try {
        boost::asio::io_context io_context;
        tcp::socket socket(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve(IP, PORT));
        cout << "Connected to server" << std::endl;

        while (true) {
            try {
                frame = process_data_to_frame(socket);
                frame_cnt ++;
                cout << "Processed Frames: " << frame_cnt;

                KNN->apply(frame, fg_mask);

                cv::threshold(fg_mask, clean_fg_mask, POST_THRESH, 255, cv::THRESH_BINARY);
                cv::morphologyEx(clean_fg_mask, clean_fg_mask, cv::MORPH_OPEN, kernel_S);
                cv::morphologyEx(clean_fg_mask, clean_fg_mask, cv::MORPH_CLOSE, kernel_L);

                cv::findContours(clean_fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                find_center_of_max_contour(contours, frame);

                cv::imshow("Video Stream", frame);
                cv::imshow("Background Subtraction", fg_mask);
                cv::imshow("Background Subtraction And Clean", clean_fg_mask);

                send_acknowledgment(socket);

                cout << flush;

                if (cv::waitKey(1) == 'q') {
                    throw std::runtime_error("Stream Ended by User on Client");
                }
            } catch (const std::exception &e) {
                std::cerr << e.what() << std::endl;
                break;
            }
        }

        cv::destroyAllWindows();
        socket.close();
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}