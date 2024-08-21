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

int main() {
    cv::Ptr<cv::BackgroundSubtractorKNN> knn = cv::createBackgroundSubtractorKNN(1, 400, false);

    try {
        boost::asio::io_context io_context;
        tcp::socket socket(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve("10.0.0.235", "12345"));
        cout << "Connected to server" << std::endl;

        while (true) {
            try {
                cv::Mat frame = process_data_to_frame(socket);

                cv::Mat sub_bckgrnd_frame;
                knn->apply(frame, sub_bckgrnd_frame);

                cv::imshow("Video Stream", frame);
                cv::imshow("Background Subtraction", sub_bckgrnd_frame);

                send_acknowledgment(socket);
                
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