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

int main() {
    try {
        boost::asio::io_context io_context;
        tcp::socket socket(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve("10.0.0.235", "12345"));
        cout << "Connected to server" << std::endl;

        // Set TCP_NODELAY to disable Nagle's algorithm
        boost::asio::ip::tcp::no_delay option(true);
        socket.set_option(option);

        boost::asio::socket_base::receive_buffer_size recv_buffer_option(4096);
        socket.set_option(recv_buffer_option);

        boost::asio::socket_base::send_buffer_size send_buffer_option(4096);
        socket.set_option(send_buffer_option);


        while (true) {
            try {
                cv::Mat frame = process_data_to_frame(socket);
                cv::imshow("Video Stream", frame);
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