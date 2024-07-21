#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

void process_data_into_frame(tcp::socket& socket) {
    // Receive frame size
    std::vector<uint8_t> frame_size_buf(4);
    boost::asio::read(socket, boost::asio::buffer(frame_size_buf));
    int frame_size = (frame_size_buf[0] << 24) + (frame_size_buf[1] << 16) +
                     (frame_size_buf[2] << 8) + frame_size_buf[3];

    if (frame_size == 0) {
        throw std::runtime_error("Stream Ended");
    }

    // Receive frame data
    std::vector<uint8_t> img_bytes(frame_size);
    size_t bytes_received = 0;
    while (bytes_received < frame_size) {
        bytes_received += boost::asio::read(socket, boost::asio::buffer(img_bytes.data() + bytes_received, frame_size - bytes_received));
    }

    // Decode frame data
    cv::Mat frame = cv::imdecode(img_bytes, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::cerr << "Failed to decode image" << std::endl;
        return;
    }

    // Display frame
    cv::imshow("Client Video Stream", frame);
    if (cv::waitKey(1) == 'q') {
        throw std::runtime_error("Client closed");
    }
}

int main() {
    try {
        boost::asio::io_context io_context;
        tcp::socket socket(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve("11.22.33.44", "12345"));

        std::cout << "Connected to server" << std::endl;

        while (true) {
            try {
                process_data_into_frame(socket);
            } catch (const std::runtime_error& e) {
                std::cerr << e.what() << std::endl;
                break;
            }
        }

        cv::destroyAllWindows();
        socket.close();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}




