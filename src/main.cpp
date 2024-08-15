#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

cv::Mat process_data_into_frame(tcp::socket& socket) {
    // Receive frame size
    vector<uint8_t> frame_size_buf(4);
    boost::asio::read(socket, boost::asio::buffer(frame_size_buf));
    unsigned long frame_size = (frame_size_buf[0] << 24) + (frame_size_buf[1] << 16) +
                     (frame_size_buf[2] << 8) + frame_size_buf[3];

    if (frame_size == 0) {
        throw runtime_error("Stream Ended on Server");
    }

    // Receive frame data
    vector<uint8_t> img_bytes(frame_size);
    size_t bytes_received = 0;
    while (bytes_received < frame_size) {
        bytes_received += boost::asio::read(socket, boost::asio::buffer(img_bytes.data() + bytes_received, frame_size - bytes_received));
    }

    // Decode frame data
    cv::Mat frame = cv::imdecode(img_bytes, cv::IMREAD_COLOR);
    if (frame.empty()) {
        throw runtime_error("Failed to decode image");
    }

    return frame;
}

int main() {
    //Web socket
    try {
        boost::asio::io_context io_context;
        boost::asio::ip::tcp::socket socket(io_context);
        boost::asio::ip::tcp::resolver resolver(io_context);
        boost::asio::connect(socket, resolver.resolve("10.0.0.235", "12345"));
        cout << "Connected to server" << std::endl;

        while (true) {
            try {
                cv::Mat frame = process_data_into_frame(socket);

                try {
                    cv::imshow("Video Stream", frame);
                    if(cv::waitKey(1) == 'q'){
                        throw runtime_error("Stream Ended by User on Client");
                    }
                } catch (const cv::Exception &e){
                    cerr << e.what() << endl;
                }
            } catch (const std::runtime_error& e) {
                cerr << e.what() << endl;
                break;
            }
        }

        cv::destroyAllWindows();
        socket.close();
    } catch (const std::exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}

