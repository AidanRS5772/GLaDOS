#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

void test_display(){
    cv::Mat image = cv::imread("/home/aidan/Programming/GLaDOS/src/test.jpg");
    if (image.empty()) {
        cerr << "Could not open or find the image!" << endl;
    }

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", image);
    cv::waitKey(0); // Wait for a keystroke in the window
}

void opencv_build_info(){
    cout << "OpenCV Version: " << CV_VERSION << endl;
    cout << "OpenCV Build Info: " << cv::getBuildInformation() << endl;
}

cv::Mat process_data_into_frame(boost::asio::ip::tcp::socket& socket) {
    // Buffer to store incoming data
    vector<uint8_t> buffer;

    // Read until "END_STREAM" is received
    while (true) {
        vector<uint8_t> frame_size_buf(4);
        boost::asio::read(socket, boost::asio::buffer(frame_size_buf));

        unsigned long frame_size = (frame_size_buf[0] << 24) + (frame_size_buf[1] << 16) +
                                   (frame_size_buf[2] << 8) + frame_size_buf[3];

        if (frame_size == 0) {
            throw runtime_error("Stream Ended on Server");
        }

        vector<uint8_t> img_bytes(frame_size);
        size_t bytes_received = 0;
        while (bytes_received < frame_size) {
            bytes_received += boost::asio::read(socket, boost::asio::buffer(img_bytes.data() + bytes_received, frame_size - bytes_received));
        }

        // Append received data to buffer
        buffer.insert(buffer.end(), img_bytes.begin(), img_bytes.end());

        // Check if the end of stream is reached
        if (buffer.size() > 8 && 
            buffer[buffer.size() - 8] == 'E' && 
            buffer[buffer.size() - 7] == 'N' &&
            buffer[buffer.size() - 6] == 'D' &&
            buffer[buffer.size() - 5] == '_' &&
            buffer[buffer.size() - 4] == 'S' &&
            buffer[buffer.size() - 3] == 'T' &&
            buffer[buffer.size() - 2] == 'R' &&
            buffer[buffer.size() - 1] == 'E') {
            buffer.resize(buffer.size() - 8); // Remove "END_STREAM"
            break;
        }
    }

    // Create a cv::Mat from the buffer
    cv::Mat video_frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (video_frame.empty()) {
        throw runtime_error("Failed to decode image");
    }

    return video_frame;
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

    // Test Display
    // test_display();

    //OpenCV Build Info
    // opencv_build_info();

    return 0;
}

