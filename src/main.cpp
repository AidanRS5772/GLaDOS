#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <map>

using boost::asio::ip::udp;
using namespace std;

void send_hello_message(udp::socket& socket, udp::endpoint& server_endpoint) {
    std::string hello_message = "HELLO";
    socket.send_to(boost::asio::buffer(hello_message), server_endpoint);
}

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <map>

using boost::asio::ip::udp;
using namespace std;

cv::Mat process_data_to_frame(udp::socket& socket, udp::endpoint& sender_endpoint) {
    std::map<int, std::vector<uint8_t>> chunks;
    int total_size = 0;
    while (true) {
        std::vector<uint8_t> buffer(65507);
        udp::endpoint endpoint;
        size_t len = socket.receive_from(boost::asio::buffer(buffer), sender_endpoint);

        if (len > 4) {  // Adjusted to read "I" format (4 bytes)
            int chunk_index = ntohl(*(uint32_t*)&buffer[0]);  // Adjusted for "I" format
            chunks[chunk_index] = std::vector<uint8_t>(buffer.begin() + 4, buffer.begin() + len);
            total_size += len - 4;

            // Assuming that the chunks are sent in order and the last chunk is smaller than MAX_UDP_SIZE
            if (len < 65507) {
                break;
            }
        }
    }

    std::vector<uint8_t> jpg_data;
    jpg_data.reserve(total_size);
    for (const auto& chunk : chunks) {
        jpg_data.insert(jpg_data.end(), chunk.second.begin(), chunk.second.end());
    }

    // Decode the received JPG image to an OpenCV Mat
    cv::Mat frame = cv::imdecode(jpg_data, cv::IMREAD_COLOR);
    if (frame.empty()) {
        throw runtime_error("Failed to decode the frame.");
    }

    return frame;
}


int main() {
    try {
        boost::asio::io_context io_context;
        udp::socket socket(io_context, udp::endpoint(udp::v4(), 0));  // 0 lets OS choose a port

        udp::endpoint server_endpoint(boost::asio::ip::address::from_string("10.0.0.235"), 12345);

        // Send a hello message to the server to register this client
        send_hello_message(socket, server_endpoint);

        udp::endpoint sender_endpoint;
        std::cout << "Waiting for server to send data..." << std::endl;

        while (true) {
            try {
                cv::Mat frame = process_data_to_frame(socket, sender_endpoint);
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
