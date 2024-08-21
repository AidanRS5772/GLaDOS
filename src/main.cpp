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
