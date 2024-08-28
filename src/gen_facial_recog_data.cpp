#include <dlib/dnn.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_io.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>

using namespace dlib;
using namespace std;
using json = nlohmann::json;

template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual = add_prev1<block<N,BN,1,tag1<SUBNET>>>;

template <template <int,template<typename>class,int,typename> class block, int N, template<typename>class BN, typename SUBNET>
using residual_down = add_prev2<avg_pool<2,2,2,2,skip1<tag2<block<N,BN,2,tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET> 
using block  = BN<con<N,3,3,1,1,relu<BN<con<N,3,3,stride,stride,SUBNET>>>>>;

template <int N, typename SUBNET> using ares      = relu<residual<block,N,affine,SUBNET>>;
template <int N, typename SUBNET> using ares_down = relu<residual_down<block,N,affine,SUBNET>>;

template <typename SUBNET> using alevel0 = ares_down<256,SUBNET>;
template <typename SUBNET> using alevel1 = ares<256,ares<256,ares_down<256,SUBNET>>>;
template <typename SUBNET> using alevel2 = ares<128,ares<128,ares_down<128,SUBNET>>>;
template <typename SUBNET> using alevel3 = ares<64,ares<64,ares<64,ares_down<64,SUBNET>>>>;
template <typename SUBNET> using alevel4 = ares<32,ares<32,ares<32,SUBNET>>>;

using anet_type = loss_metric<fc_no_bias<128,avg_pool_everything<
                            alevel0<
                            alevel1<
                            alevel2<
                            alevel3<
                            alevel4<
                            max_pool<3,3,2,2,relu<affine<con<32,7,7,2,2,
                            input_rgb_image_sized<150>
                            >>>>>>>>>>>>;

template <typename T>
void printVector(const std::vector<T>& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i < vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

const int NUM_IMGS = 4;

int main() {
    try {
        // Loading Pre-Trained Models
        frontal_face_detector detector = get_frontal_face_detector();
        shape_predictor sp;
        deserialize("../../../src/face_recog_DNN/shape_predictor_68_face_landmarks.dat") >> sp;
        anet_type net;
        deserialize("../../../src/face_recog_DNN/dlib_face_recognition_resnet_model_v1.dat") >> net;

        // Loading Images

        string gen_path = "../../../src/src_face_db/aidan/";
        std::vector<matrix<rgb_pixel>> imgs;
        for(int i=1; i <= NUM_IMGS; i++){
            string path = gen_path + to_string(i) + ".jpg";
            matrix<rgb_pixel> img;
            load_image(img, path);
            imgs.push_back(std::move(img));
        }

        // Detect Faces
        std::vector<rectangle> face_rects;
        for(unsigned long i=0; i < imgs.size(); i++){
            std::vector<rectangle> face_dets = detector(imgs[i]);
            if (face_dets.size() == 0){
                throw runtime_error("No faces found for image: " + std::to_string(i));
            }else if (face_dets.size() > 1){
                throw runtime_error("Too many faces found for image: " + std::to_string(i));
            }

            rectangle face_rect = face_dets[0];
            face_rects.push_back(std::move(face_rect));
        }

        //Get Face Chip
        std::vector<matrix<rgb_pixel>> face_chips;
        for(unsigned long i=0; i < imgs.size(); i++){
            auto shape = sp(imgs[i], face_rects[i]);
            matrix<rgb_pixel> face_chip;
            extract_image_chip(imgs[i], get_face_chip_details(shape,150,0.25), face_chip);
            face_chips.push_back(std::move(face_chip));
        }

        // Apply Facial Recog DNN
        std::vector<matrix<float,0,1>> face_descriptors = net(face_chips);

        json j;
        for (const auto& descriptor : face_descriptors) {
            std::vector<float> vec(descriptor.begin(), descriptor.end());
            j["aidan"].push_back(vec);
        }

        // Write to the JSON
        std::ofstream file("../../../src/face_recog_data.json");
        file << j.dump(imgs.size());  
        file.close();
    }
    catch (std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    return 0;
}