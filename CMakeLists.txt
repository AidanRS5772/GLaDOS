cmake_minimum_required(VERSION 3.15)
project(GLaDOS)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wextra -Wpedantic -Wall -Wshadow)

find_package(Eigen3 REQUIRED)
find_package(rapidcsv REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

# First executable from server.cpp
add_executable(server src/server.cpp)
target_link_libraries(server Eigen3::Eigen)
target_link_libraries(server rapidcsv::rapidcsv)
target_link_libraries(server boost::boost)
target_link_libraries(server ${OpenCV_LIBS})

# Second executable from gen_facial_recog_data.cpp
add_executable(gen_facial_recog_data src/gen_facial_recog_data.cpp)
target_link_libraries(gen_facial_recog_data ${OpenCV_LIBS})
