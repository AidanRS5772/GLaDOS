cmake_minimum_required(VERSION 3.15)
project(GLaDOS)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wextra -Wpedantic -Wall)

find_package(Eigen3 REQUIRED)
find_package(rapidcsv REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} rapidcsv::rapidcsv)
target_link_libraries(${PROJECT_NAME} boost::boost)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
