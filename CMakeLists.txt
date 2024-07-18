cmake_minimum_required(VERSION 3.15)
project(GLaDOS)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
