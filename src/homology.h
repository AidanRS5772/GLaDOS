#ifndef HOMOLOGY_H
#define HOMOLOGY_H

#include <Eigen/Dense>
#include "rapidcsv.h"
#include <iostream>
#include <array>
#include <vector>

const rapidcsv::Document doc("../../../src/calibrations.csv");

const std::vector<double> ORIENTATION_1 = doc.GetColumn<double>("orientation1");
const std::vector<double> TRANSLATION_1 = doc.GetColumn<double>("translation1");
const std::vector<double> ORIENTATION_2 = doc.GetColumn<double>("orientation2");
const std::vector<double> TRANSLATION_2 = doc.GetColumn<double>("translation2");
const double focal = doc.GetColumn<double>("focal")[0];

// Creating Camera Matricies

static Eigen::Matrix<double, 3, 4> make_camera_matrix(const std::vector<double> &orientation, const std::vector<double> &translation, double focal){
  Eigen::Quaterniond quaternion = 
        Eigen::AngleAxisd(orientation[2]*(2*M_PI), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(orientation[1]*(M_PI), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(orientation[0]*(M_PI / 2), Eigen::Vector3d::UnitX());

  Eigen::Matrix3d R = quaternion.toRotationMatrix();

  Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
  H.block(0,0,3,3) = R;

  for(int i=0; i < 3; i++){
    H(i,3) = translation[i];
  }

  H(3,3) = 1.0;

  Eigen::Matrix<double, 3, 4> F
  {{focal, 0.0, 0.0, 0.0},
   {0.0, focal, 0.0, 0.0},
   {0.0, 0.0, 1.0, 0.0}};

  Eigen::Matrix<double, 3, 4> C = F * H;

  return C;
}

const Eigen::Matrix<double, 3, 4> CAMERA_1 = make_camera_matrix(ORIENTATION_1, TRANSLATION_1, focal);
const Eigen::Matrix<double, 3, 4> CAMERA_2 = make_camera_matrix(ORIENTATION_2, TRANSLATION_2, focal);

// Functions To be Used

//Prints out Array
template <typename T, std::size_t N>
void printArray(const std::array<T, N>& arr) {
    std::cout << "[ ";
    for (std::size_t i = 0; i < N; ++i) {
        std::cout << arr[i];
        if (i < N - 1) {
            std::cout << ", ";
        }
    }
    std::cout << " ]" << std::endl;
}

//Finds point in 3d space based on two camera matricies and two point in the cameras frames
std::array<double, 3> find_pos_3d(const Eigen::Matrix<double, 3, 4> &C1, const Eigen::Matrix<double, 3, 4> &C2, const std::array<double, 2> p1, const std::array<double, 2> &p2){
  Eigen::VectorXd rows[4] = {p1[0] * C1.row(2) - C1.row(0), p1[1] * C1.row(2) - C1.row(1), p2[0] * C1.row(2) - C2.row(0), p2[1] * C1.row(2) - C2.row(1)};

  Eigen::Matrix<double, 4, 4> A;
  for(int i=0; i < 4; i++){
    A.row(i) = rows[i].transpose();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::VectorXd proj_pos = svd.matrixV().col(3);

  std::array<double, 3> pos = {proj_pos(0)/proj_pos(3), proj_pos(1)/proj_pos(3), proj_pos(2)/proj_pos(3)};
  return pos;
}

#endif