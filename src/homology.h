#ifndef HOMOLOGY_H
#define HOMOLOGY_H

#include <Eigen/Dense>
#include "rapidcsv.h"
#include <iostream>
#include <array>
#include <cmath>
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
template<typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr) {
    os << "[";
    for (std::size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i != N - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

//Finds point in 3d space based on two camera matricies and two point in the cameras frames
std::array<double, 3> find_pos_3d(const Eigen::Matrix<double, 3, 4> &C1, const Eigen::Matrix<double, 3, 4> &C2, const std::array<double, 2> p1, const std::array<double, 2> &p2){
  Eigen::Matrix<double, 6, 4> A;
  A.block(0,0,3,4) = C1;
  A.block(3,0,3,4) = C2;

  Eigen::Vector<double, 6> r {p1[0], p1[1], 1.0, p2[0], p2[1], 1.0};

  Eigen::Vector<double, 4> proj_pos = (A.transpose() * A).ldlt().solve(A.transpose() * r);

  std::array<double, 3> pos = {proj_pos(0)/proj_pos(3), proj_pos(1)/proj_pos(3), proj_pos(2)/proj_pos(3)};

  return pos;
}

bool check_pos_3d(const Eigen::Matrix<double, 3, 4> &C1, const Eigen::Matrix<double, 3, 4> &C2, const std::array<double, 3> &pos_3d, const std::array<double, 2> &p1, const std::array<double, 2> &p2){
  Eigen::Vector<double, 4> proj_pos {pos_3d[0], pos_3d[1], pos_3d[2], 1.0};

  Eigen::Vector<double, 3> proj_p1 = C1 * proj_pos;
  Eigen::Vector<double, 3> proj_p2 = C2 * proj_pos;

  std::array<double, 2> new_p1 = {proj_p1[0]/proj_p1[2], proj_p1[1]/proj_p1[2]};
  std::array<double, 2> new_p2 = {proj_p2[0]/proj_p2[2], proj_p2[1]/proj_p2[2]};

  return (new_p1 == p1) && (new_p2 == p2);
}

#endif