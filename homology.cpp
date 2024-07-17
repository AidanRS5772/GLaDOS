#include "utils/eigen-3.4.0/Eigen/Dense"
#include "utils/rapidcsv/src/rapidcsv.h"
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>
using namespace std;

Eigen::Matrix3d make_rotation_matrix(const vector<double> &orientation) {
  Eigen::Quaterniond quaternion =
      Eigen::AngleAxisd(orientation[2] * (2 * M_PI), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(orientation[1] * (M_PI), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(orientation[0] * (M_PI / 2), Eigen::Vector3d::UnitX());

  return quaternion.toRotationMatrix();
}

tuple<Eigen::Matrix<double, 3, 4>, Eigen::Matrix<double, 3, 4>>
make_camera_matrices() {
  rapidcsv::Document doc("calibrations.csv");
  double f = doc.GetColumn<double>("focal")[0];
  vector<double> orientation1 = doc.GetColumn<double>("orientation1");
  vector<double> orientation2 = doc.GetColumn<double>("orientation2");
  vector<double> translation1 = doc.GetColumn<double>("translation1");
  vector<double> translation2 = doc.GetColumn<double>("translation2");

  Eigen::Matrix3d R1 = make_rotation_matrix(orientation1);
  Eigen::Matrix3d R2 = make_rotation_matrix(orientation2);

  Eigen::Matrix<double, 4, 4> H1 = Eigen::Matrix<double, 4, 4>::Zero();
  Eigen::Matrix<double, 4, 4> H2 = Eigen::Matrix<double, 4, 4>::Zero();

  H1.block<3, 3>(0, 0) = R1;
  H2.block<3, 3>(0, 0) = R2;

  for (int i = 0; i < 3; i++) {
    H1(i, 3) = translation1[i];
    H2(i, 3) = translation2[i];
  }

  H1(3, 3) = 1.0;
  H2(3, 3) = 1.0;

  Eigen::Matrix<double, 3, 4> F;
  F << f, 0.0, 0.0, 0.0, 0.0, f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  Eigen::Matrix<double, 3, 4> C1 = F * H1;
  Eigen::Matrix<double, 3, 4> C2 = F * H2;

  return make_tuple(C1, C2);
}

int main() {

  auto [C1, C2] = make_camera_matrices();

  cout << "C1:\n" << C1 << endl;
  cout << "C2:\n" << C2 << endl;

  return 0;
}
