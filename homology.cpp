#include "utils/eigen-3.4.0/Eigen/Dense"
#include "utils/eigen-3.4.0/Eigen/src/Core/Matrix.h"
#include <cmath>
#include <iostream>
using namespace std;

Eigen::Matrix3d make_rotation_matrix(double (&orientation)[3]) {
  // Construct rotation quaternion
  Eigen::Quaterniond quaternion =
      Eigen::AngleAxisd(orientation[2] * (2 * M_PI), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(orientation[1] * (M_PI), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(orientation[0] * (M_PI / 2), Eigen::Vector3d::UnitX());

  // Convert the quaternion to a rotation matrix
  return quaternion.toRotationMatrix();
}

int main() {

  // Define orientation
  double orientation[3] = {0.5, 0.0, 0.0};

  Eigen::Matrix3d R = make_rotation_matrix(orientation);

  cout << "Rotation Matrix:\n" << R << endl;

  return 0;
}
