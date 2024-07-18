#include <Eigen/Dense>
#include <iostream>
using namespace std;

int main() {

  Eigen::Matrix<double, 2, 2> A{{1.0, 0.0}, {0.0, 1.0}};

  cout << A << endl;

  return 0;
}
