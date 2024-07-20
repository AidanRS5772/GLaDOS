#include <iostream>
#include "homology.h"

using namespace std;

int main() {
  array<double, 2> p1 = {0.5, -0.3};
  array<double, 2> p2 = {0.5, 0.5};

  cout << "\nC1:\n" << CAMERA_1 << endl;
  cout << "\nC2:\n" << CAMERA_2 << endl;

  array<double, 3> pos_3d = find_pos_3d(CAMERA_1, CAMERA_2, p1, p2);

  printArray(pos_3d);

  return 0;
}
