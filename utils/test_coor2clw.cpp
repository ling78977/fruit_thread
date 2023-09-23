#include "../src/module/coor2caw/coor2claw.h"

int main() {
  coor2claw::Coor2Claw coor2claw("/home/ling/fruit_cpp/config/coor2claw.xml");

  cv::Vec3f circle = {100, 200, 30};
  std::vector<double> coord =
      coor2claw.ArmCoord2Claw(cv::Point2d(int(circle[0]), int(circle[1])));
  for (size_t i = 0; i < 6; i++) {
    std::cout << coord[i] << " ";
  }

  return 0;
}
