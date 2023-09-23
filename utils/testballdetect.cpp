#include "../src/module/ball/ball.h"

int main() {
  ball::BallDetector balldetector("../config/detectball.xml", 9, 9999, false);
  //   cv::VideoCapture cap;
  //   for (int i = 3; i > -1; i--) {
  //     cap = cv::VideoCapture(i);
  //     if (cap.isOpened()) {
  //       break;
  //     } else {
  //       cap.release();
  //     }
  //   }
  cv::VideoCapture cap("/home/ling/fruit_cpp/samples/videos/detect_ball.mp4");
  if (!cap.isOpened()) {
    std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    return -1;
  } else {
    std::cout << "\033[1m open camera suceess .\033[0m" << std::endl;
  }
  cv::Mat img;
  ball::circles_result result = balldetector.detect_circles(cap, 1);
  for (int i = 0; i < result.circles.size(); i++) {
    std::cout << "第" << i + 1 << "个点:" << result.circles.at(i) << std::endl;
  }

  return 0;
}