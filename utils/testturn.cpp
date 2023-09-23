#include "../src/module/turn/turn.h"

int main() {
  turn::TurnDetector turndetecot("/home/ling/fruit_cpp/config/detectturn.xml",
                                 9, 9, false);
  //   cv::VideoCapture cap;
  //   for (int i = 3; i > -1; i--) {
  //     cap = cv::VideoCapture(i);
  //     if (cap.isOpened()) {
  //       break;
  //     } else {
  //       cap.release();
  //     }
  //   }
  cv::VideoCapture cap("/home/ling/fruit_cpp/samples/videos/detect_turn.mp4");
  if (!cap.isOpened()) {
    std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    return -1;
  } else {
    std::cout << "\033[1m open camera suceess .\033[0m" << std::endl;
  }
  int a = turndetecot.detect_turn(cap);
  std::cout << a << std::endl;
  return 0;
}