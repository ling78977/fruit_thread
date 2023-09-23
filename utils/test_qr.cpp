#include "../src/module/qr/qr.h"

int main() {
  qr::QRDtector qr("/home/ling/fruit_cpp/config/qr.xml", 999, false);
  cv::VideoCapture cap;
  for (int i = 3; i > -1; i--) {
    cap = cv::VideoCapture(i);
    if (cap.isOpened()) {
      break;
    } else {
      cap.release();
    }
  }
  // cv::VideoCapture cap("/home/ling/fruit_cpp/samples/videos/detect_qr.mp4");
  if (!cap.isOpened()) {
    std::cerr << "\033[31m Couldn't open camera.\033[0m" << std::endl;
    return -1;
  } else {
    std::cout << "\033[1m open camera suceess .\033[0m" << std::endl;
  }
  qr::decode_rusult a = qr.QR_detect(cap);
  std::cout << a.data[0] << a.data[1] << a.data[2] << a.data[3] << std::endl;

  return 0;
}