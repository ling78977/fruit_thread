#include "src/module/detect/detect.h"

namespace fruit {
namespace detect {
DetectPool::DetectPool(const std::string detect_config, int qr_timeout,
                       bool qr_timeout_en, int trun_timeout,
                       bool turn_timeout_en, int turn_sample_times) {
  
  cv::FileStorage detect(detect_config, cv::FileStorage::READ);
  if (!detect.isOpened()) {
    std::cout << "\033[31m 打开detect参数文件失败！ \033[0m" << std::endl;
  }
  // 读取 qr 参数文件
  detect["QR_EDIT"] >> this->detect_qr_config_.qr_edit;

  //读取turn 参数
  this->detect_turn_config_.turn_detect_template_ahead =
      cv::imread("/home/ling/fruit_cpp/samples/template_t/ahead.jpg");
  if (detect_turn_config_.turn_detect_template_ahead.empty()) {
    std::cout << "\033[31m 读取模板：直行 失败 \033[0m" << std::endl;
  }
  cv::cvtColor(detect_turn_config_.turn_detect_template_ahead,
               detect_turn_config_.turn_detect_template_ahead,
               cv::COLOR_BGR2GRAY);

  this->detect_turn_config_.turn_detect_template_left =
      cv::imread("/home/ling/fruit_cpp/samples/template_t/left.jpg");
  if (detect_turn_config_.turn_detect_template_left.empty()) {
    std::cout << "\033[31m 读取模板：左转 失败 \033[0m" << std::endl;
  }
  cv::cvtColor(detect_turn_config_.turn_detect_template_left,
               detect_turn_config_.turn_detect_template_left,
               cv::COLOR_BGR2GRAY);

  this->detect_turn_config_.turn_detect_template_right =
      cv::imread("/home/ling/fruit_cpp/samples/template_t/right.jpg");
  if (detect_turn_config_.turn_detect_template_right.empty()) {
    std::cout << "\033[31m 读取模板：右行 失败 \033[0m" << std::endl;
  }
  cv::cvtColor(detect_turn_config_.turn_detect_template_right,
               detect_turn_config_.turn_detect_template_right,
               cv::COLOR_BGR2GRAY);

  // 初始化基本参数
  detect["H_BLUE_MIN"] >> detect_turn_config_.h_blue_min;
  detect["H_BLUE_MAX"] >> detect_turn_config_.h_blue_max;
  detect["S_BLUE_MIN"] >> detect_turn_config_.s_blue_min;
  detect["S_BLUE_MAX"] >> detect_turn_config_.s_blue_max;
  detect["V_BLUE_MIN"] >> detect_turn_config_.v_blue_min;
  detect["V_BLUE_MAX"] >> detect_turn_config_.v_blue_max;
  detect["MORPHOLOGRAY_KERNEL_SIZE_TURN"] >>
      detect_turn_config_.morphology_kernel_size_turn;

  detect["ERODE_DILATE_KERNEL_SIZE_TURN"] >>
      detect_turn_config_.erode_dilate_kernel_size_turn;
  detect["ERODE_ITERATIONS_TURN"] >> detect_turn_config_.erode_iterations_turn;
  detect["DILATE_ITERATIONS_TURN"] >>
      detect_turn_config_.dilate_iterations_turn;
  detect["AREA_THRESHOLD"] >> detect_turn_config_.area_threshold;
  detect["MAX_VAL_THRESHOLD"] >> detect_turn_config_.max_val_threshold;
  detect["TURN_EDIT"] >> detect_turn_config_.turn_edit_;
  detect.release();
  detect_qr_config_.qr_detect_timeout_sec = qr_timeout;
  detect_qr_config_.qr_detect_timeout_en = qr_timeout_en;
  detect_turn_config_.turn_detect_timeout = trun_timeout;
  detect_turn_config_.turn_detect_timeout_en = turn_timeout_en;
  detect_turn_config_.turn_detect_sample_times = turn_sample_times;
}
DetectPool::~DetectPool() {}
void DetectPool::collecteImage(cv::VideoCapture& cap) {
  frame_mutex_.lock();
  cap >> share_ori_frame_;
  frame_mutex_.unlock();
}

ResulteQR DetectPool::detectQR() {
  printf("\n开始识别二维码\n");
  ResulteQR result_qr_;
  detect_qr_config_.qr_detect_start_time = cv::getTickCount();
  while (true) {
    // 超时退出
    double task_time_ =
        (cv::getTickCount() - detect_qr_config_.qr_detect_start_time) /
        cv::getTickFrequency();
    if (!detect_qr_config_.qr_detect_timeout_en &&
        task_time_ >= detect_qr_config_.qr_detect_timeout_sec) {
      printf("\n超时未识别到二维码!!\n");
      break;
    }
    // 获取图像帧
    cv::Mat qr_detect_img;
    frame_mutex_.lock();
    qr_detect_img = share_ori_frame_;
    frame_mutex_.unlock();
    if (!qr_detect_img.empty()) {
      if (this->detect_qr_config_.qr_edit == 1) {
        cv::imshow("qrcode", qr_detect_img);
      }
      // 定义一个矩阵变量来存储二维码的位置
      cv::Mat bbox;
      // 调用detectAndDecode()方法来检测和解码二维码
      std::string data = qr_decoder_.detectAndDecode(qr_detect_img, bbox);
      if (data.length() != 0) {
        std::cout << "\nFruit sequence: " << data << "\n二维码识别结束\n"
                  << std::endl;
        for (char c : data) {
          int num = c - 48;
          result_qr_.data.push_back(num);
        }
        result_qr_.flag = true;
        break;
      }
    }
  }
  return result_qr_;
}

int DetectPool::detectTurn() {
  double t0 = cv::getTickCount();
  std::cout << ".......................开始识别转向........................"
            << std::endl;
  std::vector<int> datalist;
  int num = 0;
  int majorityElement = 2;
  cv::Mat img, output;
  int turn_current = 0;
  while (true) {
    double task_time = (cv::getTickCount() - t0) / cv::getTickFrequency();
    if (!detect_turn_config_.turn_detect_timeout_en &&
        task_time >= detect_turn_config_.turn_detect_timeout) {
      std::cout << "超时，未识别转向牌子！" << std::endl;
    }
    frame_mutex_.lock();
    img = share_ori_frame_;
    frame_mutex_.unlock();
    if (img.empty()) {
      continue;
    }
    FindTurnSign sign = detectTurnFindSign(img);

    if (sign.flag) {
      if (detect_turn_config_.turn_edit_ == 1) {
        cv::imshow("turn_roi", sign.img);
        cv::waitKey(1);
      }
      turn_current = detectTurnMatchTemplate(sign.img);
    } else {
      continue;
    }
    if (turn_current) {
      datalist.push_back(turn_current);
      turn_current = 0;
      num += 1;
      std::string turn;
      if (num == detect_turn_config_.turn_detect_sample_times) {
        majorityElement = detectTurnArgmax(datalist);
        switch (majorityElement) {
          case 1:
            turn = "left";
            break;
          case 2:
            turn = "ahead";
            break;
          case 3:
            turn = "right";
            break;
          default:
            turn = "ahead";
            break;
        }
        std::cout << "转向标识识别结束，识别为:" << turn << std::endl;
        break;
      }
    }
  }
  cv::destroyAllWindows();
  return majorityElement;
}

int DetectPool::detectTurnArgmax(std::vector<int> nums) {
  std::sort(nums.begin(), nums.end());  // 将vector容器中的元素进行升序排序

  int maxNum = nums[0],
      maxCount = 1;  // 记录当前出现次数最多的数字及其出现次数
  int curNum = nums[0], curCount = 1;  // 记录当前数字及其出现次数

  for (int i = 1; i < nums.size(); i++) {
    if (nums[i] == curNum) {
      curCount++;  // 当前数字出现次数加1
    } else {
      curCount = 1;      // 重置当前数字出现次数为1
      curNum = nums[i];  // 更新当前数字为当前遍历的数字
    }

    if (curCount > maxCount) {
      maxCount = curCount;  // 更新出现次数最多的数字及其出现次数
      maxNum = curNum;
    }
  }
  // cout << "众数为：" << maxNum << endl;
  return maxNum;
}

cv::Mat DetectPool::detectTurnPre(cv::Mat frame) {
  if (detect_turn_config_.turn_edit_ == 1) {
    cv::imshow("turn_img", frame);
    cv::waitKey(1);
  }
  cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
  cv::Mat img_range;
  cv::inRange(
      frame,
      cv::Scalar(detect_turn_config_.h_blue_min, detect_turn_config_.s_blue_min,
                 detect_turn_config_.v_blue_min),
      cv::Scalar(detect_turn_config_.h_blue_max, detect_turn_config_.s_blue_max,
                 detect_turn_config_.v_blue_max),
      img_range);
  if (detect_turn_config_.turn_edit_ == 1) {
    cv::imshow("turn_inrange", img_range);
    cv::waitKey(1);
  }
  cv::GaussianBlur(img_range, img_range, cv::Size(11, 11), 2, 2);
  cv::Mat img_close;
  cv::morphologyEx(
      img_range, img_close, cv::MORPH_CLOSE,
      cv::getStructuringElement(
          cv::MORPH_RECT,
          cv::Size(detect_turn_config_.morphology_kernel_size_turn,
                   detect_turn_config_.morphology_kernel_size_turn)));
  if (detect_turn_config_.turn_edit_ == 1) {
    cv::imshow("turn_closed", img_close);
    cv::waitKey(1);
  }
  cv::Mat img_erode;
  cv::erode(img_close, img_erode,
            cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(detect_turn_config_.erode_dilate_kernel_size_turn,
                         detect_turn_config_.erode_dilate_kernel_size_turn)),
            cv::Point(-1, -1), detect_turn_config_.erode_iterations_turn);
  if (detect_turn_config_.turn_edit_ == 1) {
    cv::imshow("turn_erode", img_erode);
    cv::waitKey(1);
  }
  cv::Mat img_dilate;
  cv::dilate(img_erode, img_dilate,
             cv::getStructuringElement(
                 cv::MORPH_RECT,
                 cv::Size(detect_turn_config_.erode_dilate_kernel_size_turn,
                          detect_turn_config_.erode_dilate_kernel_size_turn)),
             cv::Point(-1, -1), detect_turn_config_.dilate_iterations_turn);
  if (detect_turn_config_.turn_edit_ == 1) {
    cv::imshow("turn_dilate", img_dilate);
    cv::waitKey(1);
  }
  return img_dilate;
}

FindTurnSign DetectPool::detectTurnFindSign(cv::Mat frame) {
  FindTurnSign find_sign;
  cv::Mat pre_result = detectTurnPre(frame);
  cv::Mat img_roi;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(pre_result, contours, hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_NONE);
  if (!contours.empty()) {
    int max_index = 0;
    double max_area = 0;
    for (int i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      cv::RotatedRect rect = cv::minAreaRect(contours[i]);
      double w = rect.size.width;
      double h = rect.size.height;
      double rate = cv::min(w, h) / cv::max(w, h);
      if (rate > 0.75 && area > max_area) {
        max_area = area;
        max_index = i;
      }
    }
    if (max_area > detect_turn_config_.area_threshold) {
      cv::Rect boundingRect = cv::boundingRect(contours[max_index]);
      cv::rectangle(frame, boundingRect, cv::Scalar(0, 0, 255), 2);
      if (detect_turn_config_.turn_edit_ == 1) {
        cv::imshow("img_src_sign", frame);
        cv::waitKey(1);
      }
      if (boundingRect.tl().x > 0 && boundingRect.tl().y > 0 &&
          boundingRect.br().x > 0 && boundingRect.br().y > 0 &&
          boundingRect.br().y - boundingRect.tl().y > 0 &&
          boundingRect.br().x - boundingRect.tl().x > 0) {
        img_roi = frame(boundingRect).clone();
        if (detect_turn_config_.turn_edit_ == 1) {
          cv::imshow("img_src_sign", img_roi);
          cv::waitKey(1);
        }
        cv::resize(img_roi, img_roi, cv::Size(30, 30));
        cv::cvtColor(img_roi, img_roi, cv::COLOR_BGR2GRAY);
        find_sign.flag = true;
        find_sign.img = img_roi;
      }
    }
  }
  return find_sign;
}

int DetectPool::detectTurnMatchTemplate(cv::Mat& roi) {
  cv::Mat res_left, res_ahead, res_right;
  double max_val1, min_val1, max_val2, min_val2, max_val3, min_val3, max_val;
  cv::Point max_loc1, max_loc2, max_loc3, min_loc1, min_loc2, min_loc3;

  cv::matchTemplate(roi, detect_turn_config_.turn_detect_template_left,
                    res_left, cv::TM_CCOEFF_NORMED);
  // std::cout << "1" << std::endl;
  cv::minMaxLoc(res_left, &min_val1, &max_val1, &min_loc1, &max_loc1);
  cv::matchTemplate(roi, detect_turn_config_.turn_detect_template_ahead,
                    res_ahead, cv::TM_CCOEFF_NORMED);
  cv::minMaxLoc(res_ahead, &min_val2, &max_val2, &min_loc2, &max_loc2);
  cv::matchTemplate(roi, detect_turn_config_.turn_detect_template_right,
                    res_right, cv::TM_CCOEFF_NORMED);
  cv::minMaxLoc(res_right, &min_val3, &max_val3, &min_loc3, &max_loc3);
  max_val = std::max(max_val1, std::max(max_val2, max_val3));
  if (max_val < detect_turn_config_.max_val_threshold) {
    return 0;
  }
  if (max_val == max_val1) {
    std::cout << "left!" << std::endl;
    return 1;
  } else if (max_val == max_val2) {
    std::cout << "ahead!" << std::endl;
    return 2;
  } else if (max_val == max_val3) {
    std::cout << "right!" << std::endl;
    return 3;
  }
}

}  // namespace detect
}  // namespace fruit