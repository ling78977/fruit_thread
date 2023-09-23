#include "src/module/ball/ball.h"

namespace fruit {
namespace ball {
BallDetector::BallDetector(const std::string _ball_detect_config,
                           int _sample_times, int _time_out, bool _time_out_en)
    : sample_times_(_sample_times),
      time_out_(_time_out),
      time_out_en_(_time_out_en) {
  cv::FileStorage ball(_ball_detect_config, cv::FileStorage::READ);
  if (!ball.isOpened()) {
    std::cout << "\033[31m 打开 detectball 文件失败！\033[0m" << std::endl;
  } else {
    std::cout << "\033[32m 打开 detectball 文件成功！\033[0m" << std::endl;
  }
  ball["H_YELLOW_LOWER"] >> ball_detect_config_.h_yellow_lower;
  ball["H_YELLOW_UPPER"] >> ball_detect_config_.h_yellow_upper;
  ball["L_YELLOW_LOWER"] >> ball_detect_config_.l_yellow_lower;
  ball["L_YELLOW_UPPER"] >> ball_detect_config_.l_yellow_upper;
  ball["S_YELLOW_LOWER"] >> ball_detect_config_.s_yellow_lower;
  ball["S_YELLOW_UPPER"] >> ball_detect_config_.s_yellow_upper;
  ball["R_WHITE_LOWER"] >> ball_detect_config_.r_white_lower;
  ball["R_WHITE_UPPER"] >> ball_detect_config_.r_white_upper;
  ball["G_WHITE_LOWER"] >> ball_detect_config_.g_white_lower;
  ball["G_WHITE_UPPER"] >> ball_detect_config_.g_white_upper;
  ball["B_WHITE_LOWER"] >> ball_detect_config_.b_white_lower;
  ball["B_WHITE_UPPER"] >> ball_detect_config_.b_white_upper;
  ball["MORPHOLOGRAY_KERNEL_SIZE_BALL"] >>
      ball_detect_config_.morphology_kernel_size_ball;
  ball["ERODE_DILATE_KERNEL_SIZE_BALL"] >>
      ball_detect_config_.erode_dilate_kernel_size_ball;
  ball["ERODE_ITERATIONS_BALL"] >> ball_detect_config_.erode_iterations_ball;
  ball["DILATE_ITERATIONS_BALL"] >> ball_detect_config_.dilate_iterations_ball;
  ball["CANNY_THRESHOLD1"] >> ball_detect_config_.canny_threshold1;
  ball["CANNY_THRESHOLD2"] >> ball_detect_config_.canny_threshold2;
  ball["HOUGHCIRCLES_PARAM2"] >> ball_detect_config_.HoughCircles_param2;
  ball["RMIN"] >> ball_detect_config_.rmin;
  ball["RMAX"] >> ball_detect_config_.rmax;
  ball["RMIN_THRESHOLD"] >> ball_detect_config_.rmin_threshold;
  ball["X_SELECT_REF"] >> ball_detect_config_.x_select_ref;
  ball["N_PIC_PER_TYPE"] >> ball_detect_config_.n_pic_fer_type;
  ball["RATIO"] >> ball_detect_config_.ratio;
  ball["BALL_EDIT"] >> ball_detect_config_.ball_edit;
  ball.release();

  if (ball_detect_config_.ball_edit == 1) {
    cv::namedWindow(yellow_hls_window_, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("h_yellow_min", yellow_hls_window_,
                       &ball_detect_config_.h_yellow_lower, 180, NULL);
    cv::createTrackbar("h_yellow_max", yellow_hls_window_,
                       &ball_detect_config_.h_yellow_upper, 255, NULL);
    cv::createTrackbar("l_yellow_min", yellow_hls_window_,
                       &ball_detect_config_.l_yellow_lower, 255, NULL);
    cv::createTrackbar("l_yellow_max", yellow_hls_window_,
                       &ball_detect_config_.l_yellow_upper, 255, NULL);
    cv::createTrackbar("s_yellow_min", yellow_hls_window_,
                       &ball_detect_config_.s_yellow_lower, 255, NULL);
    cv::createTrackbar("s_yellow_max", yellow_hls_window_,
                       &ball_detect_config_.s_yellow_upper, 255, NULL);

    cv::namedWindow(white_rgb_window_, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("r_white_lower", white_rgb_window_,
                       &ball_detect_config_.r_white_lower, 255, NULL);
    cv::createTrackbar("r_white_upper", white_rgb_window_,
                       &ball_detect_config_.r_white_upper, 255, NULL);
    cv::createTrackbar("g_white_lower", white_rgb_window_,
                       &ball_detect_config_.g_white_lower, 255, NULL);
    cv::createTrackbar("g_white_upper", white_rgb_window_,
                       &ball_detect_config_.g_white_upper, 255, NULL);
    cv::createTrackbar("b_white_lower", white_rgb_window_,
                       &ball_detect_config_.b_white_lower, 255, NULL);
    cv::createTrackbar("b_white_upper", white_rgb_window_,
                       &ball_detect_config_.b_white_upper, 255, NULL);
  }
}

std::vector<bool> BallDetector::zScore(const std::vector<double>& _stats,
                                       double _ref) {
  double mean = cv::mean(_stats)[0];  // 计算平均值
  double std =
      cv::norm(_stats, cv::NORM_L2) / std::sqrt(_stats.size());  // 计算标准差
  // 标准差为0，返回全真
  if (std == 0.0) {
    return std::vector<bool>(_stats.size(), true);
  }
  std::vector<double> stats_z(_stats.size());  // 计算z标准值
  for (size_t i = 0; i < _stats.size(); i++) {
    stats_z[i] = (_stats[i] - mean) / std;
  }
  // 判断各值是否小于参考值
  std::vector<bool> result(_stats.size());
  for (size_t i = 0; i < _stats.size(); i++) {
    result[i] = std::abs(stats_z[i]) < _ref;
  }
  return result;
}

cv::Mat BallDetector::colorFilter(cv::Mat _img, int _color) {
  cv::Scalar thresh_lower, thresh_upper;
  if (_color == 1) {
    cv::cvtColor(_img, _img, cv::COLOR_BGR2HLS);
    thresh_lower = cv::Scalar(ball_detect_config_.h_yellow_lower,
                              ball_detect_config_.l_yellow_lower,
                              ball_detect_config_.s_yellow_lower);
    thresh_upper = cv::Scalar(ball_detect_config_.h_yellow_upper,
                              ball_detect_config_.l_yellow_upper,
                              ball_detect_config_.s_yellow_upper);
  } else {
    thresh_lower = cv::Scalar(ball_detect_config_.r_white_lower,
                              ball_detect_config_.g_white_lower,
                              ball_detect_config_.b_white_lower);
    thresh_upper = cv::Scalar(ball_detect_config_.r_white_upper,
                              ball_detect_config_.g_white_upper,
                              ball_detect_config_.b_white_upper);
  }

  cv::inRange(_img, thresh_lower, thresh_upper, _img);
  return _img;
}

std::vector<cv::Vec3f> BallDetector::findCircle(const cv::Mat& _frame,
                                                int _color,
                                                int _circle_number) {
  // 过滤其它颜色
  cv::Mat img_filtered = colorFilter(_frame, _color);

  //    cv::imshow("Filtered", img_filtered);
  // 高斯模糊
  cv::Mat img_blurred;
  cv::GaussianBlur(img_filtered, img_blurred, cv::Size(11, 11), 2, 2);
  //    cv::imshow("GaussianBlur", img_blurred);
  // 二值化
  cv::Mat img_thresholded;
  cv::threshold(img_blurred, img_thresholded, 127, 255, cv::THRESH_BINARY);
  //    cv::imshow("threshold", img_thresholded);
  // 形态学闭操作，使区域闭合无空隙
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(ball_detect_config_.morphology_kernel_size_ball,
               ball_detect_config_.morphology_kernel_size_ball));  // 卷积核大小
  cv::Mat img_closed;
  cv::morphologyEx(img_thresholded, img_closed, cv::MORPH_CLOSE, kernel);
  //    cv::imshow("Closed", img_closed);

  // 腐蚀和膨胀 (同形态学闭操作，但精细化调整)
  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(ball_detect_config_.erode_dilate_kernel_size_ball,
               ball_detect_config_.erode_dilate_kernel_size_ball));
  cv::Mat img_eroded;
  cv::erode(img_closed, img_eroded, element, cv::Point(-1, -1),
            ball_detect_config_.erode_iterations_ball);
  //    cv::imshow("erode", img_eroded);
  cv::Mat img_dilated;
  cv::dilate(img_eroded, img_dilated, element, cv::Point(-1, -1),
             ball_detect_config_.dilate_iterations_ball);
  //    cv::imshow("dilate", img_dilated);
  // canny边缘检测
  cv::Mat img_canny;
  cv::Canny(img_dilated, img_canny, ball_detect_config_.canny_threshold1,
            ball_detect_config_.canny_threshold2);
  //    cv::imshow("Canny", img_canny);
  if (ball_detect_config_.ball_edit == 1) {
    cv::imshow("Filtered", img_filtered);
    cv::imshow("GaussianBlur", img_blurred);
    cv::imshow("threshold", img_thresholded);
    cv::imshow("Closed", img_closed);
    cv::imshow("erode", img_eroded);
    cv::imshow("dilate", img_dilated);
    cv::imshow("Canny", img_canny);
  }
  // 自适应霍夫圆检测
  int num = 0;  // 初始化检测到的圆数量
  int r_min =
      ball_detect_config_.rmin;  // 导入设定的最小半径       // 初始化圆参数
  std::vector<cv::Vec3f> circles = {{0, 0, 0}};
  while (
      num != _circle_number &&
      r_min <
          ball_detect_config_
              .rmin_threshold) {  // 当未检测到目标数量圆，且minRadius参数未达到设置的上限时，循环检测
    r_min += 2;                   // 每次minRadius参数自加2
    cv::HoughCircles(img_canny, circles, cv::HOUGH_GRADIENT, 1,
                     100,  // change this value to detect circles with different
                           // distances to each other
                     100, ball_detect_config_.HoughCircles_param2, r_min,
                     ball_detect_config_.rmax);  // 可修改的参数
    // param2，越小，检测到越多近似的圆； 越大，检测到的圆越接近完美的圆形
    // minRadius，最小检测圆半径，maxRadius，最大检测圆半径
    if (circles.empty()) {  // 检测结果为空
      continue;
    }
    num = circles.size();  // 检测到的圆的数量
  }
  return circles;
}

std::vector<cv::Vec3f> BallDetector::findCircle(const cv::Mat& _frame,
                                                int _color) {
  // 过滤其它颜色
  cv::Mat img_filtered = colorFilter(_frame, _color);

  // cv::imshow("Filtered1", img_filtered);
  // 高斯模糊
  cv::Mat img_blurred;
  cv::GaussianBlur(img_filtered, img_blurred, cv::Size(11, 11), 2, 2);
  //    cv::imshow("GaussianBlur", img_blurred);
  // 二值化
  cv::Mat img_thresholded;
  cv::threshold(img_blurred, img_thresholded, 127, 255, cv::THRESH_BINARY);
  //    cv::imshow("threshold", img_thresholded);
  // 形态学闭操作，使区域闭合无空隙
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(ball_detect_config_.morphology_kernel_size_ball,
               ball_detect_config_.morphology_kernel_size_ball));  // 卷积核大小
  cv::Mat img_closed;
  cv::morphologyEx(img_thresholded, img_closed, cv::MORPH_CLOSE, kernel);
  //    cv::imshow("Closed", img_closed);

  // 腐蚀和膨胀 (同形态学闭操作，但精细化调整)
  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(ball_detect_config_.erode_dilate_kernel_size_ball,
               ball_detect_config_.erode_dilate_kernel_size_ball));
  cv::Mat img_eroded;
  cv::erode(img_closed, img_eroded, element, cv::Point(-1, -1),
            ball_detect_config_.erode_iterations_ball);
  //    cv::imshow("erode", img_eroded);
  cv::Mat img_dilated;
  cv::dilate(img_eroded, img_dilated, element, cv::Point(-1, -1),
             ball_detect_config_.dilate_iterations_ball);
  //    cv::imshow("dilate", img_dilated);
  // canny边缘检测
  cv::Mat img_canny;
  cv::Canny(img_dilated, img_canny, ball_detect_config_.canny_threshold1,
            ball_detect_config_.canny_threshold2);
  //    cv::imshow("Canny", img_canny);
  // std::cout << "ball_detect_config_.edit:" << ball_detect_config_.edit
  //           << std::endl;
  if (ball_detect_config_.ball_edit == 1) {
    cv::imshow("Filtered", img_filtered);
    cv::imshow("GaussianBlur", img_blurred);
    cv::imshow("threshold", img_thresholded);
    cv::imshow("Closed", img_closed);
    cv::imshow("erode", img_eroded);
    cv::imshow("dilate", img_dilated);
    cv::imshow("Canny", img_canny);
  }

  // 自适应霍夫圆检测
  int num = 0;  // 初始化检测到的圆数量
  int r_min =
      ball_detect_config_.rmin;  // 导入设定的最小半径       // 初始化圆参数
  std::vector<cv::Vec3f> circles = {{0, 0, 0}};
  // cv::HoughCircles(img_canny, circles, cv::HOUGH_GRADIENT, 1, 100, 100, 18,
  // 0,
  //                  0);
  while (num == 0 && r_min < ball_detect_config_.rmin_threshold) {
    r_min += 2;  // 每次minRadius参数自加2

    cv::HoughCircles(img_canny, circles, cv::HOUGH_GRADIENT, 1, 100, 100, 20,
                     ball_detect_config_.rmin,
                     ball_detect_config_.rmax);  // 可修改的参数

    // param2，越小，检测到越多近似的圆； 越大，检测到的圆越接近完美的圆形
    // minRadius，最小检测圆半径，maxRadius，最大检测圆半径
    if (circles.empty()) {  // 检测结果为空
      continue;
    }
    num = circles.size();  // 检测到的圆的数量
  }
  // std::cout << "dsvsvvds" << std::endl;
  return circles;
}

CirclesResult BallDetector::detectCircles(
    fruit::detect::DetectPool* _detectpool, int _color, int _circle_number) {
  std::cout << "\n------开始检测圆位置------\n" << std::endl;
  double t0 = cv::getTickCount();  // 初始时间

  int detect_fail = 0;  // 初始化检测失败标志位
  int sample_time = 0;  // 初始化采样次数为0
  // 初始化数据记录表为空
  std::vector<double> x_table_1;
  std::vector<double> y_table_1;
  std::vector<double> r_table_1;

  cv::Mat img;
  while (sample_time <
         sample_times_) {  // 当采样次数小于设定次数时，循环检测采样
    detect_fail = 0;  // 失败时再置位
    // 超时退出
    double task_time = (cv::getTickCount() - t0) / cv::getTickFrequency();
    if (!time_out_en_ && task_time >= time_out_) {
      std::cout << "\n超时未检测到圆\n" << std::endl;
      detect_fail = 1;  // 失败置位
      break;
    }
    _detectpool->frame_mutex_.lock();
    img = _detectpool->share_ori_frame_;
    _detectpool->frame_mutex_.unlock();
    if (img.empty() || cv::waitKey() == 27) {
      continue;
    }
    cv::Mat img_clone = img.clone();
    // cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
    // cv::rectangle(mask, cv::Point((int)img_clone.cols / 3 - 25, 0),
    //               cv::Point((int)img_clone.cols * 2 / 3 + 25, 150),
    //               cv::Scalar(255, 255, 255), -1);
    // cv::rectangle(mask, cv::Point(0, 300),
    //               cv::Point((int)img_clone.cols / 3 + 25,
    //               (int)img_clone.rows), cv::Scalar(255, 255, 255), -1);
    // cv::rectangle(mask, cv::Point((int)img_clone.cols * 2 / 3 - 25, 300),
    //               cv::Point((int)img_clone.cols, (int)img_clone.rows),
    //               cv::Scalar(255, 255, 255), -1);
    // bitwise_and(img_clone, img_clone, img_clone, mask);
    std::vector<cv::Vec3f> circles =
        findCircle(img_clone, _color, _circle_number);  // 获取所有圆参数
    try {
      if (circles.empty() || circles[0][2] == 0 ||
          (circles.size() != _circle_number)) {  // 为空，跳过
        continue;
      }
    } catch (const std::exception& e) {
      continue;
    }

    sample_time++;  // 采样成功次数更新
    // 记录所有圆
    x_table_1.push_back(circles[0][0]);
    y_table_1.push_back(circles[0][1]);
    r_table_1.push_back(circles[0][2]);

    cv::circle(img, cv::Point(int(circles[0][0]), int(circles[0][1])),
               int(circles[0][2]), cv::Scalar(255, 0, 100), 2);
    if (ball_detect_config_.ball_edit == 1) {
      cv::imshow("circles", img);
      cv::waitKey(100);
    }
  }
  cv::destroyAllWindows();
  if (detect_fail) {
    CirclesResult result;
    result.circles = {cv::Vec3f(0, 0, 0)};
    result.image = img;
    return result;
  } else {
    // 对x数据进行标准化处理(对全部xyr数据进行处理，容易会出现三者相与之后全为false的情况)
    std::vector<bool> x_select_1 =
        zScore(x_table_1, ball_detect_config_.x_select_ref);

    // x的标准化数据全为false，误差率过大，丢弃
    if (!std::any_of(x_select_1.begin(), x_select_1.end(),
                     [](bool x) { return x; })) {
      CirclesResult result;
      result.circles = {cv::Vec3f(0, 0, 0)};
      result.image = img;
      return result;
    } else {
      // 计算标准化结果为true(排除坏点干扰)的数据的平均值
      double x_sum_1 = 0.0;
      double y_sum_1 = 0.0;
      double r_sum_1 = 0.0;
      int real_time_1 = 0;

      for (size_t i = 0; i < x_select_1.size(); i++) {
        if (x_select_1[i]) {
          x_sum_1 += x_table_1[i];
          y_sum_1 += y_table_1[i];
          r_sum_1 += r_table_1[i];
          real_time_1++;
        }
      }

      CirclesResult result;
      result.circles[0] =
          cv::Vec3f(int(x_sum_1 / real_time_1), int(y_sum_1 / real_time_1),
                    int(r_sum_1 / real_time_1));
      result.image = img;
      return result;
    }
  }
}

std::vector<cv::Vec3f> BallDetector::calculateStdResults(
    const std::vector<std::vector<double>>& _x_table,
    const std::vector<std::vector<double>>& _y_table,
    const std::vector<std::vector<double>>& _r_table,
    const std::vector<bool>& _x_select_1, const std::vector<bool>& _x_select_2,
    const std::vector<bool>& _x_select_3) {
  double x_sum_1 = 0.0;
  double y_sum_1 = 0.0;
  double r_sum_1 = 0.0;
  int real_time_1 = 0;
  double x_sum_2 = 0.0;
  double y_sum_2 = 0.0;
  double r_sum_2 = 0.0;
  int real_time_2 = 0;
  double x_sum_3 = 0.0;
  double y_sum_3 = 0.0;
  double r_sum_3 = 0.0;
  int real_time_3 = 0;
  for (int i = 0; i < 3; i++) {
    switch (i) {
      case 0:
        for (size_t j = 0; j < _x_select_1.size(); j++) {
          if (_x_select_1[j]) {
            x_sum_1 += _x_table[0][j];
            y_sum_1 += _y_table[0][j];
            r_sum_1 += _r_table[0][j];
            real_time_1++;
          }
        }
        break;
      case 1:
        for (size_t j = 0; j < _x_select_2.size(); j++) {
          if (_x_select_2[j]) {
            x_sum_2 += _x_table[1][j];
            y_sum_2 += _y_table[1][j];
            r_sum_2 += _r_table[1][j];
            real_time_2++;
          }
        }
        break;
      case 2:
        for (size_t j = 0; j < _x_select_3.size(); j++) {
          if (_x_select_3[j]) {
            x_sum_3 += _x_table[2][j];
            y_sum_3 += _y_table[2][j];
            r_sum_3 += _r_table[2][j];
            real_time_3++;
          }
        }
        break;
    }
  }
  std::vector<cv::Vec3f> caculate_circles(3);
  caculate_circles[0] =
      cv::Vec3f(int(x_sum_1 / real_time_1), int(y_sum_1 / real_time_1),
                int(r_sum_1 / real_time_1));

  caculate_circles[1] =
      cv::Vec3f(int(x_sum_2 / real_time_2), int(y_sum_2 / real_time_2),
                int(r_sum_2 / real_time_2));
  caculate_circles[2] =
      cv::Vec3f(int(x_sum_3 / real_time_3), int(y_sum_3 / real_time_3),
                int(r_sum_3 / real_time_3));
  return caculate_circles;
}

void BallDetector::standarDizate(std::vector<std::vector<double>>* _x_table,
                                 std::vector<std::vector<double>>* _y_table,
                                 std::vector<std::vector<double>>* _r_table,
                                 std::vector<bool>* _x_select_1,
                                 std::vector<bool>* _x_select_2,
                                 std::vector<bool>* _x_select_3) {
  for (int i = 0; i < 3; i++) {
    switch (i) {
      case 0:
        if ((*_x_table).at(0).size() == 0) {
          (*_x_select_1).push_back(true);
          (*_x_table).at(0).push_back(0);
          (*_y_table).at(0).push_back(0);
          (*_r_table).at(0).push_back(0);
        } else {
          (*_x_select_1) =
              zScore((*_x_table).at(0), ball_detect_config_.x_select_ref);
        }
        break;
      case 1:
        if ((*_x_table).at(0).size() == 0) {
          (*_x_select_2).push_back(true);
          (*_x_table).at(1).push_back(0);
          (*_y_table).at(1).push_back(0);
          (*_r_table).at(1).push_back(0);
        } else {
          (*_x_select_2) =
              zScore((*_x_table).at(1), ball_detect_config_.x_select_ref);
        }
        break;
      case 2:
        if ((*_x_table).at(2).size() == 0) {
          // std::cout << x_table[2].size() << std::endl;
          (*_x_select_3).push_back(true);
          (*_x_table).at(2).push_back(0);
          (*_y_table).at(2).push_back(0);
          (*_r_table).at(2).push_back(0);
        } else {
          (*_x_select_3) =
              zScore((*_x_table).at(2), ball_detect_config_.x_select_ref);
        }
        break;
    }
  }
}

void BallDetector::recordXYRParam(std::vector<std::vector<double>>* _x_table,
                                  std::vector<std::vector<double>>* _y_table,
                                  std::vector<std::vector<double>>* _r_table,
                                  cv::Mat* image, int num,
                                  const std::vector<cv::Vec3f>& circles) {
  for (int i = 0; i < num; i++) {
    if ((circles[i][0] < (int(image->cols / 3 + 25))) &&
        (circles[i][1] > 300)) {
      // 左下
      (*_x_table).at(0).push_back(circles[i][0]);
      (*_y_table).at(0).push_back(circles[i][1]);
      (*_r_table).at(0).push_back(circles[i][2]);
    } else if ((circles[i][0] > int(image->cols / 3 - 25)) &&
               (circles[i][1] < 300)) {
      // 中上
      (*_x_table).at(1).push_back(circles[i][0]);
      (*_y_table).at(1).push_back(circles[i][1]);
      (*_r_table).at(1).push_back(circles[i][2]);
    } else if ((circles[i][0] > int(image->cols * 2 / 3 - 25)) &&
               (circles[i][1] > 300)) {
      // 右下
      (*_x_table).at(2).push_back(circles[i][0]);
      (*_y_table).at(2).push_back(circles[i][1]);
      (*_r_table).at(2).push_back(circles[i][2]);
    }
  }
}

CirclesResult BallDetector::detectCircles(
    fruit ::detect::DetectPool* _detectpool, int _color) {
  std::cout << "\n------开始检测圆位置------\n" << std::endl;
  double t0 = cv::getTickCount();  // 初始时间

  int detect_fail = 0;  // 初始化检测失败标志位
  int sample_time = 0;  // 初始化采样次数为0
  // 初始化数据记录表为空
  std::vector<std::vector<double>> x_table(3);
  std::vector<std::vector<double>> y_table(3);
  std::vector<std::vector<double>> r_table(3);
  // 获取图像帧
  cv::Mat img;
  while (sample_time < sample_times_) {
    detect_fail = 0;
    // 超时退出
    double task_time = (cv::getTickCount() - t0) / cv::getTickFrequency();
    if (!time_out_en_ && task_time >= time_out_) {
      std::cout << "\n超时未检测到圆\n" << std::endl;
      detect_fail = 1;  // 失败置位
      break;
    }
    _detectpool->frame_mutex_.lock();
    img = _detectpool->share_ori_frame_;
    _detectpool->frame_mutex_.unlock();
    if (img.empty() || cv::waitKey(30) == 27) {
      continue;
    }
    cv::Mat img_clone = img.clone();
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
    cv::rectangle(mask, cv::Point((int)img_clone.cols / 3 - 25, 0),
                  cv::Point((int)img_clone.cols * 2 / 3 + 25, 150),
                  cv::Scalar(255, 255, 255), -1);
    cv::rectangle(mask, cv::Point(0, 300),
                  cv::Point((int)img_clone.cols / 3 + 25, (int)img_clone.rows),
                  cv::Scalar(255, 255, 255), -1);
    cv::rectangle(mask, cv::Point((int)img_clone.cols * 2 / 3 - 25, 300),
                  cv::Point((int)img_clone.cols, (int)img_clone.rows),
                  cv::Scalar(255, 255, 255), -1);
    bitwise_and(mask, img_clone, img_clone);
    std::vector<cv::Vec3f> circles = findCircle(img_clone, _color);
    // 获取所有圆，个数不确定，最多为三个
    if (circles.empty() || circles[0][2] == 0) {  // 为空，跳过
      continue;
    }
    int num = circles.size();  // 计圆数
    if (num > 0) {
      sample_time++;
      recordXYRParam(&x_table, &y_table, &r_table, &img_clone, num, circles);
      // 在原图上画圆
      for (int i = 0; i < num; i++) {
        cv::circle(img, cv::Point(int(circles[i][0]), int(circles[i][1])),
                   int(circles[i][2]), cv::Scalar(255, 0, 100), 2);
      }
      if (ball_detect_config_.ball_edit == 1) {
        cv::imshow("circles", img);
        cv::waitKey(100);
      }
    }
  }
  cv::destroyAllWindows();
  if (detect_fail == 1) {
    CirclesResult result;
    result.circles = {cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0),
                      cv::Vec3f(0, 0, 0)};
    result.image = img;
    return result;
  } else {
    // 对x数据进行Standardization(对全部xyr数据进行处理，容易会出现三者相与之后全为false的情况)
    std::vector<bool> x_select_1, x_select_2, x_select_3;
    standarDizate(&x_table, &y_table, &r_table, &x_select_1, &x_select_2,
                  &x_select_3);
    // x的标准化数据全为false，误差率过大，丢弃

    if (!std::any_of(x_select_1.begin(), x_select_1.end(),
                     [](bool x) { return x; }) ||
        !std::any_of(x_select_2.begin(), x_select_2.end(),
                     [](bool x) { return x; }) ||
        !std::any_of(x_select_3.begin(), x_select_3.end(),
                     [](bool x) { return x; })) {
      CirclesResult result;
      result.circles = {cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0),
                        cv::Vec3f(0, 0, 0)};
      result.image = img;
      return result;
    } else {
      // Calculate standardization results为true(排除坏点干扰)的数据的平均值
      CirclesResult result;
      result.circles = calculateStdResults(x_table, y_table, r_table,
                                           x_select_1, x_select_2, x_select_3);
      result.image = img;
      return result;
    }
  }
}

TypeResult BallDetector::detectType(fruit::detect::DetectPool* _detectpool) {
  double best_match = 0.0;  // 初始化最佳匹配的种类的相似度
  int fruit_type = 0;       // 初始化水果种类为0(为0即识别失败)
  cv::Point left_top(0, 0);  // 初始化最佳匹配左上角点位置(画边界框时使用)
  cv::Point right_bottom(0, 0);  // 初始化最佳匹配右下角点位置

  double t0 = cv::getTickCount();  // 初始时间
  // 当在时间范围内，且未检测到水果种类
  std::vector<std::vector<double>> similarity;  // 初始化相似度表
  while (fruit_type == 0) {
    double task_time = (cv::getTickCount() - t0) / cv::getTickFrequency();
    if (!time_out_en_ && task_time >= time_out_) {
      std::cout << "超时!" << std::endl;
      break;
    }
    // 先检测圆所在的位置，目的在于将圆区域提取出来，尽可能地排除背景干扰
    CirclesResult result = detectCircles(_detectpool, 1, 1);
    cv::Vec3f circle = result.circles[0];
    cv::Mat img = result.image;

    // 根据圆的xyr参数计算圆的外接正方形的左上(t_l)与右下(r_b)角点位置
    int l_t_x = circle[0] - circle[2];
    int l_t_y = circle[1] - circle[2];
    int r_b_x = circle[0] + circle[2];
    int r_b_y = circle[1] + circle[2];
    // 圆的外接正方形边界处理(防止有角点超出画面而在截取时报错)
    l_t_x = l_t_x >= 0 ? l_t_x : 0;
    l_t_y = l_t_y >= 0 ? l_t_y : 0;
    r_b_x = r_b_x <= img.cols - 1 ? r_b_x : img.cols - 1;
    r_b_y = r_b_y <= img.rows - 1 ? r_b_y : img.rows - 1;
    // 截取
    cv::Mat img_roi =
        img(cv::Rect(l_t_x, l_t_y, r_b_x - l_t_x + 1, r_b_y - l_t_y + 1));

    // 排除因未检测到圆而未截取到roi，此时将在全视野内进行识别，会带来更多的背景干扰
    if (img_roi.empty()) {
      std::cout << "\n检测圆失败，无法识别种类或精度将降低" << std::endl;
      img_roi = img;
      continue;
    }
    // 种类识别
    std::cout << "\n------开始识别种类------\n" << std::endl;
    for (int i = 1; i <= 4; i++) {  // 遍历水果种类，1 2 3 4
      int pic = 0;
      std::vector<double> sim_data;
      for (int j = 1; j <= ball_detect_config_.n_pic_fer_type;
           j++) {  // 遍历这种水果的所有模板
        std::string name =
            std::to_string(i) + std::to_string(j) + ".jpg";  // 合成模板名
        cv::Mat temp = cv::imread("template_f/" + name,
                                  cv::IMREAD_GRAYSCALE);  // 读取模板图片
        int temp_h = temp.rows;  // 获得该模板的高和宽，画边界框时用
        int temp_w = temp.cols;

        // 判断模板尺寸是否超过输入图像尺寸(输入图像应是截取出来的圆的区域，若圆过圆会有小于模板尺寸的可能)
        int roi_h = img_roi.rows;
        int roi_w = img_roi.cols;
        if (temp_h > roi_h || temp_w > roi_w) {
          std::cout << "\nErr! templ oversize\n" << std::endl;
          TypeResult result;
          result.fruit_type = 0;
          result.similarity = {{0}};
        }
        // 模板匹配
        cv::Mat res;
        cv::matchTemplate(img_roi, temp, res, cv::TM_CCOEFF_NORMED);
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(res, &min_val, &max_val, &min_loc, &max_loc);
        sim_data.push_back(max_val);
        if (max_val > best_match) {
          best_match = max_val;  // 更新最佳匹配的种类
          fruit_type = i;        // 记录当前种类
          left_top =
              max_loc;  // 记录左上角点位置，即minMaxLoc函数处理得的max_loc
          right_bottom =
              left_top + cv::Point(temp_w, temp_h);  // 记录右下角点位置
        }
        std::cout << "type:" << i << " pic:" << j << " max_val:" << max_val
                  << " currently detected as:" << fruit_type << std::endl;
      }
      //  结果展示

      std::cout << "\n" << std::endl;
      similarity.push_back(sim_data);  // 数据记录
    }
    cv::rectangle(img_roi, left_top, right_bottom, cv::Scalar(255, 0, 255),
                  2);  // 图像上画边界框
    cv::putText(img_roi, "type" + std::to_string(fruit_type), left_top,
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0),
                2);  // 图像上显示结果
    cv::namedWindow("match");
    cv::moveWindow("match", 10, 0);
    cv::imshow("match", img_roi);  // 显示图像
    cv::waitKey(1500);
  }

  cv::destroyAllWindows();
  std::cout << "fruit type detected as: " << fruit_type << std::endl;
  TypeResult result;  // 定义返回结果
  result.fruit_type = fruit_type;
  result.similarity = similarity;
}

int BallDetector::checkGrab(int _cam, int _color) {
  /**
   * 用不上
   */
  return 0;
}

BallDetector::~BallDetector() {}
}  // namespace ball
}  // namespace fruit