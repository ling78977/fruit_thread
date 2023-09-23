#ifndef FRUIT_SRC_MODULE_DETECT_DETECT_H_
#define FRUIT_SRC_MODULE_DETECT_DETECT_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

namespace fruit {
namespace detect {

typedef struct {
  bool flag = false;
  std::vector<int> data;
} ResulteQR;

typedef struct {
  bool flag = false;
  cv::Mat img;
} FindTurnSign;

typedef struct {
  // 蓝 HSV 参数
  int h_blue_max;
  int h_blue_min;
  int s_blue_max;
  int s_blue_min;
  int v_blue_max;
  int v_blue_min;
  int morphology_kernel_size_turn;
  int erode_dilate_kernel_size_turn;
  int erode_iterations_turn;
  int dilate_iterations_turn;
  int area_threshold;
  double max_val_threshold;
  int turn_edit_;

  int turn_detect_sample_times = 9;
  int turn_detect_timeout = 10;
  bool turn_detect_timeout_en = false;

  cv::Mat turn_detect_template_left;
  cv::Mat turn_detect_template_right;
  cv::Mat turn_detect_template_ahead;
} DetectTurnConfig;

typedef struct {
  int qr_detect_timeout_sec;
  double qr_detect_start_time;
  bool qr_detect_timeout_en = false;
  int qr_edit;
} DetectQRConfig;

class DetectPool {
 public:
  DetectPool(const std::string detect_config,  int qr_timeout, bool qr_timeout_en,int trun_timeout,bool turn_timeout_en,int turn_sample_times);

  ~DetectPool();

  /**
   * @brief 采集图像线程
   *
   * @param cap 摄像头对象
   */
  void collecteImage(cv::VideoCapture& cap);

  /**
   * @brief 二维码检测线程
   *
   * @return ResulteQR 返回二维码检测结果
   */
  ResulteQR detectQR();

  /**
   * @brief 方向检测线程
   *
   * @return int 返回检测结果
   */
  int detectTurn();

 private:
  /**
   * @brief 二维码检测线程的模板函数
   *
   * @param roi roi图
   * @return int 一帧识别到的方向
   */
  int detectTurnMatchTemplate(cv::Mat& roi);

  /**
   * @brief 求众数
   *
   * @param nums 数据容器
   * @return int 众数
   */
  int detectTurnArgmax(std::vector<int> nums);

  /**
   * @brief 检测方向
   *
   * @param frame 一帧图像
   * @return FindTurnSign 结构体，包含检测结果bool值和方向牌roi图
   */
  FindTurnSign detectTurnFindSign(cv::Mat frame);

  /**
   * @brief 方向检测预处理
   *
   * @param frame 一帧图像
   * @return cv::Mat 预处理结果
   */
  cv::Mat detectTurnPre(cv::Mat frame);

 private:
  cv::Mat share_ori_frame_;
  std::mutex frame_mutex_;
  DetectQRConfig detect_qr_config_;
  DetectTurnConfig detect_turn_config_;
  cv::QRCodeDetector qr_decoder_ = cv::QRCodeDetector();
};

}  // namespace detect
}  // namespace fruit
#endif