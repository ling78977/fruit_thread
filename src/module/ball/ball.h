#ifndef SRC_MODULE_BALL_BALL_H_
#define SRC_MODULE_BALL_BALL_H_


//TODO 把检测球的动作函数改为从DETECTPOLL中的属性
#include <iostream>
#include <opencv2/opencv.hpp>
namespace fruit {
namespace ball {

typedef struct {
  std::vector<cv::Vec3f> circles;  // 圆参数
  cv::Mat image;                   // 图像帧
} CirclesResult;
typedef struct {
  int fruit_type;                               // 水果种类
  std::vector<std::vector<double>> similarity;  // 相似度表
} TypeResult;
typedef struct {
  int h_yellow_lower;
  int h_yellow_upper;
  int l_yellow_lower;
  int l_yellow_upper;
  int s_yellow_lower;
  int s_yellow_upper;

  int r_white_lower;
  int r_white_upper;
  int g_white_lower;
  int g_white_upper;
  int b_white_lower;
  int b_white_upper;

  int morphology_kernel_size_ball;

  int erode_dilate_kernel_size_ball;

  int erode_iterations_ball;

  int dilate_iterations_ball;

  int canny_threshold1;
  int canny_threshold2;

  int HoughCircles_param2;

  int rmin;
  int rmax;

  int rmin_threshold;

  double x_select_ref;

  int n_pic_fer_type;

  double ratio;

  int ball_edit;
} BallDetectConfig;

class BallDetector {
 private:
  int sample_times_ = 9;
  int time_out_ = 50;
  bool time_out_en_;
  BallDetectConfig ball_detect_config_;

  // 滑动条窗口名
  std::string yellow_hls_window_ = "yellow_hls_trackbar";
  std::string white_rgb_window_ = "white_rgb_trackbar";

  const cv::Mat hls_trackbar_yellow_ = cv::Mat::zeros(1, 300, CV_8UC1);
  const cv::Mat rgb_trackbar_white_ = cv::Mat::zeros(1, 300, CV_8UC1);

 public:
  /**
   *@brief 数据标准化，剔除坏点
   *@param _stats  输入数组
   *@param _ref  参考值（默认0.8）
   *@return  输入数组满足参考值条件的对应的布尔数组
   */
  std::vector<bool> zScore(const std::vector<double>& _stats, double _ref);

  cv::Mat colorFilter(cv::Mat _img, int _color);

  /**
   *@brief 在bgr图上上找到目标数量个目标颜色的圆(即乒乓球)，得到其圆参数
   *@param _frame 图像帧
   *@param _color: 目标颜色
   *@param _circle_number: 当前树上应有目标颜色的果的数量
   *@return 圆参数
   */
  std::vector<cv::Vec3f> findCircle(const cv::Mat& _frame, int _color,
                                    int _circle_number);

  std::vector<cv::Vec3f> findCircle(const cv::Mat& _frame, int _color);
  /**
   *@brief
   *实时检测视野内指定数量个指定颜色的圆，并多次采样求平均提高定位精度，返回最左侧的一个圆的参数
   *@param _cam  摄像头
   *@param _color  目标颜色
   *@param _circle_number  当前树上应有目标颜色的果的数量
   *@return 最终圆参数和图像帧
   */
  CirclesResult detectCircles(cv::VideoCapture _cam, int _color,
                              int _circle_number, bool _grab = true);

  /**
   *@brief
   *实时检测视野内指定数量个指定颜色的圆，并多次采样求平均提高定位精度，返回最左侧的一个圆的参数
   *@param _cam  摄像头
   *@param _color  目标颜色
   *@return 最终圆参数和图像帧
   */
  CirclesResult detectCircles(cv::VideoCapture _cam, int _color);

  /**
   *@brief 根据所在区域，实时识别视野内对应个数的黄色圆上的贴纸种类
   *@param cap: 摄像头
   */
  TypeResult detectType(cv::VideoCapture _cap);
  int checkGrab(int _cam, int _color);
  BallDetector(const std::string _ball_detect_config, int _sample_times,
               int _time_out, bool _time_out_en);
  ~BallDetector();
};

}  // namespace ball
}  // namespace fruit
#endif