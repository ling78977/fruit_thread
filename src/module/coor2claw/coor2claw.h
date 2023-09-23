#ifndef SRC_MODULE_COOR2CLAW_COOR2CLAW_H_
#define SRC_MODULE_COOR2CLAW_COOR2CLAW_H_
// #include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace fruit {
namespace coor2claw {
typedef struct {
  double pi;
  double RE2RA;
  double RA2RE;

  double x_camera_to_base_l;
  double y_camera_to_base_l;
  double z_camera_to_base_l;

  double x_camera_to_base_m;
  double y_camera_to_base_m;
  double z_camera_to_base_m;

  double x_camera_to_base_r;
  double y_camera_to_base_r;
  double z_camera_to_base_r;

  double d_object_to_camera_upper;
  double d_object_to_camera_lower;

  double f_x_div_dx_l;
  double f_y_div_dy_l;
  double u0_l;
  double v0_l;

  double f_x_div_dx_m;
  double f_y_div_dy_m;
  double u0_m;
  double v0_m;

  double f_x_div_dx_r;
  double f_y_div_dy_r;
  double u0_r;
  double v0_r;

  double j5_claw_delta_y;
  double j5_claw_delta_z;

  int sight_middle;
  int edit;
} Coor2ClawConfig;

class Coor2Claw {
 private:
  Coor2ClawConfig coor_2_claw_config_;

 public:
  Coor2Claw(const std::string _coor_2_claw_config);
  std::vector<double> armCoord2Claw(cv::Point2d pixel);
};
}  // namespace coor2claw
}  // namespace fruit
#endif