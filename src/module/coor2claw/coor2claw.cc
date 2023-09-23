#include "src/module/coor2claw/coor2claw.h"

namespace fruit {
namespace coor2claw {

Coor2Claw::Coor2Claw(const std::string _coor_2_claw_config) {
  cv::FileStorage coor2claw(_coor_2_claw_config, cv::FileStorage::READ);
  if (!coor2claw.isOpened()) {
    std::cout << "\033[31m 打开coor2claw文件失败 \033[0m" << std::endl;
  }
  coor2claw["PI"] >> coor_2_claw_config_.pi;
  coor2claw["RE2RA"] >> coor_2_claw_config_.RE2RA;
  coor2claw["RA2RE"] >> coor_2_claw_config_.RA2RE;
  coor2claw["X_CAMERA_TO_BASE_L"] >> coor_2_claw_config_.x_camera_to_base_l;
  coor2claw["Y_CAMERA_TO_BASE_L"] >> coor_2_claw_config_.y_camera_to_base_l;
  coor2claw["Z_CAMERA_TO_BASE_L"] >> coor_2_claw_config_.z_camera_to_base_l;

  coor2claw["X_CAMERA_TO_BASE_M"] >> coor_2_claw_config_.x_camera_to_base_m;
  coor2claw["Y_CAMERA_TO_BASE_M"] >> coor_2_claw_config_.y_camera_to_base_m;
  coor2claw["Z_CAMERA_TO_BASE_M"] >> coor_2_claw_config_.z_camera_to_base_m;

  coor2claw["X_CAMERA_TO_BASE_R"] >> coor_2_claw_config_.x_camera_to_base_r;
  coor2claw["Y_CAMERA_TO_BASE_R"] >> coor_2_claw_config_.y_camera_to_base_r;
  coor2claw["Z_CAMERA_TO_BASE_R"] >> coor_2_claw_config_.z_camera_to_base_r;

  coor2claw["D_OBJECT_TO_CAMERA_UPPER"] >>
      coor_2_claw_config_.d_object_to_camera_upper;
  coor2claw["D_OBJECT_TO_CAMERA_LOWER"] >>
      coor_2_claw_config_.d_object_to_camera_lower;

  coor2claw["F_X_DIV_DX_L"] >> coor_2_claw_config_.f_x_div_dx_l;
  coor2claw["F_Y_DIV_DY_L"] >> coor_2_claw_config_.f_y_div_dy_l;
  coor2claw["U0_L"] >> coor_2_claw_config_.u0_l;
  coor2claw["V0_L"] >> coor_2_claw_config_.v0_l;

  coor2claw["F_X_DIV_DX_M"] >> coor_2_claw_config_.f_x_div_dx_m;
  coor2claw["F_Y_DIV_DY_M"] >> coor_2_claw_config_.f_y_div_dy_m;
  coor2claw["U0_M"] >> coor_2_claw_config_.u0_m;
  coor2claw["V0_M"] >> coor_2_claw_config_.v0_m;

  coor2claw["F_X_DIV_DX_R"] >> coor_2_claw_config_.f_x_div_dx_r;
  coor2claw["F_Y_DIV_DY_R"] >> coor_2_claw_config_.f_y_div_dy_r;
  coor2claw["U0_R"] >> coor_2_claw_config_.u0_r;
  coor2claw["V0_R"] >> coor_2_claw_config_.v0_r;

  coor2claw["J5_CLAW_DELTA_Y"] >> coor_2_claw_config_.j5_claw_delta_y;
  coor2claw["J5_CLAW_DELTA_Z"] >> coor_2_claw_config_.j5_claw_delta_z;
  coor2claw["SIGHT_MIDDLE"] >> coor_2_claw_config_.sight_middle;
  coor2claw["EDIT"] >> coor_2_claw_config_.edit;
  coor2claw.release();
}
std::vector<double> Coor2Claw::armCoord2Claw(cv::Point2d pixel) {
  // 像素平面坐标系 --> 相平面坐标系
  std::vector<double> coord;
  if (pixel.y < coor_2_claw_config_.sight_middle) {  // 中间
    double x_image = pixel.x - coor_2_claw_config_.u0_m;
    double y_image = pixel.y - coor_2_claw_config_.v0_m;
    double D_object_to_camera = coor_2_claw_config_.d_object_to_camera_upper;
    double x_camera =
        x_image * D_object_to_camera / coor_2_claw_config_.f_x_div_dx_m;
    double y_camera =
        y_image * D_object_to_camera / coor_2_claw_config_.f_y_div_dy_m;
    coord.push_back(x_camera - coor_2_claw_config_.x_camera_to_base_m);
    coord.push_back(D_object_to_camera -
                    coor_2_claw_config_.y_camera_to_base_m);
    // 写入z，相机坐标系y轴与底座坐标系z轴平行
    coord.push_back(coor_2_claw_config_.z_camera_to_base_m - y_camera);
  } else {
    if (pixel.x < (640 / 3 + 25)) {  // 左下
      double x_image = pixel.x - coor_2_claw_config_.u0_l;
      double y_image = pixel.y - coor_2_claw_config_.v0_l;
      double D_object_to_camera = coor_2_claw_config_.d_object_to_camera_lower;
      double x_camera =
          x_image * D_object_to_camera / coor_2_claw_config_.f_x_div_dx_l;
      double y_camera =
          y_image * D_object_to_camera / coor_2_claw_config_.f_y_div_dy_l;
      coord.push_back(x_camera - coor_2_claw_config_.x_camera_to_base_l);
      coord.push_back(D_object_to_camera -
                      coor_2_claw_config_.y_camera_to_base_l);
      // 写入z，相机坐标系y轴与底座坐标系z轴平行
      coord.push_back(coor_2_claw_config_.z_camera_to_base_l - y_camera);
    } else {
      double x_image = pixel.x - coor_2_claw_config_.u0_r;
      double y_image = pixel.y - coor_2_claw_config_.v0_r;
      double D_object_to_camera = coor_2_claw_config_.d_object_to_camera_lower;
      double x_camera =
          x_image * D_object_to_camera / coor_2_claw_config_.f_x_div_dx_r;
      double y_camera =
          y_image * D_object_to_camera / coor_2_claw_config_.f_y_div_dy_r;
      coord.push_back(x_camera - coor_2_claw_config_.x_camera_to_base_r);
      coord.push_back(D_object_to_camera -
                      coor_2_claw_config_.y_camera_to_base_r);
      // 写入z，相机坐标系y轴与底座坐标系z轴平行
      coord.push_back(coor_2_claw_config_.z_camera_to_base_r - y_camera);
    }
  }
  double theta = std::atan2(coord[1], coord[0]);

  // 写入夹取姿态
  // Roll值选取，仰夹为(-90, 0)，水平夹为-90，俯夹为(-90, -180)
  // 根据物体高度进行俯仰角判断（或可自定义一映射关系使抓取姿态变化更柔顺）

  if (coord[2] >= 25) {
    coord.push_back(-80.0);  // 10度仰角夹取
  } else {
    coord.push_back(-100.0);  // 10度俯角夹取
  }
  //    std::cout << "俯仰角度: " << coord[3] << std::endl;
  coord.push_back(0.0);  // Pitch
  coord.push_back(0.0);  // Yaw
  // 解算服务器不解算夹具，故需引入夹爪长度，根据俯仰角，目标点与原点的夹角更新xyz
  // 夹爪实际长度
  double l_j5_claw = std::sqrt(coor_2_claw_config_.j5_claw_delta_y *
                                   coor_2_claw_config_.j5_claw_delta_y +
                               coor_2_claw_config_.j5_claw_delta_z *
                                   coor_2_claw_config_.j5_claw_delta_z);
  // 5号舵机末端位于夹爪基线所在直线下方，俯仰角需相应减去一个偏移角
  double roll_claw = (coord[3] + 180) * coor_2_claw_config_.RE2RA -
                     std::atan2(coor_2_claw_config_.j5_claw_delta_z,
                                coor_2_claw_config_.j5_claw_delta_y);
  // 投影在xoy平面内计算
  coord[0] = coord[0] - l_j5_claw * std::sin(roll_claw) * std::cos(theta);
  coord[1] = coord[1] - l_j5_claw * std::sin(roll_claw) * std::sin(theta);
  coord[2] = coord[2] + l_j5_claw * std::cos(roll_claw);  // roll_claw > 90°
  return coord;
}
}  // namespace coor2claw
}  // namespace fruit