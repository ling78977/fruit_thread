#include "src/module/motion/motion.h"

namespace fruit {
namespace motion {

void armStandby(int direction, int const claw) {
  // 待命位(左中右)
  arm_device.armSerialServoWrite6(
      direction + arm_device.delta_yaw_servo_1_, arm_device.level_servo_2_,
      arm_device.level_servo_3_, arm_device.level_servo_4_, 90, claw, 800);
  usleep(1000000);
}

void armScout(int direction) {
  arm_device.armSerialServoWrite6(direction + arm_device.delta_yaw_servo_1_, 90,
                                  90, 0, 90, arm_device.loose_, 800);
  usleep(1000000);
}
void armMidway() {
  // 中途防碰撞位
  arm_device.armSerialServoWrite6(90, 90, 30, 30, 90, arm_device.loose_, 800);
  usleep(1000000);
}
void armGrab(std::vector<double> data_ik) {
  // 居中待命
  arm_device.armSerialServoWrite6(
      90 + arm_device.delta_yaw_servo_1_, arm_device.level_servo_2_,
      arm_device.level_servo_3_, arm_device.level_servo_4_, 90,
      arm_device.loose_, 800);
  usleep(1000000);
  // 中途防撞
  armMidway();
  // 张开,前出,预备
  arm_device.armSerialServoWrite6(data_ik[0], data_ik[1], data_ik[2],
                                  data_ik[3] - 10, 90, arm_device.loose_, 500);
  usleep(1000000);
  // 张开,前出,到位
  arm_device.armSerialServoWrite6(data_ik[0], data_ik[1], data_ik[2] - 2,
                                  data_ik[3], 90, arm_device.loose_, 500);
  usleep(1000000);
  // 抓紧
  arm_device.armSerialServoWrite6(data_ik[0], data_ik[1], data_ik[2] - 2,
                                  data_ik[3], 90, arm_device.tight_, 500);
  usleep(1000000);
  // 抓紧，摘下
  arm_device.armSerialServoWrite6(data_ik[0], data_ik[1], data_ik[2] - 2,
                                  data_ik[3] - 25, 90, arm_device.tight_, 500);
  usleep(1000000);
  // 回中
  arm_device.armSerialServoWrite6(
      90 + arm_device.delta_yaw_servo_1_, arm_device.level_servo_2_,
      arm_device.level_servo_3_, arm_device.level_servo_4_, 90,
      arm_device.tight_, 800);
  usleep(1000000);
  // 回中
  arm_device.armSerialServoWrite6(
      90 + arm_device.delta_yaw_servo_1_, arm_device.level_servo_2_,
      arm_device.level_servo_3_, arm_device.level_servo_4_, 90,
      arm_device.tight_, 800);
  usleep(1000000);
}
void armLook(int direction) {
  arm_device.armSerialServoWrite6(direction + arm_device.delta_yaw_servo_1_,
                                  arm_device.level_servo_2_ - 62, -25, 90, 90,
                                  arm_device.loose_, 800);
}
void putFruitToCup(int cup) {
  // 翻转
  arm_device.armSerialServoWrite6(25 + cup * 25, 90, 110, 145, 90,
                                  arm_device.tight_, 800);
  usleep(800000);
  // 松开爪子
  arm_device.armSerialServoWrite6(25 + cup * 25, 90, 110, 145, 90,
                                  arm_device.loose_, 800);
  usleep(1000000);
  // 机械臂直立
  arm_device.armSerialServoWrite6(25 + cup * 25, 90, 90, 90, 90,
                                  arm_device.loose_, 800);
  usleep(800000);
  // 居中待命
  arm_device.armSerialServoWrite6(
      90, arm_device.level_servo_2_, arm_device.level_servo_3_,
      arm_device.level_servo_4_, 90, arm_device.loose_, 800);
  usleep(800000);
}
void putFruitsToBasket(int direction, int cup) {
  // 预备动作
  arm_device.armSerialServoWrite6(25 + cup * 25, 65, 145, 135, 90, 30, 800);
  usleep(800000);
  // 靠近杯子
  arm_device.armSerialServoWrite6(25 + cup * 25, 70, 180, 110, 90, 30, 800);
  usleep(800000);
  // 夹紧
  arm_device.armSerialServoWrite6(25 + cup * 25, 70, 180, 110, 90, 85, 800);
  usleep(800000);
  // 取出杯子
  arm_device.armSerialServoWrite6(25 + cup * 25, 30, 155, 170, 90, 85, 800);
  usleep(800000);
  // 转到对应方向
  arm_device.armSerialServoWrite6(abs((180 - direction) - 15), 30, 155, 170, 90,
                                  85, 800);
  usleep(800000);
  // 伸出去
  arm_device.armSerialServoWrite6(abs((180 - direction) - 15), 125, 140, 85, 90,
                                  85, 800);
  usleep(800000);
  // 倒
  arm_device.armSerialServoWrite6(abs((180 - direction) - 15), 125, 140, 85,
                                  270, 85, 500);
  usleep(1500000);
  // 5号舵机回位
  arm_device.armSerialServoWrite6(abs((180 - direction) - 15), 125, 140, 85, 90,
                                  85, 800);
  usleep(800000);
  // 收回来
  arm_device.armSerialServoWrite6(abs((180 - direction) - 15), 30, 155, 170, 90,
                                  85, 800);
  usleep(800000);
  // 转回去
  arm_device.armSerialServoWrite6(25 + cup * 25, 30, 155, 170, 90, 85, 800);
  usleep(800000);
  // 放回对应位置
  arm_device.armSerialServoWrite6(25 + cup * 25, 70 - 5, 180, 110, 90, 85, 800);
  usleep(800000);
  // 松开
  arm_device.armSerialServoWrite6(25 + cup * 25, 70 - 5, 180, 110, 90, 30, 800);
  usleep(800000);
  // 机械臂缩回，防止撞倒
  arm_device.armSerialServoWrite6(25 + cup * 25, 30, 125, 180, 90, 30, 800);
  usleep(800000);
  // 居中待命
  arm_device.armSerialServoWrite6(
      90, arm_device.level_servo_2_, arm_device.level_servo_3_,
      arm_device.level_servo_4_, 90, arm_device.loose_, 800);  // 回
  usleep(800000);
}

}  // namespace motion
}  // namespace fruit