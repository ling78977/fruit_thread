
#ifndef FRUIT_SRC_MODULE_ARM_ARM_H_
#define FRUIT_SRC_MODULE_ARM_ARM_H_

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
// #include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
// #include <string>
#include <vector>

namespace fruit {
namespace arm {

typedef enum {
  DATA_BUFF_LENGTH = 13,
  TIME_BUFF_LENGTH = 3,
};

class ArmDevice {
  // int Arm_serial_servo_read(int id);

 public:
  static ArmDevice &getInstance() {
    static ArmDevice arm;
    return arm;
  }
  
  void armSerialServoWrite6(int s1, int s2, int s3, int s4, int s5, int s6,
                            int time);

 private:
  ArmDevice();
  ArmDevice(const ArmDevice &) = delete;
  ArmDevice(const ArmDevice &&) = delete;
  ArmDevice &operator=(const ArmDevice &) = delete;

 private:
  static ArmDevice *instance_;
  int addr_;
  int bus_;

 public:
  int const tight_ = 130;  // 夹紧时的6号舵机角度(需比读取到的角度略大3~5°)
  int loose_ = 50;                   // 松开使的6号舵机角度
  int const delta_yaw_servo_1_ = 0;  // 待命位调水平时的1号舵机偏差角度
  int const level_servo_2_ = 175;  // 待命位调水平时的2号舵机角度
  int const level_servo_3_ = 0;    // 待命位调水平时的3号舵机角度
  int const level_servo_4_ = 0;    // 待命位调水平时的4号舵机角度
  int const arm_left_ = 180;
  int const arm_middle_ = 90;
  int const arm_right_ = 0;
};

}  // namespace arm
}  // namespace fruit

#endif