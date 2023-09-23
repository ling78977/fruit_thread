
#include "src/module/arm/arm.h"
namespace fruit {

namespace arm {

ArmDevice::ArmDevice() {
  addr_ = 0x15;
  bus_ = open("/dev/i2c-1",
             O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);  // Open the i2c bus
  if (bus_ < 0) {
    std::cout << "\033[31m 打开 /dev/i2c-1 失败！\033[0m" << std::endl;
    return;
  }
}

void ArmDevice::armSerialServoWrite6(int s1, int s2, int s3, int s4, int s5,
                                     int s6, int time) {
  if (s1 > 180 || s2 > 180 || s3 > 180 || s4 > 180 || s5 > 270 || s6 > 180) {
    std::cout << "\033[31m Parameter range is not within 0-180! \033[0m"
              << std::endl;
    return;
  }
  // Set the i2c slave address
  if (ioctl(bus_, I2C_SLAVE, addr_) < 0) {
    std::cout << "\033[31m Set the i2c slave address 失败！\033[0m"
              << std::endl;
    exit(1);
  }
  try {
    // // Write some data to the device
    unsigned char buff_data[DATA_BUFF_LENGTH];
    unsigned char buff_time[TIME_BUFF_LENGTH];

    // Calculate the values for each servo
    int pos = (3100 - 900) * (s1 - 0) / (180 - 0) + 900;
    unsigned char value1_H = (pos >> 8) & 0xFF;
    unsigned char value1_L = pos & 0xFF;

    s2 = 180 - s2;
    pos = (3100 - 900) * (s2 - 0) / (180 - 0) + 900;
    unsigned char value2_H = (pos >> 8) & 0xFF;
    unsigned char value2_L = pos & 0xFF;

    s3 = 180 - s3;
    pos = (3100 - 900) * (s3 - 0) / (180 - 0) + 900;
    unsigned char value3_H = (pos >> 8) & 0xFF;
    unsigned char value3_L = pos & 0xFF;

    s4 = 180 - s4;
    pos = (3100 - 900) * (s4 - 0) / (180 - 0) + 900;
    unsigned char value4_H = (pos >> 8) & 0xFF;
    unsigned char value4_L = pos & 0xFF;

    pos = (3700 - 380) * (s5 - 0) / (270 - 0) + 380;
    unsigned char value5_H = (pos >> 8) & 0xFF;
    unsigned char value5_L = pos & 0xFF;

    pos = (3100 - 900) * (s6 - 0) / (180 - 0) + 900;
    unsigned char value6_H = (pos >> 8) & 0xFF;
    unsigned char value6_L = pos & 0xFF;

    // Write the time to register
    unsigned char time_H = (time >> 8) & 0xFF;
    unsigned char time_L = time & 0xFF;
    memset(buff_time, '0', sizeof(buff_time));
    buff_time[0] = 0x1e;
    buff_time[1] = time_H;
    buff_time[2] = time_L;
    // Write the values to register
    memset(buff_data, '0', sizeof(buff_data));
    buff_data[0] = 0x1d;
    buff_data[1] = value1_H;
    buff_data[2] = value1_L;
    buff_data[3] = value2_H;
    buff_data[4] = value2_L;
    buff_data[5] = value3_H;
    buff_data[6] = value3_L;
    buff_data[7] = value4_H;
    buff_data[8] = value4_L;
    buff_data[9] = value5_H;
    buff_data[10] = value5_L;
    buff_data[11] = value6_H;
    buff_data[12] = value6_L;

    write(bus_, buff_data, sizeof(buff_data));
    write(bus_, buff_time, sizeof(buff_time));
  } catch (const std::exception& e) {
    std::cout << "\033[31m 写入 i2c数据失败 \033[0m" << std::endl;
  }
}



}  // namespace arm
}  // namespace fruit
