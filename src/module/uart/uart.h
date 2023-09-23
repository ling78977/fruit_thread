#ifndef FRUIT_SRC_MODULE_UART_UART_H_
#define FRUIT_SRC_MODULE_UART_UART_H_
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>
// #include <string>

namespace fruit {
namespace uart {
class SerialPort {
 private:
  // 串口名称
  std::string port_name_;
  unsigned char write_buff_[1];
  unsigned char read_buff_[1];
  // 波特率
  int baud_rate_;
  int edit_;
  // 文件描述符
  int fd_;
  ssize_t Write(unsigned char _data);
  unsigned char Read();

  ssize_t read_messge_;
  ssize_t write_messge_;

 public:
  SerialPort(const std::string uart_config_);
  int serialCommunicate(unsigned char& _data_send);
  ~SerialPort();
};
}  // namespace uart
}  // namespace fruit

#endif