#include "src/module/uart/uart.h"

namespace fruit {
namespace uart {
SerialPort::SerialPort(const std::string uart_config_) {
  cv::FileStorage uart(uart_config_, cv::FileStorage::READ);
  if (!uart.isOpened()) {
    std::cerr << "\033[31mopen uart files failed !\033[0m" << std::endl;
  }
  uart["PORT_NAME"] >> this->port_name_;
  uart["BAUD_RATE"] >> this->baud_rate_;
  uart["EDIT"] >> this->edit_;
  uart.release();
  struct termios newstate;
  bzero(&newstate, sizeof(newstate));
  fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    std::cerr << "\033[31m open serial failed:" << port_name_ << "\033[0m"
              << std::endl;
    exit(1);
  } else {
    std::cout << "\033[1m open serial sucess:" << port_name_ << "\033[0m"
              << std::endl;
  }
  switch (baud_rate_) {
    case 1:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    case 10:
      cfsetospeed(&newstate, B921600);
      cfsetispeed(&newstate, B921600);
      break;
    default:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
  }
  newstate.c_cflag |= CLOCAL | CREAD;
  newstate.c_cflag &= ~CSIZE;
  newstate.c_cflag &= ~CSTOPB;
  newstate.c_cflag |= CS8;
  newstate.c_cflag &= ~PARENB;
  newstate.c_cc[VTIME] = 0;
  newstate.c_cc[VMIN] = 0;
  tcflush(fd_, TCIOFLUSH);
  tcsetattr(fd_, TCSANOW, &newstate);
}

SerialPort::~SerialPort(void) {
  if (!close(fd_)) {
    std::cout << "\033[1mClose serial device success:" << fd_ << "\033[0m"
              << std::endl;
  }
};
ssize_t SerialPort::Write(unsigned char _data) {
  if (fd_ == -1) {  // 如果没有打开，抛出异常
    std::cout << "\033[31m Serial:" << port_name_ << " is not opened! \033[0m"
              << std::endl;
  }
  memset(write_buff_, '0', sizeof(write_buff_));
  write_buff_[0] = _data;
  write_messge_ = write(fd_, write_buff_, sizeof(write_buff_));
  if (write_messge_ == -1) {  // 如果写入失败，抛出异常
    std::cout << "\033[31m Failed to write data to serial port! \033[0m"
              << std::endl;
  }
  if (edit_ == 1) {
    printf("write_buff:%x\n", write_buff_[0]);
  }
  return write_messge_;  // 返回写入的字节数
}
unsigned char SerialPort::Read() {
  if (fd_ == -1) {  // 如果没有打开，抛出异常
    std::cout << "\033[31m Serial:" << port_name_ << " is not opened! \033[0m"
              << std::endl;
  }
  memset(read_buff_, '0', sizeof(read_buff_));
  read_messge_ = read(fd_, read_buff_, sizeof(read_buff_));
  if (read_messge_ == -1) {  // 如果读取失败，抛出异常
    std::cout << "\033[31m Failed to read data to serial port! \033[0m"
              << std::endl;
  }
  return read_buff_[0];
}

int SerialPort::serialCommunicate(unsigned char& _data_send) {
  std::cout << "get data to send: " << _data_send << std::endl;
  int ret = 0;
  if (fd_ != -1) {                  // 判断是否打开成功
    ssize_t a = Write(_data_send);  // 调用write()函数来发送指令
    if (a == 1) {
      std::cout << "sended position code: " << _data_send << std::endl;
    }
  }
  std::cout << "....等待接受数据...." << std::endl;
  while (true) {
    unsigned char get_data;  // 定义一个字符变量，用于存储读取的数据
    get_data = Read();
    // if (edit == 1) {
    //   printf("get_data:%x", get_data);
    // }
    if (get_data == 0x01) {  // 如果数据是退出标志
      ret = 1;
      break;
    }
  }
  std::cout << "Chassic on position" << std::endl;
  usleep(500000);  // 留出时间，防止下一指令意外
  return ret;
}

}  // namespace uart
}  // namespace fruit