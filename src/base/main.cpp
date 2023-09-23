#pragma once
#include <algorithm>
#include <numeric>

#include "/home/ling/fruit_cpp/include/module/motion/motion.h"
#include "/home/ling/fruit_cpp/include/module/socket/socket.h"
#include "include/module/ball/ball.h"
#include "include/module/coor2claw/coor2claw.h"
#include "include/module/qr/qr.h"
#include "include/module/turn/turn.h"
#include "include/module/uart/uart.h"
char zero = '0';
char one = '1';
char two = '2';
char three = '3';
char four = '4';
char five = '5';
char six = '6';
char seven = '7';
char eight = '8';
char turn_left = 'a';
char turn_right = 'd';
char drive_ahead = 'w';
char drive_back = 's';
//TODO 增加一个抓取类，调用 balldetect类，实现抓取
uart::SerialPort serial = uart::SerialPort("/dev/CH340", 115200);
turn::TurnDetector turn_detector =turn::TurnDetector("config/param.xml", 9, 10, false);
qr::QRDtector qr_detector = qr::QRDtector(10, false);
coor2claw::Coor2Claw coor_2_claw = coor2claw::Coor2Claw("config/param.xml");
ball::BallDetector ball_detector =ball::BallDetector("config/param.xml", 9, 10, false);
socket::Socket soc = socket::Socket();

char go_grab_retreat(int cam, std::vector<std::vector<int>> fruit_tree_basket,
                     int grab_time_out, bool go, bool retreat, int num);

void go_detect_seq(int cam);
int main() {
  int cam = 0;
  int chassis_reset = serial.serial_communicate(zero);  // 底盘重置

  int traffic_position_get = serial.serial_communicate(one);  // 前往1号交叉点
  if (traffic_position_get) {
    arm_scout(arm_middle);                      // 机械臂抬起
    int turn = turn_detector.detect_turn(cam);  // 读转向标识
    arm_standby(arm_middle);                    // 机械臂待命位
    char direction;
    switch (turn) {
      case 1:
        direction = turn_left;
        break;
      case 2:
        direction = drive_ahead;
        break;
      case 3:
        direction = turn_right;
        break;
      default:
        direction = drive_ahead;
        break;
    }
    serial.serial_communicate(direction);
  }

  int qr_position_get = serial.serial_communicate(two);  // 前往2号交叉点
  std::vector<char> fruit_seq(4);  // 初始化解码结果
  if (qr_position_get) {
    arm_scout(arm_middle);  // 机械臂抬起
    qr_detector.QR_detect(cam);
    // 读二维码
    arm_standby(arm_middle);  // 机械臂待命位
  }
  // 创建3行4列空表，第1行存采摘区果号，第2行存树号，第3行存放置区果号
  std::vector<std::vector<int>> fruit_tree_basket(3, std::vector<int>(4, 0));
  fruit_tree_basket[1] = {1, 2, 3, 4};    // 初始化树号
  if (qr_detector.ReturnResult().flag) {  // 解到二维码
                                          // 解码结果记录到第1行
    for (int i = 0; i < 4; i++) {
      fruit_tree_basket[0][i] = (int)fruit_seq[i];
    }
    // 按果号(第1行)倒序排列整个二维向量，要保持果号和树号的对应关系不变
    // 先创建一个存放索引的向量
    std::vector<int> idx(4);
    // 用iota函数给索引赋值为0, 1, 2, 3
    std::iota(idx.begin(), idx.end(), 0);
    // 用sort函数对索引按照第1行的倒序进行排序
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
      return fruit_tree_basket[0][a] > fruit_tree_basket[0][b];
    });
    // 用索引对二维向量进行切片
    std::vector<std::vector<int>> temp(3, std::vector<int>(4));
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        temp[j][i] = fruit_tree_basket[j][idx[i]];
      }
    }
    fruit_tree_basket = temp;

    char a = go_grab_retreat(cam, fruit_tree_basket, 15, true, true, 4);

    go_detect_seq(cam);
    int park = serial.serial_communicate(eight);
  } else {
    int park = serial.serial_communicate(eight);
  }
  return 0;
}

char go_grab_retreat(int cam, std::vector<std::vector<int>> fruit_tree_basket,
                     int grab_time_out = 15, bool go = true,
                     bool retreat = true, int num = 4) {
  if (go) {
    int ret = serial.serial_communicate(four);
  }
  char nowpos = '4';
  for (int i = 0; i < num; i++) {
    int tree = fruit_tree_basket[1][i];
    // 先判断所在位置与目标点之间的关系
    if (nowpos == '4') {
      if (tree > 2) {
        // 向前走
        int ret = serial.serial_communicate(drive_ahead);
      }
    } else {
      if (tree < 3) {
        // 往后退
        int ret = serial.serial_communicate(drive_back);
      }
    }
    if (tree & 1) {
      int ret = serial.serial_communicate(turn_left);
    } else {
      int ret = serial.serial_communicate(turn_right);
    }
    // 前往停止线
    int fruit = fruit_tree_basket[0][i];
    int ret = serial.serial_communicate(drive_ahead);
    // 抓黄球
    double t0 = cv::getTickCount();
    while ((cv::getTickCount() - t0) / cv::getTickFrequency() < grab_time_out) {
      arm_standby(arm_middle);
      ball::circles_result result;
      result = ball_detector.detect_circles(cam, 1, 2);
      if (result.circles[0][2] == 0) {  // 地一个半径是零，代表都是零
        std::cout << "\nNO circles!" << std::endl;
        continue;
      }
      for (int i = 0; i < 2; i++) {
        std::cout << "current circle:" << std::endl;
        for (int j = 0; j < 3; j++) {
          std::cout << result.circles[i][j] << "\t";
        }
        coor_2_claw.ArmCoord2Claw(
            cv::Point2d(result.circles[i][0], result.circles[i][1]));
        std::vector<double> coord = coor_2_claw.ReturnCoord();
        soc.socket_communicate(coord);
        std::vector<double> data_ik = soc.Return_data_it();
        arm_grab(data_ik);
        switch (fruit) {
          case 1:
            put_in_box1();
            break;
          case 2:
            put_in_box2();
            break;
          case 3:
            put_in_box3();
            break;
          case 4:
            put_in_box4();
            break;
          default:
            put_in_box1();
            break;
        }
      }
      // 抓完一颗数完毕退回主路
      if (retreat) {
        int ret = serial.serial_communicate(drive_back);
        switch (tree) {
          case 1:
            int ret = serial.serial_communicate(turn_right);
            nowpos = '4';
            break;
          case 2:
            int ret = serial.serial_communicate(turn_left);
            nowpos = '4';
            break;
          case 3:
            int ret = serial.serial_communicate(turn_right);
            nowpos = '5';
            break;
          case 4:
            int ret = serial.serial_communicate(turn_left);
            nowpos = '5';
            break;
          default:
            int ret = serial.serial_communicate(turn_right);
            nowpos = '4';
            break;
        }
      }
    }
  }
  return nowpos;
}

void go_detect_seq(int cam) {
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      int ret = serial.serial_communicate(six);
    } else if (i == 2) {
      int ret = serial.serial_communicate(seven);
    }
    int direction;
    if (i & 1) {  // 1 3    2 4号
      direction = arm_right;
      arm_standby(direction);
    } else {
      direction = arm_left;
      arm_standby(direction);
    }

    // 种类识别
    double t0 = cv::getTickCount();
    int fruit_type = 0;
    while (fruit_type) {
      ball::type_result result = ball_detector.detect_type(cam);
      if ((cv::getTickCount() - t0) > 10) {
        break;
      }
      switch (result.fruit_type) {
        case 1:
          box_in_basket_1(direction);
          break;
        case 2:
          box_in_basket_2(direction);
          break;
        case 3:
          box_in_basket_3(direction);
          break;
        case 4:
          box_in_basket_4(direction);
          break;
        default:
          break;
      }
    }
  }
}
