#include "src/module/socket/socket.h"

namespace fruit {

namespace mysocket {
Socket::Socket() {}
Socket::~Socket() {}

std::vector<double> Socket::socketCommunicate(const std::vector<double>& data) {
  // 服务器套接字的路径
  const char* ADDR = "/home/jetson/dofbot_ws/src/dofbot_moveit/src/server.sock";
  // 创建客户端套接字
  int clientfd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (clientfd == -1) {
    std::cout << "\033[31m Failed to create socket \033[0m \n";
    return {};
  }
  // 连接到服务器套接字
  sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  strcpy(server_addr.sun_path, ADDR);
  if (connect(clientfd, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
    std::cout << "\033[31m Failed to connect to server \033[0m \n";
    return {};
  }
  // 发送数据
  std::string msg_send;
  for (double d : data) {
    msg_send += std::to_string(d) + " ";
  }
  std::cout << "sending object pos data : \n" << msg_send << "\n";
  if (write(clientfd, msg_send.c_str(), msg_send.size()) == -1) {
    std::cout << "\033[31m Failed to write data \033[0m \n";
    return {};
  }
  // 接收数据
  char buffer[1024];
  int n = read(clientfd, buffer, sizeof(buffer));
  if (n == -1) {
    std::cout << "\033[31m Failed to read data \033[0m \n";
    return {};
  }
  buffer[n] = '\0';
  std::string msg_recv(buffer);
  std::cout << " servo data received: \n" << msg_recv << "\n";
  // 解析数据
  std::vector<double> data_ik;
  size_t pos = 0;
  while ((pos = msg_recv.find(" ")) != std::string::npos) {
    data_ik.push_back(std::stod(msg_recv.substr(0, pos)));
    msg_recv.erase(0, pos + 1);
  }
  // 关闭客户端套接字
  close(clientfd);
  return data_ik;
}

}  // namespace mysocket
}  // namespace fruit