#ifndef SRC_MODULE_SOCKET_SOCKET_H_
#define SRC_MODULE_SOCKET_SOCKET_H_

#include <unistd.h>

#include <iostream>
// #include <sstream>
// #include <string>
#include <sys/socket.h>
#include <sys/un.h>

#include <vector>

namespace fruit {
namespace mysocket {

class Socket {
 private:
  std::vector<double> data_ik_;

 public:
  Socket();
  std::vector<double> socketCommunicate(const std::vector<double>& data);
  ~Socket();
};
}  // namespace mysocket
}  // namespace fruit
#endif