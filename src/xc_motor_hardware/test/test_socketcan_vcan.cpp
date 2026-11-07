#include "xc_motor_hardware/socketcan_transport.hpp"

#include <iostream>

// 该测试只在 Ubuntu 已创建 vcan0 时手动运行：
//   ./test_socketcan_vcan vcan0
// 没有 vcan0 时返回 77，表示环境未提供集成测试条件，而不是模拟通过。
int main(int argc, char **argv)
{
  const std::string interface_name = argc > 1 ? argv[1] : "vcan0";
  xc_motor_hardware::SocketCanTransport transport;
  std::string error;
  if (!transport.open(interface_name, 1000000, &error)) {
    std::cerr << "vcan unavailable: " << error << '\n';
    return 77;
  }
  std::cout << "SocketCAN opened on " << interface_name << '\n';
  return 0;
}
