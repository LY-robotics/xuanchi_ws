#pragma once
#include "motor_protocol.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xc_motor_hardware {

class SocketCanTransport {
public:
  SocketCanTransport();
  ~SocketCanTransport();
  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;
  bool open(const std::string &interface_name, int bitrate, std::string *error = nullptr);
  void close();
  bool is_open() const;
  bool send(const CanFrame &frame, std::string *error = nullptr);
  bool receive(CanFrame *frame, int timeout_ms = 0, std::string *error = nullptr);
  bool set_motor_filter(const std::vector<std::uint8_t> &motor_ids, std::string *error = nullptr);

private:
  int fd_{-1};
  std::string interface_name_;
};

}  // namespace xc_motor_hardware


