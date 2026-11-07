#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace xc_motor_hardware {

struct CanFrame {
  std::uint32_t id{0};
  std::uint8_t dlc{0};
  std::uint8_t data[8]{};
  bool extended{false};
};

struct A2Response { std::int32_t speed_centi_rpm{0}; };
struct A3Response { std::uint16_t single_turn_count{0}; std::int32_t multi_turn_count{0}; };
struct A4Response {
  std::uint8_t temperature_c{0};
  std::int16_t q_axis_current{0};
  std::int16_t speed_centi_rpm{0};
  std::uint16_t single_turn_count{0};
};
struct AeResponse {
  std::uint16_t bus_voltage{0};
  std::uint16_t bus_current{0};
  std::uint8_t temperature_c{0};
  std::uint8_t mode{0};
  std::uint8_t fault_code{0};
};
struct B0Response {
  std::uint8_t pole_pairs{0};
  float torque_constant{0.0F};
  std::uint8_t protocol_gear_ratio{0};
};

class MotorProtocol {
public:
  static constexpr std::uint32_t kCommandBaseId = 0x100;
  static constexpr std::int32_t kMinMotorId = 1;
  static constexpr std::int32_t kMaxMotorId = 254;
  static CanFrame make_request(std::uint8_t motor_id, std::uint8_t command);
  static CanFrame make_c2_position_command(std::uint8_t motor_id, std::int32_t target_count);
  static CanFrame make_cf_disable_output(std::uint8_t motor_id);
  static std::optional<A2Response> parse_a2(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  static std::optional<A3Response> parse_a3(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  // C2 位置控制的应答数据布局与 A3 相同，但 Data[0] 为 C2。
  static std::optional<A3Response> parse_position_feedback(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  static std::optional<A4Response> parse_a4(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  static std::optional<AeResponse> parse_ae(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  static std::optional<B0Response> parse_b0(const CanFrame &frame, std::uint8_t motor_id, std::string *error = nullptr);
  static std::vector<std::string> decode_faults(std::uint8_t fault_code);
  static bool valid_motor_id(std::uint32_t id);

  // 判断是否为本驱动接受的经典 CAN 标准帧。该检查供传输层和测试共同使用。
  static bool valid_standard_frame(const CanFrame &frame);

private:
  static bool validate_response(const CanFrame &frame, std::uint8_t motor_id, std::uint8_t command, std::uint8_t dlc, std::string *error);
};

}  // namespace xc_motor_hardware
