#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <limits>

namespace xc_motor_hardware {

enum class PositionCountReference { MotorShaft, JointOutput };
enum class DeactivateAction { None, DisableOutput };

struct MotorConfig {
  std::string joint_name;
  std::uint8_t motor_id{0};
  std::string can_interface{"can0"};
  double gear_ratio{1.0};
  int direction{1};
  double joint_offset_rad{0.0};
  std::int32_t encoder_zero_count{0};
  double min_position_rad{-std::numeric_limits<double>::infinity()};
  double max_position_rad{std::numeric_limits<double>::infinity()};
  PositionCountReference position_count_reference{PositionCountReference::MotorShaft};

  bool validate(std::string *error = nullptr) const;
};

struct HardwareConfig {
  std::string protocol{"sanpo_custom_v3"};
  std::string can_interface{"can0"};
  int bitrate{1000000};
  std::int32_t encoder_counts_per_rev{16384};
  PositionCountReference position_count_reference{PositionCountReference::MotorShaft};
  double feedback_rate_hz{20.0};
  int state_timeout_ms{250};
  int response_timeout_ms{100};
  double command_rate_limit_hz{0.0};
  DeactivateAction deactivate_action{DeactivateAction::None};
  std::vector<MotorConfig> joints;

  bool validate(const std::vector<std::string> &hardware_joint_names, std::string *error = nullptr) const;
};

std::optional<HardwareConfig> load_hardware_config(const std::string &path, std::string *error = nullptr);

}  // namespace xc_motor_hardware
