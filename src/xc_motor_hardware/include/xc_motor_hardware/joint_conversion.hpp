#pragma once
#include "motor_config.hpp"
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

namespace xc_motor_hardware {

class JointConversion {
public:
  static constexpr double kTwoPi = 6.28318530717958647692;
  static bool finite(double value) { return std::isfinite(value); }
  static bool joint_to_count(double joint_rad, const MotorConfig &config, std::int32_t counts_per_rev, std::int32_t *count, std::string *error = nullptr);
  static bool count_to_joint(std::int32_t count, const MotorConfig &config, std::int32_t counts_per_rev, double *joint_rad, std::string *error = nullptr);
  static double rpm_to_joint_velocity(double motor_rpm, const MotorConfig &config);
};

}  // namespace xc_motor_hardware
