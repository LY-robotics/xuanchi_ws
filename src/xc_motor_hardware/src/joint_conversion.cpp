#include "xc_motor_hardware/joint_conversion.hpp"
#include <cmath>
#include <limits>

namespace xc_motor_hardware {
bool JointConversion::joint_to_count(double q, const MotorConfig &c, std::int32_t n, std::int32_t *out, std::string *e) {
  if (!out || n <= 0 || c.gear_ratio <= 0.0 || (c.direction != 1 && c.direction != -1) || !finite(q) || !finite(c.joint_offset_rad)) { if(e)*e="无效的角度、计数分辨率、减速比或方向"; return false; }
  const double ratio = c.position_count_reference == PositionCountReference::MotorShaft ? c.gear_ratio : 1.0;
  const double raw = static_cast<double>(c.encoder_zero_count) + static_cast<double>(c.direction) * (q - c.joint_offset_rad) * ratio * static_cast<double>(n) / kTwoPi;
  if (!finite(raw) || raw < static_cast<double>(std::numeric_limits<std::int32_t>::min()) || raw > static_cast<double>(std::numeric_limits<std::int32_t>::max())) { if(e)*e="目标 Count 溢出 int32"; return false; }
  *out = static_cast<std::int32_t>(std::llround(raw)); return true;
}
bool JointConversion::count_to_joint(std::int32_t count, const MotorConfig &c, std::int32_t n, double *out, std::string *e) {
  if (!out || n <= 0 || c.gear_ratio <= 0.0 || (c.direction != 1 && c.direction != -1) || !finite(c.joint_offset_rad)) { if(e)*e="无效的反馈转换参数"; return false; }
  const double ratio = c.position_count_reference == PositionCountReference::MotorShaft ? c.gear_ratio : 1.0;
  *out = static_cast<double>(c.direction) * (static_cast<double>(count) - c.encoder_zero_count) * kTwoPi / (static_cast<double>(n) * ratio) + c.joint_offset_rad;
  return finite(*out);
}
double JointConversion::rpm_to_joint_velocity(double motor_rpm, const MotorConfig &c) {
  if (!finite(motor_rpm) || c.gear_ratio <= 0.0 || (c.direction != 1 && c.direction != -1)) return std::numeric_limits<double>::quiet_NaN();
  const double ratio = c.position_count_reference == PositionCountReference::MotorShaft ? c.gear_ratio : 1.0;
  return static_cast<double>(c.direction) * motor_rpm * kTwoPi / 60.0 / ratio;
}
}  // namespace xc_motor_hardware
