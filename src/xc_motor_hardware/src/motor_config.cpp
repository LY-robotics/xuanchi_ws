#include "xc_motor_hardware/motor_config.hpp"
#include "xc_motor_hardware/motor_protocol.hpp"
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <set>
#include <fstream>
#include <cmath>
#include <sstream>

namespace xc_motor_hardware {
namespace {
std::optional<PositionCountReference> parse_ref(const std::string &v) {
  if (v == "motor_shaft") return PositionCountReference::MotorShaft;
  if (v == "joint_output") return PositionCountReference::JointOutput;
  return std::nullopt;
}
std::optional<DeactivateAction> parse_deactivate(const std::string &v) {
  if (v == "none") return DeactivateAction::None;
  if (v == "disable_output") return DeactivateAction::DisableOutput;
  return std::nullopt;
}
template<class T> T get(const YAML::Node &n, const char *key, const T &fallback) { return n[key] ? n[key].as<T>() : fallback; }
}
bool MotorConfig::validate(std::string *e) const {
  if (joint_name.empty()) { if(e)*e="关节名不能为空"; return false; }
  if (can_interface.empty()) { if(e)*e="can_interface 不能为空"; return false; }
  if (!MotorProtocol::valid_motor_id(motor_id)) { if(e)*e="motor_id 必须在 1..254，不能使用广播/公共地址"; return false; }
  if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) { if(e)*e="gear_ratio 必须是正数"; return false; }
  if (direction != 1 && direction != -1) { if(e)*e="direction 必须为 +1 或 -1"; return false; }
  if (!std::isfinite(joint_offset_rad)) { if(e)*e="joint_offset_rad 必须是有限数"; return false; }
  if (std::isnan(min_position_rad) || std::isnan(max_position_rad) || min_position_rad > max_position_rad) { if(e)*e="位置上下限必须有效且 min_position_rad <= max_position_rad"; return false; }
  return true;
}
bool HardwareConfig::validate(const std::vector<std::string> &names, std::string *e) const {
  if (protocol != "sanpo_custom_v3") { if(e)*e="protocol 必须为 sanpo_custom_v3"; return false; }
  if (can_interface.empty()) { if(e)*e="can_interface 不能为空"; return false; }
  if (bitrate != 1000000 && bitrate != 500000 && bitrate != 250000 && bitrate != 125000 && bitrate != 100000) { if(e)*e="bitrate 必须是协议支持的 1000000、500000、250000、125000 或 100000"; return false; }
  if (encoder_counts_per_rev <= 0 || !std::isfinite(feedback_rate_hz) || feedback_rate_hz <= 0.0 || state_timeout_ms <= 0 || response_timeout_ms <= 0 || !std::isfinite(command_rate_limit_hz) || command_rate_limit_hz < 0.0) { if(e)*e="采样频率、超时和编码器分辨率必须有效且为正数（command_rate_limit_hz 可为 0）"; return false; }
  if (joints.empty()) { if(e)*e="至少需要配置一个关节"; return false; }
  if (joints.size() != names.size()) { if(e)*e="配置关节数量与 HardwareInfo 不一致"; return false; }
  std::set<std::string> seen_names; std::set<int> seen_ids;
  for (const auto &j : joints) { std::string x; if(!j.validate(&x)){if(e)*e=j.joint_name+": "+x;return false;} if(j.can_interface != can_interface){if(e)*e=j.joint_name+": 当前 SystemInterface 只支持同一 CAN 接口";return false;} if(!seen_names.insert(j.joint_name).second){if(e)*e="重复关节名";return false;} if(!seen_ids.insert(j.motor_id).second){if(e)*e="重复 motor_id";return false;} }
  for (const auto &n : names) if (!seen_names.count(n)) { if(e)*e="配置缺少 HardwareInfo 关节: "+n; return false; }
  return true;
}
std::optional<HardwareConfig> load_hardware_config(const std::string &path, std::string *e) {
  try {
    YAML::Node root = YAML::LoadFile(path); YAML::Node h = root["hardware"] ? root["hardware"] : root;
    if (!h || !h.IsMap()) throw std::runtime_error("顶层 hardware 必须是 YAML 映射");
    HardwareConfig c; c.protocol=get<std::string>(h,"protocol",c.protocol); c.can_interface=get<std::string>(h,"can_interface",c.can_interface); c.bitrate=get<int>(h,"bitrate",c.bitrate); c.encoder_counts_per_rev=get<int>(h,"encoder_counts_per_rev",c.encoder_counts_per_rev); auto ref=parse_ref(get<std::string>(h,"position_count_reference","motor_shaft")); if(!ref) throw std::runtime_error("position_count_reference 只能是 motor_shaft 或 joint_output"); c.position_count_reference=*ref; c.feedback_rate_hz=get<double>(h,"feedback_rate_hz",c.feedback_rate_hz); c.state_timeout_ms=get<int>(h,"state_timeout_ms",c.state_timeout_ms); c.response_timeout_ms=get<int>(h,"response_timeout_ms",c.response_timeout_ms); c.command_rate_limit_hz=get<double>(h,"command_rate_limit_hz",c.command_rate_limit_hz); auto action=parse_deactivate(get<std::string>(h,"deactivate_action","none")); if(!action) throw std::runtime_error("deactivate_action 只能是 none 或 disable_output"); c.deactivate_action=*action;
    if (!h["joints"] || !h["joints"].IsSequence()) throw std::runtime_error("hardware.joints 必须是 YAML 列表");
    for (const auto &n : h["joints"]) {
      if (!n.IsMap() || !n["name"] || !n["motor_id"] || !n["gear_ratio"]) throw std::runtime_error("每个关节必须提供 name、motor_id、gear_ratio");
      MotorConfig j; j.joint_name=n["name"].as<std::string>(); const int motor_id=n["motor_id"].as<int>(); if(motor_id<1 || motor_id>254) throw std::runtime_error(j.joint_name+": motor_id 必须在 1..254"); j.motor_id=static_cast<std::uint8_t>(motor_id); j.can_interface=get<std::string>(n,"can_interface",c.can_interface); j.gear_ratio=n["gear_ratio"].as<double>(); j.direction=get<int>(n,"direction",1); j.joint_offset_rad=get<double>(n,"joint_offset_rad",0.0); j.encoder_zero_count=get<int>(n,"encoder_zero_count",0); j.min_position_rad=get<double>(n,"min_position_rad",j.min_position_rad); j.max_position_rad=get<double>(n,"max_position_rad",j.max_position_rad); if (n["position_count_reference"]) { auto jr=parse_ref(n["position_count_reference"].as<std::string>()); if(!jr) throw std::runtime_error(j.joint_name+": position_count_reference 无效"); j.position_count_reference=*jr; } else j.position_count_reference=c.position_count_reference; c.joints.push_back(j);
    }
    return c;
  } catch (const std::exception &ex) { if(e)*e="读取配置文件失败: "+std::string(ex.what()); return std::nullopt; }
}
}  // namespace xc_motor_hardware


