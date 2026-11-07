#include "xc_motor_hardware/motor_protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace xc_motor_hardware {
namespace {
std::uint16_t u16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8); }
std::int16_t i16(const std::uint8_t *p) { const auto v=u16(p); return v <= 0x7FFFu ? static_cast<std::int16_t>(v) : static_cast<std::int16_t>(static_cast<std::int32_t>(v)-0x10000); }
std::uint32_t u32(const std::uint8_t *p) { return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24); }
std::int32_t i32(const std::uint8_t *p) { const auto v=u32(p); return v <= 0x7FFFFFFFu ? static_cast<std::int32_t>(v) : static_cast<std::int32_t>(static_cast<std::int64_t>(v)-0x100000000LL); }
float f32(const std::uint8_t *p) { const auto bits=u32(p); float value=0.0F; std::memcpy(&value,&bits,sizeof(value)); return value; }
void put_i32(std::uint8_t *p, std::int32_t value) { const auto v = static_cast<std::uint32_t>(value); p[0]=v&0xff; p[1]=(v>>8)&0xff; p[2]=(v>>16)&0xff; p[3]=(v>>24)&0xff; }
}

bool MotorProtocol::valid_motor_id(std::uint32_t id) { return id >= 1 && id <= 254; }
bool MotorProtocol::valid_standard_frame(const CanFrame &frame) {
  return !frame.extended && frame.id > 0u && frame.id <= 0x7FFu && frame.dlc >= 1u && frame.dlc <= 8u;
}
CanFrame MotorProtocol::make_request(std::uint8_t motor_id, std::uint8_t command) {
  CanFrame frame;
  // 0x00 是广播地址，0xFF 是公共地址，不能被当作单电机地址使用。
  if (!valid_motor_id(motor_id)) return frame;
  frame.id = kCommandBaseId | motor_id; frame.dlc = 1; frame.data[0] = command; return frame;
}
CanFrame MotorProtocol::make_c2_position_command(std::uint8_t motor_id, std::int32_t target_count) {
  CanFrame frame = make_request(motor_id, 0xC2); if (frame.dlc == 0) return frame;
  frame.dlc = 5; put_i32(&frame.data[1], target_count); return frame;
}
CanFrame MotorProtocol::make_cf_disable_output(std::uint8_t motor_id) { return make_request(motor_id, 0xCF); }

bool MotorProtocol::validate_response(const CanFrame &frame, std::uint8_t motor_id, std::uint8_t command, std::uint8_t dlc, std::string *error) {
  if (!valid_motor_id(motor_id)) { if (error) *error = "motor_id 必须在 1..254"; return false; }
  if (!valid_standard_frame(frame)) { if (error) *error = "响应不是合法的经典 CAN 标准帧"; return false; }
  if (frame.id != motor_id) { if (error) *error = "响应 CAN ID 与 motor_id 不匹配"; return false; }
  if (frame.dlc != dlc) { if (error) *error = "响应 DLC 不符合协议"; return false; }
  if (frame.data[0] != command) { if (error) *error = "响应命令字不匹配"; return false; }
  return true;
}
std::optional<A2Response> MotorProtocol::parse_a2(const CanFrame &f, std::uint8_t id, std::string *e) { if (!validate_response(f,id,0xA2,5,e)) return std::nullopt; return A2Response{i32(&f.data[1])}; }
std::optional<A3Response> MotorProtocol::parse_a3(const CanFrame &f, std::uint8_t id, std::string *e) { if (!validate_response(f,id,0xA3,7,e)) return std::nullopt; return A3Response{u16(&f.data[1]),i32(&f.data[3])}; }
std::optional<A3Response> MotorProtocol::parse_position_feedback(const CanFrame &f, std::uint8_t id, std::string *e) {
  if (!valid_motor_id(id) || !valid_standard_frame(f) || f.id != id || f.dlc != 7 || (f.data[0] != 0xA3 && f.data[0] != 0xC2)) {
    if (e) *e = "位置反馈必须是 A3 或 C2 命令字且 DLC=7";
    return std::nullopt;
  }
  return A3Response{u16(&f.data[1]),i32(&f.data[3])};
}
std::optional<A4Response> MotorProtocol::parse_a4(const CanFrame &f, std::uint8_t id, std::string *e) {
  // A4：温度 uint8、Q 轴电流 int16、速度 int16、单圈角度 uint16。
  if (!validate_response(f,id,0xA4,8,e)) return std::nullopt;
  return A4Response{f.data[1], i16(&f.data[2]), i16(&f.data[4]), u16(&f.data[6])};
}
std::optional<AeResponse> MotorProtocol::parse_ae(const CanFrame &f, std::uint8_t id, std::string *e) {
  // AE：母线电压 uint16、母线电流 uint16、温度 uint8、模式 uint8、故障码 uint8。
  if (!validate_response(f,id,0xAE,8,e)) return std::nullopt;
  return AeResponse{u16(&f.data[1]),u16(&f.data[3]),f.data[5],f.data[6],f.data[7]};
}
std::optional<B0Response> MotorProtocol::parse_b0(const CanFrame &f, std::uint8_t id, std::string *e) {
  if (!validate_response(f,id,0xB0,7,e)) return std::nullopt;
  const float torque_constant=f32(&f.data[2]);
  if (!std::isfinite(torque_constant)) { if (e) *e="B0 力矩常数不是有限单精度浮点数"; return std::nullopt; }
  return B0Response{f.data[1],torque_constant,f.data[6]};
}
std::vector<std::string> MotorProtocol::decode_faults(std::uint8_t code) {
  std::vector<std::string> faults;
  if (code & (1u<<0)) faults.emplace_back("电压故障");
  if (code & (1u<<1)) faults.emplace_back("电流故障");
  if (code & (1u<<2)) faults.emplace_back("温度故障");
  if (code & (1u<<3)) faults.emplace_back("编码器故障");
  if (code & (1u<<6)) faults.emplace_back("硬件故障");
  if (code & (1u<<7)) faults.emplace_back("软件故障");
  return faults;
}
}  // namespace xc_motor_hardware
