#include "xc_motor_hardware/sanpo_can_system.hpp"
#include "xc_motor_hardware/joint_conversion.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <string>

namespace xc_motor_hardware {
namespace {
std::string hardware_parameter(const hardware_interface::HardwareInfo &info, const std::string &key)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? std::string{} : it->second;
}
}  // namespace

SanpoCanSystem::~SanpoCanSystem()//对象销毁时：先停 CAN 接收线程，关闭 SocketCAN 通道，防止资源泄漏。
{
  stop_receiver();
  if (transport_) transport_->close();
}

/*
统一报错封装：打印 ERROR 日志，返回 `ERROR` 给 hardware
 lifecycle 状态机（on_init /on_configure 失败返回这个）。
*/ 
hardware_interface::CallbackReturn SanpoCanSystem::fail(const std::string &message)
{
  RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "%s", message.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn SanpoCanSystem::on_init(
  const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = info;
  std::vector<std::string> joint_names;
  joint_names.reserve(info.joints.size());
  for (const auto &joint : info.joints) {
    joint_names.push_back(joint.name);
    bool position_command = false, position_state = false, velocity_state = false;
    for (const auto &interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) position_command = true;
    }
    for (const auto &interface : joint.state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) position_state = true;
      if (interface.name == hardware_interface::HW_IF_VELOCITY) velocity_state = true;
    }
    if (!position_command || !position_state || !velocity_state) {
      return fail("每个关节必须提供 position 命令以及 position/velocity 状态接口: " + joint.name);
    }
  }

  const auto config_path = hardware_parameter(info, "config_file");
  if (config_path.empty()) return fail("hardware 参数缺少 config_file");
  std::string error;
  const auto loaded = load_hardware_config(config_path, &error);
  if (!loaded) return fail(error);
  config_ = *loaded;
  if (!config_.validate(joint_names, &error)) return fail(error);

  states_.assign(config_.joints.size(), MotorState{});
  commands_.assign(config_.joints.size(), 0.0);
  initialized_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SanpoCanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(info_.joints.size() * 2);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                        &states_[i].position_rad);
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                        &states_[i].velocity_rad_s);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> SanpoCanSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return result;
}

hardware_interface::CallbackReturn SanpoCanSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!initialized_) return fail("尚未完成 on_init");
  transport_ = std::make_unique<SocketCanTransport>();
  std::string error;
  if (!transport_->open(config_.can_interface, config_.bitrate, &error)) return fail(error);

  std::vector<std::uint8_t> ids;
  ids.reserve(config_.joints.size());
  for (const auto &joint : config_.joints) ids.push_back(joint.motor_id);
  if (!transport_->set_motor_filter(ids, &error)) return fail(error);

  receiver_running_ = true;
  receiver_thread_ = std::thread(&SanpoCanSystem::receiver_loop, this);
  // B0 只用于记录驱动板诊断参数，不会覆盖 YAML 中的浮点机械减速比。
  for (const auto &joint : config_.joints) {
    if (!transport_->send(MotorProtocol::make_request(joint.motor_id, 0xB0), &error)) {
      RCLCPP_WARN(rclcpp::get_logger("SanpoCanSystem"), "发送 B0 诊断请求失败 (%s): %s",
                  joint.joint_name.c_str(), error.c_str());
    }
  }
  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SanpoCanSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_) return fail("尚未完成 on_configure");
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto &state : states_) {
      state.valid_position = false;
      state.valid_velocity = false;
      state.faulted = false;
      state.fault_code = 0;
    }
  }
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (!request_position(i)) return fail("发送初始位置请求失败: " + config_.joints[i].joint_name);
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(config_.response_timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    bool all_valid = true;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const auto &state : states_) all_valid = all_valid && state.valid_position;
      if (all_valid) {
        for (std::size_t i = 0; i < commands_.size(); ++i) commands_[i] = states_[i].position_rad;
      }
    }
    if (all_valid) {
      active_ = true;
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return fail("激活超时：没有获得所有电机的有效 A3 初始位置");
}

hardware_interface::CallbackReturn SanpoCanSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (config_.deactivate_action == DeactivateAction::DisableOutput && transport_) {
    for (const auto &joint : config_.joints) {
      std::string error;
      if (!transport_->send(MotorProtocol::make_cf_disable_output(joint.motor_id), &error)) {
        RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "发送 CF 失败 (%s): %s",
                     joint.joint_name.c_str(), error.c_str());
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SanpoCanSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_receiver();
  if (transport_) transport_->close();
  transport_.reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    states_.clear();
    commands_.clear();
  }
  active_ = false;
  configured_ = false;
  initialized_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SanpoCanSystem::on_shutdown(
  const rclcpp_lifecycle::State &previous_state)
{
  return on_cleanup(previous_state);
}

bool SanpoCanSystem::request_position(std::size_t index)
{
  if (!transport_ || !transport_->is_open()) return false;
  std::string error;
  if (!transport_->send(MotorProtocol::make_request(config_.joints[index].motor_id, 0xA3), &error)) {
    RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "发送 A3 失败 (%s): %s",
                 config_.joints[index].joint_name.c_str(), error.c_str());
    return false;
  }
  return true;
}

void SanpoCanSystem::receiver_loop()
{
  while (receiver_running_) {
    CanFrame frame{};
    if (!transport_ || !transport_->receive(&frame, 20)) continue;
    for (std::size_t i = 0; i < config_.joints.size(); ++i) {
      const auto &config = config_.joints[i];
      if (frame.id != config.motor_id) continue;
      std::string error;
      if (frame.data[0] == 0xA3 || frame.data[0] == 0xC2) {
        const auto parsed = MotorProtocol::parse_position_feedback(frame, config.motor_id, &error);
        if (parsed) {
          double position = 0.0;
          if (JointConversion::count_to_joint(parsed->multi_turn_count, config,
                                              config_.encoder_counts_per_rev, &position, &error)) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            states_[i].actual_count = parsed->multi_turn_count;
            states_[i].position_rad = position;
            states_[i].valid_position = true;
            states_[i].last_position_update = std::chrono::steady_clock::now();
            states_[i].last_update = states_[i].last_position_update;
          }
        }
      } else if (frame.data[0] == 0xA2) {
        const auto parsed = MotorProtocol::parse_a2(frame, config.motor_id, &error);
        if (parsed) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          states_[i].velocity_rad_s = JointConversion::rpm_to_joint_velocity(
            parsed->speed_centi_rpm / 100.0, config);
          states_[i].valid_velocity = true;
          states_[i].last_velocity_update = std::chrono::steady_clock::now();
        }
      } else if (frame.data[0] == 0xAE) {
        const auto parsed = MotorProtocol::parse_ae(frame, config.motor_id, &error);
        if (parsed) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          states_[i].fault_code = parsed->fault_code;
          states_[i].faulted = parsed->fault_code != 0;
          if (states_[i].faulted) {
            RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"),
                         "关节 %s 报告故障码 0x%02X", config.joint_name.c_str(), parsed->fault_code);
          }
        }
      } else if (frame.data[0] == 0xB0) {
        const auto parsed = MotorProtocol::parse_b0(frame, config.motor_id, &error);
        if (parsed) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          states_[i].pole_pairs = parsed->pole_pairs;
          states_[i].torque_constant = parsed->torque_constant;
          states_[i].protocol_gear_ratio = parsed->protocol_gear_ratio;
        }
      }
    }
  }
}

hardware_interface::return_type SanpoCanSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return hardware_interface::return_type::ERROR;
  const auto now = std::chrono::steady_clock::now();
  if (now - last_poll_ >= std::chrono::duration<double>(1.0 / config_.feedback_rate_hz)) {
    last_poll_ = now;
    for (const auto &joint : config_.joints) {
      if (!transport_) continue;
      std::string error;
      if (!transport_->send(MotorProtocol::make_request(joint.motor_id, 0xA3), &error)) {
        RCLCPP_WARN(rclcpp::get_logger("SanpoCanSystem"), "发送 A3 失败 (%s): %s",
                    joint.joint_name.c_str(), error.c_str());
      }
      if (!transport_->send(MotorProtocol::make_request(joint.motor_id, 0xA2), &error)) {
        RCLCPP_WARN(rclcpp::get_logger("SanpoCanSystem"), "发送 A2 失败 (%s): %s",
                    joint.joint_name.c_str(), error.c_str());
      }
    }
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto &state : states_) {
    if (!state.valid_position ||
        now - state.last_position_update > std::chrono::milliseconds(config_.state_timeout_ms) ||
        state.faulted) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SanpoCanSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !transport_) return hardware_interface::return_type::ERROR;
  const auto now = std::chrono::steady_clock::now();
  if (config_.command_rate_limit_hz > 0.0 &&
      now - last_command_send_ < std::chrono::duration<double>(1.0 / config_.command_rate_limit_hz)) {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto &state : states_) {
    if (state.faulted) {
      RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "检测到电机故障，拒绝继续发送运动命令");
      return hardware_interface::return_type::ERROR;
    }
  }
  for (std::size_t i = 0; i < commands_.size(); ++i) {
    if (!std::isfinite(commands_[i])) return hardware_interface::return_type::ERROR;
    if (commands_[i] < config_.joints[i].min_position_rad || commands_[i] > config_.joints[i].max_position_rad) {
      RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "关节 %s 的目标角度超出配置范围",
                   config_.joints[i].joint_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    std::int32_t count = 0;
    std::string error;
    if (!JointConversion::joint_to_count(commands_[i], config_.joints[i],
                                         config_.encoder_counts_per_rev, &count, &error)) {
      RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "%s", error.c_str());
      return hardware_interface::return_type::ERROR;
    }
    if (!transport_->send(MotorProtocol::make_c2_position_command(config_.joints[i].motor_id, count), &error)) {
      RCLCPP_ERROR(rclcpp::get_logger("SanpoCanSystem"), "发送 C2 失败 (%s): %s",
                   config_.joints[i].joint_name.c_str(), error.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  last_command_send_ = now;
  return hardware_interface::return_type::OK;
}

void SanpoCanSystem::stop_receiver()
{
  receiver_running_ = false;
  if (receiver_thread_.joinable()) receiver_thread_.join();
}

}  // namespace xc_motor_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(xc_motor_hardware::SanpoCanSystem, hardware_interface::SystemInterface)

