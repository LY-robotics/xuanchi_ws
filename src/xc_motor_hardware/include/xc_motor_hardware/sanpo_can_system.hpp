#pragma once
#include "motor_config.hpp"
#include "motor_protocol.hpp"
#include "socketcan_transport.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace xc_motor_hardware {

struct MotorState {
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  std::int32_t actual_count{0};
  bool valid_position{false};
  bool valid_velocity{false};
  bool faulted{false};
  std::uint8_t fault_code{0};
  std::uint8_t pole_pairs{0};
  std::uint8_t protocol_gear_ratio{0};
  float torque_constant{0.0F};
  std::chrono::steady_clock::time_point last_update{};
  std::chrono::steady_clock::time_point last_position_update{};
  std::chrono::steady_clock::time_point last_velocity_update{};
};

class SanpoCanSystem final : public hardware_interface::SystemInterface {
public:
  ~SanpoCanSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void receiver_loop();
  bool request_position(std::size_t index);
  void stop_receiver();
  hardware_interface::CallbackReturn fail(const std::string &message);

  hardware_interface::HardwareInfo info_;
  HardwareConfig config_;
  std::unique_ptr<SocketCanTransport> transport_;
  std::vector<MotorState> states_;
  std::vector<double> commands_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_running_{false};
  mutable std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_poll_{};
  std::chrono::steady_clock::time_point last_command_send_{};
  bool initialized_{false};
  bool configured_{false};
  bool active_{false};
};

}  // namespace xc_motor_hardware
