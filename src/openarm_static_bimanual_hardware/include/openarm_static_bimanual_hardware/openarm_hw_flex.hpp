#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <mutex>
#include <algorithm>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "openarm_static_bimanual_hardware/visibility_control.h"

#include "canbus.hpp"
#include "motor.hpp"
#include "motor_control.hpp"

namespace openarm_static_bimanual_hardware {

class OPENARM_STATIC_BIMANUAL_HARDWARE_PUBLIC OpenArmHWFlex
    : public hardware_interface::SystemInterface {
public:
  OpenArmHWFlex();

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & prev_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & prev_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & prev_state) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ---- helpers ----
  static DM_Motor_Type str_to_motor_type(std::string s);  // 문자열 → enum
  static std::string trim(std::string s);

  // ---- per-arm runtime ----
  std::vector<std::unique_ptr<Motor>> motors_;

  std::vector<double> pos_commands_, pos_states_;
  std::vector<double> vel_commands_, vel_states_;
  std::vector<double> tau_ff_commands_, tau_states_;

  // ---- params from URDF ----
  std::vector<DM_Motor_Type> motor_types_;
  std::vector<uint16_t> can_device_ids_;
  std::vector<uint16_t> can_master_ids_;
  std::vector<double> kp_, kd_;
  std::string prefix_{};
  bool disable_torque_{false};
  bool can_fd_{false};

  // ---- shared resources across both arms ----
  static std::shared_ptr<CANBus> global_canbus_;
  static std::shared_ptr<MotorControl> global_motor_control_;
  static std::mutex can_mutex_;
};

}  // namespace openarm_static_bimanual_hardware
