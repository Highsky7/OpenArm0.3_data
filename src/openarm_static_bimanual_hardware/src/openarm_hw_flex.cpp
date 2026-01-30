#include "openarm_static_bimanual_hardware/openarm_hw_flex.hpp"

#include <thread>
#include <chrono>
#include <cctype>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_static_bimanual_hardware {

// 공유 리소스: CAN 버스, MotorControl 인스턴스, 뮤텍스
std::shared_ptr<CANBus> OpenArmHWFlex::global_canbus_;
std::shared_ptr<MotorControl> OpenArmHWFlex::global_motor_control_;
std::mutex OpenArmHWFlex::can_mutex_;

OpenArmHWFlex::OpenArmHWFlex() = default;

// ---- small utils ----
std::string OpenArmHWFlex::trim(std::string s) {
  auto not_space = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

DM_Motor_Type OpenArmHWFlex::str_to_motor_type(std::string s) {
  s = trim(s);
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "DM4310") return DM_Motor_Type::DM4310;
  if (s == "DM4340") return DM_Motor_Type::DM4340;
  // 필요한 경우 여기에 다른 모터 타입 추가
  RCLCPP_WARN(rclcpp::get_logger("OpenArmHWFlex"),
              "Unknown motor type '%s', fallback to DM4340", s.c_str());
  return DM_Motor_Type::DM4340;
}

hardware_interface::CallbackReturn OpenArmHWFlex::on_init(
    const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- 하드웨어 파라미터 읽기 ----
  auto it = info.hardware_parameters.find("prefix");
  if (it != info.hardware_parameters.end()) prefix_ = it->second;

  it = info.hardware_parameters.find("disable_torque");
  if (it != info.hardware_parameters.end()) {
    std::string v = it->second; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    disable_torque_ = (v == "true" || v == "1");
  }

  it = info.hardware_parameters.find("can_fd");
  if (it != info.hardware_parameters.end()) {
    std::string v = it->second; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    can_fd_ = (v == "true" || v == "1");
  }

  // ---- 모터 관련 파라미터 (벡터) 파싱 ----
  auto parse_vector = [&](const std::string& name, auto& out_vec, auto converter) {
    auto it_param = info.hardware_parameters.find(name);
    if (it_param == info.hardware_parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHWFlex"), "Missing param: %s for %s", name.c_str(), prefix_.c_str());
      return false;
    }
    std::stringstream ss(it_param->second);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      tok = trim(tok);
      if (!tok.empty()) out_vec.push_back(converter(tok));
    }
    return true;
  };

  if (!parse_vector("motor_types", motor_types_, [this](const std::string& s){ return str_to_motor_type(s); })) return hardware_interface::CallbackReturn::ERROR;
  if (!parse_vector("can_device_ids", can_device_ids_, [](const std::string& s){ return static_cast<uint16_t>(std::stoi(s, nullptr, 0)); })) return hardware_interface::CallbackReturn::ERROR;
  if (!parse_vector("can_master_ids", can_master_ids_, [](const std::string& s){ return static_cast<uint16_t>(std::stoi(s, nullptr, 0)); })) return hardware_interface::CallbackReturn::ERROR;
  if (!parse_vector("kp", kp_, [](const std::string& s){ return std::stod(s); })) return hardware_interface::CallbackReturn::ERROR;
  if (!parse_vector("kd", kd_, [](const std::string& s){ return std::stod(s); })) return hardware_interface::CallbackReturn::ERROR;

  // ---- 파라미터 개수 검증 ----
  const size_t N = motor_types_.size();
  if (can_device_ids_.size() != N || can_master_ids_.size() != N ||
      kp_.size() != N || kd_.size() != N || info_.joints.size() != N) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHWFlex"),
      "Param size mismatch for %s: N=%zu, dev=%zu, master=%zu, kp=%zu, kd=%zu, joints=%zu",
      prefix_.c_str(), N, can_device_ids_.size(), can_master_ids_.size(), kp_.size(), kd_.size(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- 공유 리소스 초기화 (최초 한 번만) ----
  {
    std::lock_guard<std::mutex> lk(can_mutex_);
    if (!global_canbus_) {
      auto it_dev = info.hardware_parameters.find("can_device");
      if (it_dev == info.hardware_parameters.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmHWFlex"), "Missing global param: can_device");
        return hardware_interface::CallbackReturn::ERROR;
      }
      global_canbus_ = std::make_shared<CANBus>(it_dev->second, can_fd_ ? CAN_MODE_FD : CAN_MODE_CLASSIC);
      global_motor_control_ = std::make_shared<MotorControl>(*global_canbus_);
    }
  }

  // ---- 모터 객체 생성 및 공유 MotorControl에 등록 ----
  motors_.resize(N);
  for (size_t i = 0; i < N; ++i) {
    motors_[i] = std::make_unique<Motor>(motor_types_[i], can_device_ids_[i], can_master_ids_[i]);
    global_motor_control_->addMotor(*motors_[i]);
  }

  // ---- 상태/명령 변수 초기화 ----
  pos_states_.assign(N, 0.0);
  vel_states_.assign(N, 0.0);
  tau_states_.assign(N, 0.0);
  pos_commands_.assign(N, 0.0);
  vel_commands_.assign(N, 0.0);
  tau_ff_commands_.assign(N, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHWFlex::on_configure(
    const rclcpp_lifecycle::State &) {
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHWFlex::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(motors_.size() * 3);
  for (size_t i = 0; i < motors_.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &tau_states_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHWFlex::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(motors_.size() * 3);
  for (size_t i = 0; i < motors_.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &tau_ff_commands_[i]);
  }
  return out;
}

hardware_interface::CallbackReturn
OpenArmHWFlex::on_activate(const rclcpp_lifecycle::State &) {
  for (auto & m : motors_) global_motor_control_->enable(*m);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OpenArmHWFlex::on_deactivate(const rclcpp_lifecycle::State &) {
  for (auto & m : motors_) {
    global_motor_control_->controlMIT(*m, 0.0, 0.0, 0.0, 0.0, 0.0);
    global_motor_control_->disable(*m);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OpenArmHWFlex::read(const rclcpp::Time &, const rclcpp::Duration &) {
  // Non-blocking drain: read all available CAN frames without blocking
  // With non-blocking socket, recv() returns immediately if no data
  // Loop until no more data (max 100 iterations as safety limit)
  for (int i = 0; i < 100; ++i) {
    if (!global_motor_control_->recv()) {
      break;  // No more data available, exit immediately
    }
  }

  // 업데이트된 모터 상태를 자신의 상태 변수로 복사
  for (size_t i = 0; i < motors_.size(); ++i) {
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArmHWFlex::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (disable_torque_) {
    // disable_torque 모드에서도 write는 호출되므로, MIT(0) 명령을 보내서 모터 토크를 끔
    for (size_t i = 0; i < motors_.size(); ++i) {
      global_motor_control_->controlMIT(*motors_[i], 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < motors_.size(); ++i) {
    global_motor_control_->controlMIT(*motors_[i],
                               kp_.at(i), kd_.at(i),
                               pos_commands_[i],
                               vel_commands_[i],
                               tau_ff_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

} // namespace openarm_static_bimanual_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_static_bimanual_hardware::OpenArmHWFlex,
                       hardware_interface::SystemInterface)
