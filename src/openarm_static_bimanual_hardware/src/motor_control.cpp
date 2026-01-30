// Copyright 2025 Reazon Holdings, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// http://www.apache.org/licenses/LICENSE-2.0

#include "openarm_static_bimanual_hardware/motor_control.hpp"
#include "openarm_static_bimanual_hardware/motor.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <algorithm>

namespace {
// 어떤 map 타입(std::map / std::unordered_map)도 받도록 템플릿화
template <typename MapT>
inline Motor* find_motor_by_id_(MapT& map, uint16_t can_id, const uint8_t* data) {
  // can_id가 0x000이면 프레임 data[0]의 하위 nibble로 MasterID를 구분하던 기존 로직을 유지
  uint16_t key = (can_id == 0x000 && data) ? static_cast<uint16_t>(data[0] & 0x0F)
                                           : can_id;
  auto it = map.find(key);
  return (it != map.end()) ? it->second : nullptr;
}
} // namespace


// -----------------------------------------------------------------------------

MotorControl::MotorControl(CANBus& canbus) : canbus_(canbus) {}

bool MotorControl::addMotor(Motor& motor) {
  motors_map_[motor.SlaveID] = &motor;
  if (motor.MasterID != 0) {
    motors_map_[motor.MasterID] = &motor;
  }
  return true;
}

void MotorControl::enable(Motor& motor) {
  controlCmd(motor, 0xFC);
  sleep(0.3);
}

void MotorControl::disable(Motor& motor) {
  controlCmd(motor, 0xFD);
  sleep(0.3);
}

void MotorControl::set_zero_position(Motor& motor) {
  controlCmd(motor, 0xFE);
  sleep(0.3);
  recv();
}

void MotorControl::controlMIT(Motor& motor, double kp, double kd, double q,
                              double dq, double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
  uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

  int motor_index = static_cast<int>(motor.MotorType);
  double Q_MAX  = Limit_Param[motor_index][0];
  double DQ_MAX = Limit_Param[motor_index][1];
  double TAU_MAX= Limit_Param[motor_index][2];

  uint16_t q_uint   = double_to_uint(q,  -Q_MAX,  Q_MAX,  16);
  uint16_t dq_uint  = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
  uint16_t tau_uint = double_to_uint(tau,-TAU_MAX,TAU_MAX,12);

  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>((q_uint >> 8) & 0xFF),
      static_cast<uint8_t>(q_uint & 0xFF),
      static_cast<uint8_t>(dq_uint >> 4),
      static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
      static_cast<uint8_t>(kp_uint & 0xFF),
      static_cast<uint8_t>(kd_uint >> 4),
      static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
      static_cast<uint8_t>(tau_uint & 0xFF)};

  sendData(motor.SlaveID, data);
  // recv();
}

void MotorControl::controlMIT2(Motor& motor, double kp, double kd, double q,
                               double dq, double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
  uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

  int motor_index = static_cast<int>(motor.MotorType);
  double Q_MAX  = Limit_Param[motor_index][0];
  double DQ_MAX = Limit_Param[motor_index][1];
  double TAU_MAX= Limit_Param[motor_index][2];

  uint16_t q_uint   = double_to_uint(q,  -Q_MAX,  Q_MAX,  16);
  uint16_t dq_uint  = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
  uint16_t tau_uint = double_to_uint(tau,-TAU_MAX,TAU_MAX,12);

  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>((q_uint >> 8) & 0xFF),
      static_cast<uint8_t>(q_uint & 0xFF),
      static_cast<uint8_t>(dq_uint >> 4),
      static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
      static_cast<uint8_t>(kp_uint & 0xFF),
      static_cast<uint8_t>(kd_uint >> 4),
      static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
      static_cast<uint8_t>(tau_uint & 0xFF)};

  sendData(motor.SlaveID, data);
}

void MotorControl::sendData(uint16_t motor_id,
                            const std::array<uint8_t, 8>& data) {
  canbus_.send(motor_id, data);
}

bool MotorControl::recv() {
  uint16_t id;
  uint8_t len;
  std::array<uint8_t, 64> data = canbus_.recv(id, len);

  // No data received (non-blocking socket returned empty)
  if (len == 0) {
    return false;
  }

  if (canbus_.whichCAN() == CAN_MODE_CLASSIC) {
    can_frame frame{};
    frame.can_id  = id;
    frame.can_dlc = len;
    std::memcpy(frame.data, data.data(), len);
    processPacket(frame);
  } else if (canbus_.whichCAN() == CAN_MODE_FD) {
    canfd_frame fd_frame{};
    fd_frame.can_id = id;
    fd_frame.len    = len;
    std::memcpy(fd_frame.data, data.data(), len);
    processPacketFD(fd_frame);
  }
  return true;
}

void MotorControl::control_delay(Motor& motor, double kp, double kd, double q,
                                 double dq, double tau, double delay) {
  controlMIT(motor, kp, kd, q, dq, tau);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(delay)));
}

void MotorControl::controlPosVel(Motor& motor, double pos, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosVel ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x100 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto pos_buf = float_to_uint8s(static_cast<float>(pos));

  for (int i = 0; i < 4; ++i) {
    data_buf[i]     = pos_buf[i];
    data_buf[i + 4] = vel_buf[i];
  }

  sendData(motor_id, data_buf);
  recv();
}

void MotorControl::controlPosVel2(Motor& motor, double pos, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosVel2 ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x100 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));

  for (int i = 0; i < 4; ++i) {
    data_buf[i]     = pos_buf[i];
    data_buf[i + 4] = vel_buf[i];
  }

  sendData(motor_id, data_buf);
}

void MotorControl::controlVel(Motor& motor, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlVel ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x200 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  for (int i = 0; i < 4; ++i) data_buf[i] = vel_buf[i];

  sendData(motor_id, data_buf);
  recv();
}

void MotorControl::controlVel2(Motor& motor, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlVel2 ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x200 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  for (int i = 0; i < 4; ++i) data_buf[i] = vel_buf[i];

  sendData(motor_id, data_buf);
}

void MotorControl::controlPosForce(Motor& motor, double pos, double vel,
                                   double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosForce ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x300 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto tau_buf = float_to_uint8s(static_cast<float>(tau));

  for (int i = 0; i < 4; ++i) data_buf[i] = pos_buf[i];
  data_buf[4] = vel_buf[0];
  data_buf[5] = vel_buf[1];
  data_buf[6] = tau_buf[0];
  data_buf[7] = tau_buf[1];

  sendData(motor_id, data_buf);
  recv();
}

void MotorControl::controlPosForce2(Motor& motor, double pos, double vel,
                                    double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosForce ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = static_cast<uint16_t>(0x300 + motor.SlaveID);
  std::array<uint8_t, 8> data_buf{};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto tau_buf = float_to_uint8s(static_cast<float>(tau));

  for (int i = 0; i < 4; ++i) data_buf[i] = pos_buf[i];
  data_buf[4] = vel_buf[0];
  data_buf[5] = vel_buf[1];
  data_buf[6] = tau_buf[0];
  data_buf[7] = tau_buf[1];

  sendData(motor_id, data_buf);
}

bool MotorControl::switchControlMode(Motor& motor, Control_Type control_mode) {
  const int max_retries = 20;
  const double retry_interval = 0.1;
  DM_variable RID = DM_variable::CTRL_MODE;

  writeMotorParam(motor, RID, static_cast<int>(control_mode));

  for (int i = 0; i < max_retries; ++i) {
    usleep(static_cast<useconds_t>(retry_interval * 1e6));
    recv_set_param_data();
    if (motor.getParam(static_cast<int>(RID)) ==
        static_cast<int>(control_mode)) {
      return true;
    }
  }
  return false;
}

void MotorControl::save_motor_param(Motor& motor) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
      0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};

  disable(motor);
  canbus_.send(0x7FF, data);
  usleep(1000);
}

void MotorControl::recv_set_param_data() {
  uint16_t id;
  uint8_t len;
  std::array<uint8_t, 64> data = canbus_.recv(id, len);

  // 디버그 프린트(필요하면 유지)
  if (len >= 8) {
    std::cout << "CANID: 0x" << std::hex << id << std::dec << " LEN:" << int(len) << "\n";
    for (int i = 0; i < 8; ++i)
      std::cout << "0x" << std::hex << int(data[i]) << " ";
    std::cout << std::dec << "\n";
  }
}

// ---------------------------- 응답 파싱 핵심 변경부 ----------------------------

void MotorControl::processPacket(const can_frame& frame) {
  Motor* motor = find_motor_by_id_(motors_map_, frame.can_id, frame.data);
  if (!motor) return;

  // 상태 프레임 payload 해석(기존 구현 유지)
  uint16_t q_uint   = static_cast<uint16_t>((frame.data[1] << 8) | frame.data[2]);
  uint16_t dq_uint  = static_cast<uint16_t>((frame.data[3] << 4) | (frame.data[4] >> 4));
  uint16_t tau_uint = static_cast<uint16_t>(((frame.data[4] & 0x0F) << 8) | frame.data[5]);
  int t_mos         = static_cast<int>(frame.data[6]);
  int t_rotor       = static_cast<int>(frame.data[7]);

  const int idx = static_cast<int>(motor->MotorType);
  double Q_MAX   = Limit_Param[idx][0];
  double DQ_MAX  = Limit_Param[idx][1];
  double TAU_MAX = Limit_Param[idx][2];

  double recv_q   = uint_to_double(q_uint,  -Q_MAX,  Q_MAX,  16);
  double recv_dq  = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
  double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

  motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
}

void MotorControl::processPacketFD(const canfd_frame& frame) {
  Motor* motor = find_motor_by_id_(motors_map_, frame.can_id, frame.data);
  if (!motor) return;

  uint16_t q_uint   = static_cast<uint16_t>((frame.data[1] << 8) | frame.data[2]);
  uint16_t dq_uint  = static_cast<uint16_t>((frame.data[3] << 4) | (frame.data[4] >> 4));
  uint16_t tau_uint = static_cast<uint16_t>(((frame.data[4] & 0x0F) << 8) | frame.data[5]);
  int t_mos         = static_cast<int>(frame.data[6]);
  int t_rotor       = static_cast<int>(frame.data[7]);

  const int idx = static_cast<int>(motor->MotorType);
  double Q_MAX   = Limit_Param[idx][0];
  double DQ_MAX  = Limit_Param[idx][1];
  double TAU_MAX = Limit_Param[idx][2];

  double recv_q   = uint_to_double(q_uint,  -Q_MAX,  Q_MAX,  16);
  double recv_dq  = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
  double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

  motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
}

// ----------------------------- 파라미터 읽기/쓰기 -----------------------------

void MotorControl::controlCmd(Motor& motor, uint8_t cmd) {
  std::array<uint8_t, 8> data_buf = {0xFF, 0xFF, 0xFF, 0xFF,
                                     0xFF, 0xFF, 0xFF, cmd};
  sendData(motor.SlaveID, data_buf);
}

void MotorControl::readRIDParam(Motor& motor, DM_variable RID) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
      0x33,
      static_cast<uint8_t>(RID),
      0x00, 0x00, 0x00, 0x00};
  canbus_.send(0x7FF, data);
}

void MotorControl::writeMotorParam(Motor& motor, DM_variable RID, double value) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
      0x55,                              // write param
      static_cast<uint8_t>(RID),         // register id
      0x00, 0x00, 0x00, 0x00             // payload(4B)
  };

  if (is_in_ranges(static_cast<int>(RID))) {
    // 정수형 파라미터 → int32
    const int32_t vi = static_cast<int32_t>(value);
    std::memcpy(&data[4], &vi, sizeof(vi));  // (리틀엔디안 가정)
  } else {
    // 일반 파라미터 → float(4B)
    const float vf = static_cast<float>(value);
    std::memcpy(&data[4], &vf, sizeof(vf));
  }

  canbus_.send(0x7FF, data);
}
