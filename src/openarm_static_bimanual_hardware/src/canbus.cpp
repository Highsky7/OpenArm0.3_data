#include "openarm_static_bimanual_hardware/canbus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>     // (여기선 O_NONBLOCK 안 씀, 그래도 포함 OK)
#include <sys/ioctl.h>
#include <unistd.h>

CANBus::CANBus(const std::string& interface, int mode) : mode_(mode) {
  struct ifreq ifr;
  struct sockaddr_can addr;

  sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    perror("Error while opening CAN socket");
    std::exit(EXIT_FAILURE);
  }

  // Increase socket receive buffer to 512KB to prevent overflow
  int rcvbuf = 524288;  // 512KB
  ::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  // Reduce timeout to 1ms for faster recv loop (was 2ms)
  ::timeval tv{0, 1000};  // 0.001s
  ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
  if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    perror("Error getting interface index");
    std::exit(EXIT_FAILURE);
  }

  std::memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (mode_ == CAN_MODE_FD) {
    int enable_canfd = 1;
    if (::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                     &enable_canfd, sizeof(enable_canfd)) < 0) {
      perror("CAN FD setsockopt failed");
      std::exit(EXIT_FAILURE);
    }
  }

  if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("Error in CAN socket bind");
    std::exit(EXIT_FAILURE);
  }
}

CANBus::~CANBus() { ::close(sock_); }

int CANBus::whichCAN() { return mode_; }

bool CANBus::send(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
  return (mode_ == CAN_MODE_FD) ? sendFD(motor_id, data) : sendClassic(motor_id, data);
}

std::array<uint8_t, 64> CANBus::recv(uint16_t& out_id, uint8_t& out_len) {
  std::array<uint8_t, 64> buffer{};
  if (mode_ == CAN_MODE_FD) {
    auto f = recvFD();
    out_id = f.can_id;
    out_len = f.len;
    if (f.len > 0) std::copy(f.data, f.data + f.len, buffer.begin());
  } else {
    auto f = recvClassic();
    out_id = f.can_id;
    out_len = f.can_dlc;
    if (f.can_dlc > 0) std::copy(f.data, f.data + f.can_dlc, buffer.begin());
  }
  return buffer;
}

bool CANBus::sendClassic(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
  struct can_frame f{};
  f.can_id  = motor_id;
  f.can_dlc = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), f.data);

  ssize_t n = ::write(sock_, &f, sizeof(f));
  if (n != static_cast<ssize_t>(sizeof(f))) {
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false; // 송신큐 꽉참
    perror("Error sending CAN frame");
    return false;
  }
  return true;
}

bool CANBus::sendFD(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
  struct canfd_frame f{};
  f.can_id = motor_id;
  f.len    = static_cast<__u8>(data.size());
  f.flags  = CANFD_BRS;
  std::copy(data.begin(), data.end(), f.data);

  ssize_t n = ::write(sock_, &f, sizeof(f));
  if (n != static_cast<ssize_t>(sizeof(f))) {
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
    perror("Error sending CAN FD frame");
    return false;
  }
  return true;
}

struct can_frame CANBus::recvClassic() {
  struct can_frame f{};
  int nbytes = ::read(sock_, &f, sizeof(f));
  if (nbytes <= 0) {
    // timeout 또는 오류 → 데이터 없음으로 취급
    f.can_dlc = 0;
  }
  return f;
}

struct canfd_frame CANBus::recvFD() {
  struct canfd_frame f{};
  int nbytes = ::read(sock_, &f, sizeof(f));
  if (nbytes <= 0) {
    f.len = 0;
  }
  return f;
}
