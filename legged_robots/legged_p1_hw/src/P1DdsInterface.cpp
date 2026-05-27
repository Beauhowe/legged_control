#include "legged_p1_hw/P1DdsInterface.h"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/logging.hpp>

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace legged {
namespace {

bool readExact(int fd, void* data, size_t size) {
  auto* bytes = static_cast<char*>(data);
  while (size > 0) {
    const ssize_t received = ::read(fd, bytes, size);
    if (received < 0 && errno == EINTR) {
      continue;
    }
    if (received <= 0) {
      return false;
    }
    bytes += static_cast<size_t>(received);
    size -= static_cast<size_t>(received);
  }
  return true;
}

}  // namespace

P1DdsInterface::P1DdsInterface() = default;

P1DdsInterface::~P1DdsInterface() { shutdown(); }

bool P1DdsInterface::init(uint32_t domain, const std::string& stateTopic, const std::string& imuTopic,
                          const std::string& commandTopic) {
  int statePipe[2];
  int commandPipe[2];
  if (::pipe(statePipe) != 0 || ::pipe(commandPipe) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to create P1 DDS worker pipes: %s", std::strerror(errno));
    return false;
  }

  workerPid_ = ::fork();
  if (workerPid_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to fork P1 DDS worker: %s", std::strerror(errno));
    ::close(statePipe[0]);
    ::close(statePipe[1]);
    ::close(commandPipe[0]);
    ::close(commandPipe[1]);
    return false;
  }

  if (workerPid_ == 0) {
    ::close(statePipe[0]);
    ::close(commandPipe[1]);
    const auto prefix = ament_index_cpp::get_package_prefix("legged_p1_hw");
    const auto workerPath = prefix + "/lib/legged_p1_hw/p1_dds_worker";
    const auto stateFd = std::to_string(statePipe[1]);
    const auto commandFd = std::to_string(commandPipe[0]);
    const auto domainText = std::to_string(domain);
    const std::string fastDdsLib = P1_HW_FASTDDS_LIB_DIR;
    const char* oldPath = std::getenv("LD_LIBRARY_PATH");
    const auto newPath = oldPath == nullptr ? fastDdsLib : fastDdsLib + ":" + oldPath;
    ::setenv("LD_LIBRARY_PATH", newPath.c_str(), 1);
    ::execl(workerPath.c_str(), workerPath.c_str(), "--state-fd", stateFd.c_str(), "--command-fd", commandFd.c_str(),
            "--dds-domain", domainText.c_str(), "--state-topic", stateTopic.c_str(), "--imu-topic", imuTopic.c_str(),
            "--command-topic", commandTopic.c_str(), static_cast<char*>(nullptr));
    std::perror("execl p1_dds_worker");
    _exit(127);
  }

  ::close(statePipe[1]);
  ::close(commandPipe[0]);
  stateReadFd_ = statePipe[0];
  commandWriteFd_ = commandPipe[1];
  running_ = true;
  stateThread_ = std::thread([this] { stateReadLoop(); });

  RCLCPP_INFO(rclcpp::get_logger("legged_p1_hw"), "P1 DDS worker started: state=%s imu=%s command=%s domain=%u",
              stateTopic.c_str(), imuTopic.c_str(), commandTopic.c_str(), domain);
  return true;
}

void P1DdsInterface::shutdown() {
  running_ = false;
  if (stateReadFd_ >= 0) {
    ::close(stateReadFd_);
    stateReadFd_ = -1;
  }
  if (commandWriteFd_ >= 0) {
    ::close(commandWriteFd_);
    commandWriteFd_ = -1;
  }
  if (stateThread_.joinable()) {
    stateThread_.join();
  }
  if (workerPid_ > 0) {
    ::kill(workerPid_, SIGTERM);
    ::waitpid(workerPid_, nullptr, 0);
    workerPid_ = -1;
  }
}

bool P1DdsInterface::getLatestMotorState(MotorState& state) const {
  if (!hasState_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(stateMutex_);
  state = latestState_;
  return true;
}

bool P1DdsInterface::getLatestImuState(ImuState& state) const {
  if (!hasImuState_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(stateMutex_);
  state = latestImuState_;
  return true;
}

bool P1DdsInterface::writeCommand(const Command& command) {
  if (commandWriteFd_ < 0) {
    return false;
  }
  return writeAll(commandWriteFd_, &command, sizeof(command));
}

void P1DdsInterface::stateReadLoop() {
  while (running_) {
    StatePacket packet;
    if (!readExact(stateReadFd_, &packet, sizeof(packet))) {
      break;
    }
    if (packet.tag == StatePacketTag::MotorState) {
      updateMotorState(packet.motor_state);
    } else if (packet.tag == StatePacketTag::ImuState) {
      updateImuState(packet.imu_state);
    }
  }
}

bool P1DdsInterface::writeAll(int fd, const void* data, size_t size) const {
  const auto* bytes = static_cast<const char*>(data);
  while (size > 0) {
    const ssize_t written = ::write(fd, bytes, size);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      return false;
    }
    bytes += static_cast<size_t>(written);
    size -= static_cast<size_t>(written);
  }
  return true;
}

void P1DdsInterface::updateMotorState(const MotorState& state) {
  {
    std::lock_guard<std::mutex> lock(stateMutex_);
    latestState_ = state;
  }
  hasState_.store(true);
}

void P1DdsInterface::updateImuState(const ImuState& state) {
  {
    std::lock_guard<std::mutex> lock(stateMutex_);
    latestImuState_ = state;
  }
  hasImuState_.store(true);
}

}  // namespace legged
