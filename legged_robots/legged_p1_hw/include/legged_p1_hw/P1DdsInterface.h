#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <thread>
#include <sys/types.h>

namespace legged {

// P1 DDS 通讯封装：订阅电机/IMU 状态，发布 Motor_Command_12::motor_cmd。
class P1DdsInterface {
 public:
  // 对应 Motor_State_12::motor_state，下位机发布 12 个电机状态。
  struct MotorState {
    std::array<uint32_t, 12> motor_id{};
    std::array<int64_t, 12> last_seen_time{};
    std::array<uint8_t, 12> status{};
    std::array<float, 12> position{};
    std::array<float, 12> speed{};
    std::array<float, 12> current{};
    std::array<float, 12> motor_temperature{};
    std::array<float, 12> mos_temperature{};
    std::array<bool, 12> is_online{};
  };

  // 对应 Motor_Command_12::motor_cmd，上位机发送 12 个电机控制量。
  struct Command {
    std::array<float, 12> kp{};
    std::array<float, 12> kd{};
    std::array<float, 12> position_desired{};
    std::array<float, 12> velocity_desired{};
    std::array<float, 12> torque{};
    std::array<float, 12> feedforward_torque{};
    std::array<uint32_t, 12> mode{};
  };

  // 对应 imu_topic_base::IMUData，下位机发布机身 IMU。
  struct ImuState {
    double orientation_w{1.0};
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
    double linear_acceleration_x{0.0};
    double linear_acceleration_y{0.0};
    double linear_acceleration_z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  };

  P1DdsInterface();
  ~P1DdsInterface();

  bool init(uint32_t domain, const std::string& stateTopic, const std::string& imuTopic, const std::string& commandTopic);
  void shutdown();

  bool getLatestMotorState(MotorState& state) const;
  bool getLatestImuState(ImuState& state) const;
  bool writeCommand(const Command& command);

 private:
  enum class StatePacketTag : uint32_t { MotorState = 1, ImuState = 2 };

  struct StatePacket {
    StatePacketTag tag{StatePacketTag::MotorState};
    MotorState motor_state{};
    ImuState imu_state{};
  };

  void updateMotorState(const MotorState& state);
  void updateImuState(const ImuState& state);
  void stateReadLoop();
  bool writeAll(int fd, const void* data, size_t size) const;

  // Worker 进程写入最新状态，controller_manager 的 read() 线程读取。
  mutable std::mutex stateMutex_;
  MotorState latestState_{};
  ImuState latestImuState_{};
  std::atomic<bool> hasState_{false};
  std::atomic<bool> hasImuState_{false};

  std::atomic<bool> running_{false};
  int stateReadFd_{-1};
  int commandWriteFd_{-1};
  pid_t workerPid_{-1};
  std::thread stateThread_;
};

}  // namespace legged
