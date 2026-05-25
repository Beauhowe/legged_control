#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

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
  class MotorStatePubSubType;
  class ImuStatePubSubType;
  class CommandPubSubType;
  class MotorStateListener;
  class ImuStateListener;

  void updateMotorState(const MotorState& state);
  void updateImuState(const ImuState& state);

  // DDS 回调线程写入最新状态，controller_manager 的 read() 线程读取。
  mutable std::mutex stateMutex_;
  MotorState latestState_{};
  ImuState latestImuState_{};
  std::atomic<bool> hasState_{false};
  std::atomic<bool> hasImuState_{false};

  // Fast DDS 实体生命周期由 init()/shutdown() 管理。
  eprosima::fastdds::dds::DomainParticipant* participant_{nullptr};
  eprosima::fastdds::dds::Publisher* publisher_{nullptr};
  eprosima::fastdds::dds::Subscriber* subscriber_{nullptr};
  eprosima::fastdds::dds::Topic* stateTopicHandle_{nullptr};
  eprosima::fastdds::dds::Topic* imuTopicHandle_{nullptr};
  eprosima::fastdds::dds::Topic* commandTopicHandle_{nullptr};
  eprosima::fastdds::dds::DataReader* stateReader_{nullptr};
  eprosima::fastdds::dds::DataReader* imuReader_{nullptr};
  eprosima::fastdds::dds::DataWriter* commandWriter_{nullptr};
  eprosima::fastdds::dds::TypeSupport stateType_;
  eprosima::fastdds::dds::TypeSupport imuType_;
  eprosima::fastdds::dds::TypeSupport commandType_;
  std::unique_ptr<MotorStateListener> stateListener_;
  std::unique_ptr<ImuStateListener> imuListener_;
};

}  // namespace legged
