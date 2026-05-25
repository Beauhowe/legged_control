#include "legged_p1_hw/P1DdsInterface.h"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/rtps/common/CdrSerialization.hpp>
#include <rclcpp/logging.hpp>

namespace legged {
namespace {

// Fast DDS 生成代码通常按数组逐元素写 CDR，这里手写同样的固定数组格式。
// 注意：这里不是自定义协议，字段顺序必须严格等于对应 IDL 中的声明顺序。
template <typename T, size_t N>
void serializeArray(eprosima::fastcdr::Cdr& cdr, const std::array<T, N>& values) {
  for (const auto& value : values) {
    cdr << value;
  }
}

template <typename T, size_t N>
void deserializeArray(eprosima::fastcdr::Cdr& cdr, std::array<T, N>& values) {
  for (auto& value : values) {
    cdr >> value;
  }
}

}  // namespace

// 手写 Motor_State_12::motor_state 的 TopicDataType，保持和 fastdds_bridge 中 IDL 的线格式一致。
// 这个类的作用等价于 fastddsgen 生成的 motor_statePubSubType。
class P1DdsInterface::MotorStatePubSubType final : public eprosima::fastdds::dds::TopicDataType {
 public:
  MotorStatePubSubType() {
    // type name 必须和下位机/bridge 发布端注册的类型名完全一致，否则 DDS 不会匹配。
    set_name("Motor_State_12::motor_state");
    // 固定长度数组，没有动态成员；这里给足 payload 缓冲区，避免序列化时越界。
    max_serialized_type_size = 512;
    // 这个 IDL 没有 @key 字段，所以不需要计算实例 key。
    is_compute_key_provided = false;
  }

  bool serialize(const void* const data, eprosima::fastdds::rtps::SerializedPayload_t& payload,
                 eprosima::fastdds::dds::DataRepresentationId_t dataRepresentation) override {
    // DataWriter 会把 void* 传进来，这里恢复成 P1DdsInterface 内部使用的状态结构。
    const auto* state = static_cast<const MotorState*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.max_size);
    eprosima::fastcdr::Cdr serializer(
        fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::CdrVersion::XCDRv1
            : eprosima::fastcdr::CdrVersion::XCDRv2);
    // encapsulation/encoding flag 是 DDS CDR 数据头，接收端靠它判断大小端和 XCDR 版本。
    payload.encapsulation = serializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    serializer.set_encoding_flag(
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR
            : eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR2);

    try {
      serializer.serialize_encapsulation();
      // 字段顺序对应 motor_state_12.idl：motorId, lastSeenTime, status, position...
      serializeArray(serializer, state->motor_id);
      serializeArray(serializer, state->last_seen_time);
      serializeArray(serializer, state->status);
      serializeArray(serializer, state->position);
      serializeArray(serializer, state->speed);
      serializeArray(serializer, state->current);
      serializeArray(serializer, state->motor_temperature);
      serializeArray(serializer, state->mos_temperature);
      serializeArray(serializer, state->is_online);
      serializer.set_dds_cdr_options({0, 0});
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    payload.length = static_cast<uint32_t>(serializer.get_serialized_data_length());
    return true;
  }

  bool deserialize(eprosima::fastdds::rtps::SerializedPayload_t& payload, void* data) override {
    auto* state = static_cast<MotorState*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.length);
    eprosima::fastcdr::Cdr deserializer(fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);

    try {
      // 先读 DDS CDR 数据头，再按 IDL 字段顺序解包 payload。
      deserializer.read_encapsulation();
      payload.encapsulation = deserializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
      deserializeArray(deserializer, state->motor_id);
      deserializeArray(deserializer, state->last_seen_time);
      deserializeArray(deserializer, state->status);
      deserializeArray(deserializer, state->position);
      deserializeArray(deserializer, state->speed);
      deserializeArray(deserializer, state->current);
      deserializeArray(deserializer, state->motor_temperature);
      deserializeArray(deserializer, state->mos_temperature);
      deserializeArray(deserializer, state->is_online);
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    return true;
  }

  uint32_t calculate_serialized_size(const void* const, eprosima::fastdds::dds::DataRepresentationId_t) override {
    return max_serialized_type_size;
  }

  bool compute_key(eprosima::fastdds::rtps::SerializedPayload_t&, eprosima::fastdds::rtps::InstanceHandle_t&,
                   bool = false) override {
    return false;
  }

  bool compute_key(const void* const, eprosima::fastdds::rtps::InstanceHandle_t&, bool = false) override {
    return false;
  }

  // Fast DDS 在 reader 内部需要创建/释放样本对象时会调用 create_data/delete_data。
  void* create_data() override { return reinterpret_cast<void*>(new MotorState()); }

  void delete_data(void* data) override { delete reinterpret_cast<MotorState*>(data); }

  void register_type_object_representation() override {}

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  bool is_bounded() const override { return true; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  bool is_plain(eprosima::fastdds::dds::DataRepresentationId_t) const override { return false; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
  bool construct_sample(void* memory) const override {
    new (memory) MotorState();
    return true;
  }
#endif
};

// 手写 imu_topic_base::IMUData 的 TopicDataType，用于订阅下位机 IMU。
// 这里保留 roll/pitch/yaw，虽然 P1HW 当前主要导出四元数、角速度和线加速度。
class P1DdsInterface::ImuStatePubSubType final : public eprosima::fastdds::dds::TopicDataType {
 public:
  ImuStatePubSubType() {
    // 对应 /workspace/src/fastdds_bridge/dds_topic/imu_topic_base 中的类型名。
    set_name("imu_topic_base::IMUData");
    // IMUData 由 13 个 double 组成，加上 CDR 头部和对齐空间。
    max_serialized_type_size = 112;
    is_compute_key_provided = false;
  }

  bool serialize(const void* const data, eprosima::fastdds::rtps::SerializedPayload_t& payload,
                 eprosima::fastdds::dds::DataRepresentationId_t dataRepresentation) override {
    const auto* imu = static_cast<const ImuState*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.max_size);
    eprosima::fastcdr::Cdr serializer(
        fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::CdrVersion::XCDRv1
            : eprosima::fastcdr::CdrVersion::XCDRv2);
    // encapsulation/encoding flag 是 DDS CDR 数据头，接收端靠它判断大小端和 XCDR 版本。
    payload.encapsulation = serializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    serializer.set_encoding_flag(
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR
            : eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR2);

    try {
      serializer.serialize_encapsulation();
      // 字段顺序对应 imu_topic_base.idl：orientation, angular_velocity, linear_acceleration, rpy。
      serializer << imu->orientation_w << imu->orientation_x << imu->orientation_y << imu->orientation_z;
      serializer << imu->angular_velocity_x << imu->angular_velocity_y << imu->angular_velocity_z;
      serializer << imu->linear_acceleration_x << imu->linear_acceleration_y << imu->linear_acceleration_z;
      serializer << imu->roll << imu->pitch << imu->yaw;
      serializer.set_dds_cdr_options({0, 0});
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    payload.length = static_cast<uint32_t>(serializer.get_serialized_data_length());
    return true;
  }

  bool deserialize(eprosima::fastdds::rtps::SerializedPayload_t& payload, void* data) override {
    auto* imu = static_cast<ImuState*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.length);
    eprosima::fastcdr::Cdr deserializer(fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);

    try {
      // 先读 DDS CDR 数据头，再按 IDL 字段顺序解包 payload。
      deserializer.read_encapsulation();
      payload.encapsulation = deserializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
      deserializer >> imu->orientation_w >> imu->orientation_x >> imu->orientation_y >> imu->orientation_z;
      deserializer >> imu->angular_velocity_x >> imu->angular_velocity_y >> imu->angular_velocity_z;
      deserializer >> imu->linear_acceleration_x >> imu->linear_acceleration_y >> imu->linear_acceleration_z;
      deserializer >> imu->roll >> imu->pitch >> imu->yaw;
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    return true;
  }

  uint32_t calculate_serialized_size(const void* const, eprosima::fastdds::dds::DataRepresentationId_t) override {
    return max_serialized_type_size;
  }

  bool compute_key(eprosima::fastdds::rtps::SerializedPayload_t&, eprosima::fastdds::rtps::InstanceHandle_t&,
                   bool = false) override {
    return false;
  }

  bool compute_key(const void* const, eprosima::fastdds::rtps::InstanceHandle_t&, bool = false) override {
    return false;
  }

  // Fast DDS 在 reader 内部需要创建/释放样本对象时会调用 create_data/delete_data。
  void* create_data() override { return reinterpret_cast<void*>(new ImuState()); }

  void delete_data(void* data) override { delete reinterpret_cast<ImuState*>(data); }

  void register_type_object_representation() override {}

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  bool is_bounded() const override { return true; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  bool is_plain(eprosima::fastdds::dds::DataRepresentationId_t) const override { return false; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
  bool construct_sample(void* memory) const override {
    new (memory) ImuState();
    return true;
  }
#endif
};

// 手写 Motor_Command_12::motor_cmd 的 TopicDataType，用于发布控制命令。
// 下位机订阅的就是这个 IDL，因此不要改字段顺序或类型宽度。
class P1DdsInterface::CommandPubSubType final : public eprosima::fastdds::dds::TopicDataType {
 public:
  CommandPubSubType() {
    // 对应 motor_command_12.idl 中的 module/type。
    set_name("Motor_Command_12::motor_cmd");
    // 6 组 float[12] + 1 组 uint32[12]，再预留 CDR 对齐空间。
    max_serialized_type_size = 340;
    is_compute_key_provided = false;
  }

  bool serialize(const void* const data, eprosima::fastdds::rtps::SerializedPayload_t& payload,
                 eprosima::fastdds::dds::DataRepresentationId_t dataRepresentation) override {
    const auto* command = static_cast<const Command*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.max_size);
    eprosima::fastcdr::Cdr serializer(
        fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::CdrVersion::XCDRv1
            : eprosima::fastcdr::CdrVersion::XCDRv2);
    // encapsulation/encoding flag 是 DDS CDR 数据头，接收端靠它判断大小端和 XCDR 版本。
    payload.encapsulation = serializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    serializer.set_encoding_flag(
        dataRepresentation == eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION
            ? eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR
            : eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR2);

    try {
      serializer.serialize_encapsulation();
      // 字段顺序对应 motor_cmd.idl：kp, kd, pos, vel, torque, ff_torque, mode。
      serializeArray(serializer, command->kp);
      serializeArray(serializer, command->kd);
      serializeArray(serializer, command->position_desired);
      serializeArray(serializer, command->velocity_desired);
      serializeArray(serializer, command->torque);
      serializeArray(serializer, command->feedforward_torque);
      serializeArray(serializer, command->mode);
      serializer.set_dds_cdr_options({0, 0});
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    payload.length = static_cast<uint32_t>(serializer.get_serialized_data_length());
    return true;
  }

  bool deserialize(eprosima::fastdds::rtps::SerializedPayload_t& payload, void* data) override {
    auto* command = static_cast<Command*>(data);
    eprosima::fastcdr::FastBuffer fastBuffer(reinterpret_cast<char*>(payload.data), payload.length);
    eprosima::fastcdr::Cdr deserializer(fastBuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);

    try {
      // 先读 DDS CDR 数据头，再按 IDL 字段顺序解包 payload。
      deserializer.read_encapsulation();
      payload.encapsulation = deserializer.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
      deserializeArray(deserializer, command->kp);
      deserializeArray(deserializer, command->kd);
      deserializeArray(deserializer, command->position_desired);
      deserializeArray(deserializer, command->velocity_desired);
      deserializeArray(deserializer, command->torque);
      deserializeArray(deserializer, command->feedforward_torque);
      deserializeArray(deserializer, command->mode);
    } catch (eprosima::fastcdr::exception::Exception&) {
      return false;
    }

    return true;
  }

  uint32_t calculate_serialized_size(const void* const, eprosima::fastdds::dds::DataRepresentationId_t) override {
    return max_serialized_type_size;
  }

  bool compute_key(eprosima::fastdds::rtps::SerializedPayload_t&, eprosima::fastdds::rtps::InstanceHandle_t&,
                   bool = false) override {
    return false;
  }

  bool compute_key(const void* const, eprosima::fastdds::rtps::InstanceHandle_t&, bool = false) override {
    return false;
  }

  // Fast DDS 在 writer/reader 内部需要创建/释放样本对象时会调用 create_data/delete_data。
  void* create_data() override { return reinterpret_cast<void*>(new Command()); }

  void delete_data(void* data) override { delete reinterpret_cast<Command*>(data); }

  void register_type_object_representation() override {}

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  bool is_bounded() const override { return true; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  bool is_plain(eprosima::fastdds::dds::DataRepresentationId_t) const override { return false; }
#endif

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
  bool construct_sample(void* memory) const override {
    new (memory) Command();
    return true;
  }
#endif
};

// DDS reader 回调只缓存最新电机状态，实际写入 ros2_control 在 P1HW::read() 中完成。
class P1DdsInterface::MotorStateListener final : public eprosima::fastdds::dds::DataReaderListener {
 public:
  explicit MotorStateListener(P1DdsInterface& dds) : dds_(dds) {}

  void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
    // DDS 线程里只 take 样本并更新缓存，不直接碰 ros2_control 的 jointStates_。
    eprosima::fastdds::dds::SampleInfo info;
    MotorState state;
    // 一次回调里可能已经堆了多帧数据，循环取完，最终缓存最新一帧。
    while (reader->take_next_sample(&state, &info) == eprosima::fastdds::dds::RETCODE_OK) {
      if (info.valid_data) {
        dds_.updateMotorState(state);
      }
    }
  }

 private:
  P1DdsInterface& dds_;
};

// DDS reader 回调只缓存最新 IMU 状态，避免在 DDS 线程直接操作控制接口。
class P1DdsInterface::ImuStateListener final : public eprosima::fastdds::dds::DataReaderListener {
 public:
  explicit ImuStateListener(P1DdsInterface& dds) : dds_(dds) {}

  void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
    // IMU 也只在 DDS 回调中缓存，P1HW::read() 再同步到 state interface。
    eprosima::fastdds::dds::SampleInfo info;
    ImuState state;
    while (reader->take_next_sample(&state, &info) == eprosima::fastdds::dds::RETCODE_OK) {
      if (info.valid_data) {
        dds_.updateImuState(state);
      }
    }
  }

 private:
  P1DdsInterface& dds_;
};

// 构造函数放在 cpp 中定义，是因为类里持有指向内部 listener 类型的 unique_ptr。
P1DdsInterface::P1DdsInterface() = default;

P1DdsInterface::~P1DdsInterface() { shutdown(); }

// 创建同一个 DDS participant，并在其中订阅电机/IMU、发布电机命令。
// P1HW::on_init() 调用这个函数；失败时会清理已经创建的 DDS 实体。
bool P1DdsInterface::init(uint32_t domain, const std::string& stateTopic, const std::string& imuTopic,
                          const std::string& commandTopic) {
  // DomainParticipant 是 DDS 通讯的根对象，domain 必须和下位机/bridge 配置一致。
  auto* factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
  eprosima::fastdds::dds::DomainParticipantQos participantQos;
  participantQos.name("legged_p1_hw");
  participant_ = factory->create_participant(domain, participantQos);
  if (participant_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to create DDS participant");
    return false;
  }

  // 先注册三个 TopicDataType，再用注册后的 type name 创建 topic。
  stateType_ = eprosima::fastdds::dds::TypeSupport(new MotorStatePubSubType());
  imuType_ = eprosima::fastdds::dds::TypeSupport(new ImuStatePubSubType());
  commandType_ = eprosima::fastdds::dds::TypeSupport(new CommandPubSubType());
  stateType_.register_type(participant_);
  imuType_.register_type(participant_);
  commandType_.register_type(participant_);

  // topic 名来自 ros2_control hardware parameter，默认是 p1_motor_state/p1_imu/p1_motor_cmd。
  stateTopicHandle_ = participant_->create_topic(stateTopic, stateType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
  imuTopicHandle_ = participant_->create_topic(imuTopic, imuType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
  commandTopicHandle_ =
      participant_->create_topic(commandTopic, commandType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
  if (stateTopicHandle_ == nullptr || imuTopicHandle_ == nullptr || commandTopicHandle_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to create DDS topics");
    shutdown();
    return false;
  }

  // 同一个 participant 下创建一个 publisher 和一个 subscriber，分别负责命令和状态。
  publisher_ = participant_->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
  subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
  if (publisher_ == nullptr || subscriber_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to create DDS publisher/subscriber");
    shutdown();
    return false;
  }

  eprosima::fastdds::dds::DataWriterQos writerQos;
  // 控制命令只关心最新值，depth=1 可以减少旧命令排队造成的滞后。
  writerQos.history().depth = 1;
  commandWriter_ = publisher_->create_datawriter(commandTopicHandle_, writerQos);

  eprosima::fastdds::dds::DataReaderQos readerQos;
  // 状态也只保留最新帧；控制循环每次 read() 取缓存中的最近状态。
  readerQos.history().depth = 1;
  // listener 的生命周期必须覆盖 datareader，因此成员里用 unique_ptr 持有。
  stateListener_ = std::make_unique<MotorStateListener>(*this);
  imuListener_ = std::make_unique<ImuStateListener>(*this);
  stateReader_ = subscriber_->create_datareader(stateTopicHandle_, readerQos, stateListener_.get());
  imuReader_ = subscriber_->create_datareader(imuTopicHandle_, readerQos, imuListener_.get());
  if (commandWriter_ == nullptr || stateReader_ == nullptr || imuReader_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("legged_p1_hw"), "Failed to create DDS reader/writer");
    shutdown();
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("legged_p1_hw"), "DDS ready: state=%s imu=%s command=%s domain=%u", stateTopic.c_str(),
              imuTopic.c_str(), commandTopic.c_str(), domain);
  return true;
}

// 插件卸载时释放 DDS 实体，避免 controller_manager 退出时残留线程/句柄。
// delete_contained_entities() 会删除 participant 下的 topic、reader、writer、publisher/subscriber。
void P1DdsInterface::shutdown() {
  if (participant_ != nullptr) {
    participant_->delete_contained_entities();
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
    participant_ = nullptr;
  }
  imuListener_.reset();
  stateListener_.reset();
  stateReader_ = nullptr;
  imuReader_ = nullptr;
  commandWriter_ = nullptr;
  stateTopicHandle_ = nullptr;
  imuTopicHandle_ = nullptr;
  commandTopicHandle_ = nullptr;
  publisher_ = nullptr;
  subscriber_ = nullptr;
}

bool P1DdsInterface::getLatestMotorState(MotorState& state) const {
  // 第一次收到 DDS 数据前返回 false，P1HW::read() 会保持默认状态。
  if (!hasState_.load()) {
    return false;
  }

  // 返回一份拷贝，避免控制线程在使用状态时被 DDS 回调线程改写。
  std::lock_guard<std::mutex> lock(stateMutex_);
  state = latestState_;
  return true;
}

bool P1DdsInterface::getLatestImuState(ImuState& state) const {
  // 第一次收到 IMU 前返回 false，P1HW::read() 会继续使用默认 IMU 值。
  if (!hasImuState_.load()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(stateMutex_);
  state = latestImuState_;
  return true;
}

bool P1DdsInterface::writeCommand(const Command& command) {
  // writer 尚未创建时不能发送，通常说明 init() 没有成功。
  if (commandWriter_ == nullptr) {
    return false;
  }

  // Fast DDS write() 接口接收非 const 指针，所以这里复制一份再交给 writer。
  Command commandCopy = command;
  return commandWriter_->write(&commandCopy) == eprosima::fastdds::dds::RETCODE_OK;
}

// DDS 回调线程入口：保存最新电机状态帧。
// 这个函数只更新缓存和标志位，保持回调轻量，避免阻塞 DDS 接收线程。
void P1DdsInterface::updateMotorState(const MotorState& state) {
  {
    std::lock_guard<std::mutex> lock(stateMutex_);
    latestState_ = state;
  }
  hasState_.store(true);
}

// DDS 回调线程入口：保存最新 IMU 状态帧。
// 和电机状态共用同一把锁，保证 P1HW::read() 拿到的是完整结构体拷贝。
void P1DdsInterface::updateImuState(const ImuState& state) {
  {
    std::lock_guard<std::mutex> lock(stateMutex_);
    latestImuState_ = state;
  }
  hasImuState_.store(true);
}

}  // namespace legged
