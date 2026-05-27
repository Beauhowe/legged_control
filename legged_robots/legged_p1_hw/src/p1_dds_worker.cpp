#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include "legged_p1_hw/P1DdsInterface.h"
#include "imu_topic_base.hpp"
#include "imu_topic_basePubSubTypes.hpp"
#include "motor_command_12.hpp"
#include "motor_command_12PubSubTypes.hpp"
#include "motor_state_12.hpp"
#include "motor_state_12PubSubTypes.hpp"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint32_t kPacketMotorState = 1;
constexpr uint32_t kPacketImuState = 2;
std::atomic<bool> running{true};

struct StatePacket {
  uint32_t tag{kPacketMotorState};
  legged::P1DdsInterface::MotorState motor_state{};
  legged::P1DdsInterface::ImuState imu_state{};
};

void signalHandler(int) {
  _exit(0);
}

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

bool writeAll(int fd, const void* data, size_t size) {
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

class P1DdsWorker final {
 public:
  P1DdsWorker(int stateFd, int commandFd, uint32_t domain, std::string stateTopic, std::string imuTopic,
              std::string commandTopic)
      : stateFd_(stateFd), commandFd_(commandFd), domain_(domain), stateTopic_(std::move(stateTopic)),
        imuTopic_(std::move(imuTopic)), commandTopic_(std::move(commandTopic)) {}

  ~P1DdsWorker() { shutdown(); }

  bool init() {
    auto* factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    eprosima::fastdds::dds::DomainParticipantQos participantQos;
    participantQos.name("p1_dds_worker");
    participant_ = factory->create_participant(domain_, participantQos);
    if (participant_ == nullptr) {
      std::fprintf(stderr, "p1_dds_worker: failed to create participant\n");
      return false;
    }

    stateType_ = eprosima::fastdds::dds::TypeSupport(new Motor_State_12::motor_statePubSubType());
    imuType_ = eprosima::fastdds::dds::TypeSupport(new imu_topic_base::IMUDataPubSubType());
    commandType_ = eprosima::fastdds::dds::TypeSupport(new Motor_Command_12::motor_cmdPubSubType());
    stateType_.register_type(participant_);
    imuType_.register_type(participant_);
    commandType_.register_type(participant_);

    stateTopicHandle_ = participant_->create_topic(stateTopic_, stateType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    imuTopicHandle_ = participant_->create_topic(imuTopic_, imuType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    commandTopicHandle_ = participant_->create_topic(commandTopic_, commandType_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    publisher_ = participant_->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
    subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
    if (stateTopicHandle_ == nullptr || imuTopicHandle_ == nullptr || commandTopicHandle_ == nullptr || publisher_ == nullptr ||
        subscriber_ == nullptr) {
      std::fprintf(stderr, "p1_dds_worker: failed to create DDS topics/entities\n");
      return false;
    }

    eprosima::fastdds::dds::DataWriterQos writerQos;
    writerQos.history().depth = 1;
    commandWriter_ = publisher_->create_datawriter(commandTopicHandle_, writerQos);

    eprosima::fastdds::dds::DataReaderQos readerQos;
    readerQos.history().depth = 1;
    stateListener_ = std::make_unique<StateListener>(*this);
    imuListener_ = std::make_unique<ImuListener>(*this);
    stateReader_ = subscriber_->create_datareader(stateTopicHandle_, readerQos, stateListener_.get());
    imuReader_ = subscriber_->create_datareader(imuTopicHandle_, readerQos, imuListener_.get());
    if (commandWriter_ == nullptr || stateReader_ == nullptr || imuReader_ == nullptr) {
      std::fprintf(stderr, "p1_dds_worker: failed to create DDS reader/writer\n");
      return false;
    }

    commandThread_ = std::thread([this] { commandLoop(); });
    std::fprintf(stderr, "p1_dds_worker: DDS ok (state=%s imu=%s command=%s domain=%u)\n", stateTopic_.c_str(),
                 imuTopic_.c_str(), commandTopic_.c_str(), domain_);
    return true;
  }

  void spin() const {
    while (running) {
      ::pause();
    }
  }

 private:
  class StateListener final : public eprosima::fastdds::dds::DataReaderListener {
   public:
    explicit StateListener(P1DdsWorker& worker) : worker_(worker) {}

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
      eprosima::fastdds::dds::SampleInfo info;
      Motor_State_12::motor_state data;
      while (reader->take_next_sample(&data, &info) == eprosima::fastdds::dds::RETCODE_OK) {
        if (!info.valid_data) {
          continue;
        }
        StatePacket packet;
        packet.tag = kPacketMotorState;
        packet.motor_state.motor_id = data.motorId();
        packet.motor_state.last_seen_time = data.lastSeenTime();
        packet.motor_state.status = data.status();
        packet.motor_state.position = data.position();
        packet.motor_state.speed = data.speed();
        packet.motor_state.current = data.current();
        packet.motor_state.motor_temperature = data.motorTemperature();
        packet.motor_state.mos_temperature = data.mosTemperature();
        packet.motor_state.is_online = data.isonline();
        if (!writeAll(worker_.stateFd_, &packet, sizeof(packet))) {
          running = false;
          break;
        }
      }
    }

   private:
    P1DdsWorker& worker_;
  };

  class ImuListener final : public eprosima::fastdds::dds::DataReaderListener {
   public:
    explicit ImuListener(P1DdsWorker& worker) : worker_(worker) {}

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
      eprosima::fastdds::dds::SampleInfo info;
      imu_topic_base::IMUData data;
      while (reader->take_next_sample(&data, &info) == eprosima::fastdds::dds::RETCODE_OK) {
        if (!info.valid_data) {
          continue;
        }
        StatePacket packet;
        packet.tag = kPacketImuState;
        packet.imu_state.orientation_w = data.orientation_w();
        packet.imu_state.orientation_x = data.orientation_x();
        packet.imu_state.orientation_y = data.orientation_y();
        packet.imu_state.orientation_z = data.orientation_z();
        packet.imu_state.angular_velocity_x = data.angular_velocity_x();
        packet.imu_state.angular_velocity_y = data.angular_velocity_y();
        packet.imu_state.angular_velocity_z = data.angular_velocity_z();
        packet.imu_state.linear_acceleration_x = data.linear_acceleration_x();
        packet.imu_state.linear_acceleration_y = data.linear_acceleration_y();
        packet.imu_state.linear_acceleration_z = data.linear_acceleration_z();
        packet.imu_state.roll = data.roll();
        packet.imu_state.pitch = data.pitch();
        packet.imu_state.yaw = data.yaw();
        if (!writeAll(worker_.stateFd_, &packet, sizeof(packet))) {
          running = false;
          break;
        }
      }
    }

   private:
    P1DdsWorker& worker_;
  };

  void commandLoop() {
    while (running) {
      legged::P1DdsInterface::Command command;
      if (!readExact(commandFd_, &command, sizeof(command))) {
        break;
      }
      Motor_Command_12::motor_cmd ddsCommand;
      ddsCommand.kp(command.kp);
      ddsCommand.kd(command.kd);
      ddsCommand.pos(command.position_desired);
      ddsCommand.vel(command.velocity_desired);
      ddsCommand.torque(command.torque);
      ddsCommand.ff_torque(command.feedforward_torque);
      ddsCommand.mode(command.mode);
      commandWriter_->write(&ddsCommand);
    }
    running = false;
  }

  void shutdown() {
    running = false;
    if (commandFd_ >= 0) {
      ::close(commandFd_);
      commandFd_ = -1;
    }
    if (stateFd_ >= 0) {
      ::close(stateFd_);
      stateFd_ = -1;
    }
    if (commandThread_.joinable()) {
      commandThread_.join();
    }
    if (participant_ != nullptr) {
      participant_->delete_contained_entities();
      eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
      participant_ = nullptr;
    }
  }

  int stateFd_{-1};
  int commandFd_{-1};
  uint32_t domain_{0};
  std::string stateTopic_;
  std::string imuTopic_;
  std::string commandTopic_;
  std::thread commandThread_;
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
  std::unique_ptr<StateListener> stateListener_;
  std::unique_ptr<ImuListener> imuListener_;
};

}  // namespace

int main(int argc, char** argv) {
  int stateFd = -1;
  int commandFd = -1;
  uint32_t domain = 0;
  std::string stateTopic = "p1_motor_state";
  std::string imuTopic = "p1_imu";
  std::string commandTopic = "p1_motor_cmd";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--state-fd" && i + 1 < argc) {
      stateFd = std::atoi(argv[++i]);
    } else if (arg == "--command-fd" && i + 1 < argc) {
      commandFd = std::atoi(argv[++i]);
    } else if (arg == "--dds-domain" && i + 1 < argc) {
      domain = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--state-topic" && i + 1 < argc) {
      stateTopic = argv[++i];
    } else if (arg == "--imu-topic" && i + 1 < argc) {
      imuTopic = argv[++i];
    } else if (arg == "--command-topic" && i + 1 < argc) {
      commandTopic = argv[++i];
    }
  }

  if (stateFd < 0 || commandFd < 0) {
    std::fprintf(stderr, "usage: %s --state-fd <fd> --command-fd <fd> [--dds-domain id] [--state-topic name] [--imu-topic name] [--command-topic name]\n",
                 argv[0]);
    return 1;
  }

  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  P1DdsWorker worker(stateFd, commandFd, domain, stateTopic, imuTopic, commandTopic);
  if (!worker.init()) {
    return 1;
  }
  worker.spin();
  return 0;
}
