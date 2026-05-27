#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include "legged_p1_hw/P1GaitIpc.h"
#include "p1_gait.hpp"
#include "p1_gaitPubSubTypes.hpp"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};

void signalHandler(int) {
  running = false;
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

class GaitDdsWorker final {
 public:
  GaitDdsWorker(int outFd, uint32_t domain, std::string topic) : outFd_(outFd), domain_(domain), topic_(std::move(topic)) {}

  ~GaitDdsWorker() { shutdown(); }

  bool init() {
    auto* factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    eprosima::fastdds::dds::DomainParticipantQos participantQos;
    participantQos.name("p1_gait_dds_worker");
    participant_ = factory->create_participant(domain_, participantQos);
    if (participant_ == nullptr) {
      std::fprintf(stderr, "p1_gait_dds_worker: failed to create participant\n");
      return false;
    }

    type_ = eprosima::fastdds::dds::TypeSupport(new P1_Gait::gait_commandPubSubType());
    type_.register_type(participant_);
    topicHandle_ = participant_->create_topic(topic_, type_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
    if (topicHandle_ == nullptr || subscriber_ == nullptr) {
      std::fprintf(stderr, "p1_gait_dds_worker: failed to create topic/subscriber\n");
      return false;
    }

    eprosima::fastdds::dds::DataReaderQos readerQos;
    readerQos.history().depth = 1;
    listener_ = std::make_unique<Listener>(*this);
    reader_ = subscriber_->create_datareader(topicHandle_, readerQos, listener_.get());
    if (reader_ == nullptr) {
      std::fprintf(stderr, "p1_gait_dds_worker: failed to create reader\n");
      return false;
    }

    std::fprintf(stderr, "p1_gait_dds_worker: DDS ok (%s, domain=%u)\n", topic_.c_str(), domain_);
    return true;
  }

  void spin() const {
    while (running) {
      ::pause();
    }
  }

 private:
  class Listener final : public eprosima::fastdds::dds::DataReaderListener {
   public:
    explicit Listener(GaitDdsWorker& worker) : worker_(worker) {}

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override {
      eprosima::fastdds::dds::SampleInfo info;
      P1_Gait::gait_command command;
      while (reader->take_next_sample(&command, &info) == eprosima::fastdds::dds::RETCODE_OK) {
        if (!info.valid_data) {
          continue;
        }

        legged_p1_hw::P1GaitCommandPacket packet;
        packet.stamp = command.stamp();
        packet.gait_id = command.gait_id();
        packet.emergency_stop = command.emergencyStop() ? 1 : 0;
        packet.trigger = command.trigger() ? 1 : 0;
        packet.velocity_x = command.velocity_x();
        packet.velocity_y = command.velocity_y();
        packet.yaw_rate = command.yaw_rate();
        if (!writeAll(worker_.outFd_, &packet, sizeof(packet))) {
          running = false;
          break;
        }
      }
    }

   private:
    GaitDdsWorker& worker_;
  };

  void shutdown() {
    if (participant_ != nullptr) {
      participant_->delete_contained_entities();
      eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
      participant_ = nullptr;
    }
    listener_.reset();
    reader_ = nullptr;
    topicHandle_ = nullptr;
    subscriber_ = nullptr;
  }

  int outFd_{-1};
  uint32_t domain_{0};
  std::string topic_;
  eprosima::fastdds::dds::DomainParticipant* participant_{nullptr};
  eprosima::fastdds::dds::Subscriber* subscriber_{nullptr};
  eprosima::fastdds::dds::Topic* topicHandle_{nullptr};
  eprosima::fastdds::dds::DataReader* reader_{nullptr};
  eprosima::fastdds::dds::TypeSupport type_;
  std::unique_ptr<Listener> listener_;
};

}  // namespace

int main(int argc, char** argv) {
  int outFd = -1;
  uint32_t domain = 0;
  std::string topic = "p1_gait_command";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--out-fd" && i + 1 < argc) {
      outFd = std::atoi(argv[++i]);
    } else if (arg == "--dds-domain" && i + 1 < argc) {
      domain = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--dds-gait-topic" && i + 1 < argc) {
      topic = argv[++i];
    }
  }

  if (outFd < 0) {
    std::fprintf(stderr, "usage: %s --out-fd <fd> [--dds-domain id] [--dds-gait-topic topic]\n", argv[0]);
    return 1;
  }

  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  GaitDdsWorker worker(outFd, domain, topic);
  if (!worker.init()) {
    return 1;
  }
  worker.spin();
  return 0;
}
