#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include "legged_p1_hw/P1GaitIpc.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

const std::map<std::string, int>& modeNameToInt() {
  static const std::map<std::string, int> modes{
      {"FLY", 0},       {"RH", 1},        {"LH", 2},        {"LH_RH", 3},
      {"RF", 4},        {"RF_RH", 5},     {"RF_LH", 6},     {"RF_LH_RH", 7},
      {"LF", 8},        {"LF_RH", 9},     {"LF_LH", 10},    {"LF_LH_RH", 11},
      {"LF_RF", 12},    {"LF_RF_RH", 13}, {"LF_RF_LH", 14}, {"STANCE", 15},
  };
  return modes;
}

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open " + path);
  }
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

std::string trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\n\r");
  return value.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCommaSeparated(const std::string& value) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto name = trim(item);
    if (!name.empty()) {
      result.push_back(name);
    }
  }
  return result;
}

std::vector<int> splitCommaSeparatedInts(const std::string& value) {
  std::vector<int> result;
  for (const auto& item : splitCommaSeparated(value)) {
    result.push_back(std::stoi(item));
  }
  return result;
}

std::string extractBracedBlock(const std::string& text, const std::string& key) {
  size_t searchFrom = 0;
  while (searchFrom < text.size()) {
    const size_t keyPos = text.find(key, searchFrom);
    if (keyPos == std::string::npos) {
      return "";
    }

    const size_t lineStart = text.rfind('\n', keyPos);
    const size_t prefixStart = lineStart == std::string::npos ? 0 : lineStart + 1;
    const auto prefix = trim(text.substr(prefixStart, keyPos - prefixStart));
    const size_t afterKey = keyPos + key.size();
    const bool hasTokenBoundary = afterKey >= text.size() ||
                                  !(std::isalnum(static_cast<unsigned char>(text[afterKey])) || text[afterKey] == '_');
    if (!prefix.empty() || !hasTokenBoundary) {
      searchFrom = afterKey;
      continue;
    }

    size_t brace = afterKey;
    while (brace < text.size() && std::isspace(static_cast<unsigned char>(text[brace]))) {
      ++brace;
    }
    if (brace >= text.size() || text[brace] != '{') {
      searchFrom = afterKey;
      continue;
    }

    int depth = 0;
    for (size_t i = brace; i < text.size(); ++i) {
      if (text[i] == '{') {
        ++depth;
      } else if (text[i] == '}') {
        --depth;
        if (depth == 0) {
          return text.substr(brace + 1, i - brace - 1);
        }
      }
    }
    return "";
  }
  return "";
}

std::vector<std::string> parseIndexedNames(const std::string& block) {
  std::vector<std::string> values;
  std::istringstream stream(block);
  std::string line;
  while (std::getline(stream, line)) {
    const auto bracket = line.find(']');
    if (bracket == std::string::npos) {
      continue;
    }
    std::istringstream valueStream(line.substr(bracket + 1));
    std::string value;
    if (valueStream >> value) {
      values.push_back(value);
    }
  }
  return values;
}

std::vector<double> parseIndexedDoubles(const std::string& block) {
  std::vector<double> values;
  std::istringstream stream(block);
  std::string line;
  while (std::getline(stream, line)) {
    const auto bracket = line.find(']');
    if (bracket == std::string::npos) {
      continue;
    }
    std::istringstream valueStream(line.substr(bracket + 1));
    double value = 0.0;
    if (valueStream >> value) {
      values.push_back(value);
    }
  }
  return values;
}

std::vector<int8_t> parseModeSequence(const std::string& gaitBlock) {
  std::vector<int8_t> modes;
  for (const auto& token : parseIndexedNames(extractBracedBlock(gaitBlock, "modeSequence"))) {
    const auto modeIt = modeNameToInt().find(token);
    if (modeIt == modeNameToInt().end()) {
      throw std::runtime_error("unknown gait mode token: " + token);
    }
    modes.push_back(static_cast<int8_t>(modeIt->second));
  }
  return modes;
}


std::map<std::string, ocs2_msgs::msg::ModeSchedule> loadGaits(const std::string& path) {
  const auto content = readFile(path);
  std::map<std::string, ocs2_msgs::msg::ModeSchedule> gaits;
  for (const auto& name : parseIndexedNames(extractBracedBlock(content, "list"))) {
    const auto gaitBlock = extractBracedBlock(content, name);
    if (gaitBlock.empty()) {
      continue;
    }

    ocs2_msgs::msg::ModeSchedule msg;
    msg.event_times = parseIndexedDoubles(extractBracedBlock(gaitBlock, "switchingTimes"));
    msg.mode_sequence = parseModeSequence(gaitBlock);
    if (!msg.event_times.empty() && !msg.mode_sequence.empty()) {
      gaits[name] = msg;
    }
  }
  return gaits;
}

ocs2_msgs::msg::ModeSchedule loadDefaultModeSequenceTemplate(const std::string& path) {
  const auto content = readFile(path);
  const auto block = extractBracedBlock(content, "defaultModeSequenceTemplate");
  if (block.empty()) {
    throw std::runtime_error("missing defaultModeSequenceTemplate in " + path);
  }

  ocs2_msgs::msg::ModeSchedule msg;
  msg.event_times = parseIndexedDoubles(extractBracedBlock(block, "switchingTimes"));
  msg.mode_sequence = parseModeSequence(block);
  if (msg.event_times.empty() || msg.mode_sequence.empty()) {
    throw std::runtime_error("invalid defaultModeSequenceTemplate in " + path);
  }
  return msg;
}

std::vector<ocs2_msgs::msg::ModeSchedule> selectGaitsById(const std::map<std::string, ocs2_msgs::msg::ModeSchedule>& gaitMap,
                                                          const std::vector<std::string>& gaitIdMapping) {
  std::vector<ocs2_msgs::msg::ModeSchedule> gaits;
  gaits.reserve(gaitIdMapping.size());
  for (const auto& name : gaitIdMapping) {
    const auto it = gaitMap.find(name);
    if (it == gaitMap.end()) {
      throw std::runtime_error("gait_id_mapping references unknown gait: " + name);
    }
    gaits.push_back(it->second);
  }
  return gaits;
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

bool isZeroTwist(const geometry_msgs::msg::Twist& twist) {
  return twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0 && twist.angular.x == 0.0 &&
         twist.angular.y == 0.0 && twist.angular.z == 0.0;
}

}  // namespace

class P1GaitDdsBridge final : public rclcpp::Node {
 public:
  P1GaitDdsBridge() : Node("p1_gait_dds_bridge") {
    const auto domain = static_cast<uint32_t>(declare_parameter<int>("dds_domain", 0));
    const auto gaitTopic = declare_parameter<std::string>("dds_gait_topic", "p1_gait_command");
    const auto gaitFile = declare_parameter<std::string>("gait_file", "");
    const auto gaitIdMapping = declare_parameter<std::string>("gait_id_mapping", "stance,trot,standing_trot,pace,static_walk,lie_down");
    const auto cmdVelTopic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto emergencyStopTopic = declare_parameter<std::string>("emergency_stop_topic", "/p1_emergency_stop");
    const auto robotName = declare_parameter<std::string>("robot_name", "legged_robot");
    standGaitId_ = declare_parameter<int>("stand_gait_id", 0);
    lieDownGaitId_ = declare_parameter<int>("lie_down_gait_id", 5);
    emergencyResetModeSchedule_ = declare_parameter<bool>("emergency_reset_mode_schedule", false);
    auto mpcReferenceFile = declare_parameter<std::string>("mpc_reference_file", "");
    const auto locomotionGaitIds = declare_parameter<std::string>("locomotion_gait_ids", "1");
    locomotionGaitIds_ = splitCommaSeparatedInts(locomotionGaitIds);
    auto modeScheduleTopic = declare_parameter<std::string>("mode_schedule_topic", "");
    auto postureCommandTopic = declare_parameter<std::string>("posture_command_topic", "");
    if (modeScheduleTopic.empty()) {
      modeScheduleTopic = "/" + robotName + "_mpc_mode_schedule";
    }
    if (postureCommandTopic.empty()) {
      postureCommandTopic = "/" + robotName + "_posture_command";
    }

    if (!mpcReferenceFile.empty()) {
      defaultModeSequenceTemplate_ = loadDefaultModeSequenceTemplate(mpcReferenceFile);
      hasDefaultModeSequenceTemplate_ = true;
      mpcReferenceFile_ = mpcReferenceFile;
    }

    const auto gaitMap = loadGaits(gaitFile);
    gaits_ = selectGaitsById(gaitMap, splitCommaSeparated(gaitIdMapping));
    if (gaits_.empty()) {
      throw std::runtime_error("no valid gait_id_mapping entries loaded from " + gaitFile);
    }

    cmdVelPublisher_ = create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
    emergencyStopPublisher_ = create_publisher<std_msgs::msg::Bool>(emergencyStopTopic, rclcpp::QoS(1).reliable().transient_local());
    auto modeQos = rclcpp::QoS(1).reliable().transient_local();
    modeSchedulePublisher_ = create_publisher<ocs2_msgs::msg::ModeSchedule>(modeScheduleTopic, modeQos);
    // 固定姿态命令(Int8: 1=STAND, 2=LIE_DOWN, 0=解除)。stance/lie_down 不再发 mode_schedule/target，改发此话题给控制器。
    postureCommandPublisher_ = create_publisher<std_msgs::msg::Int8>(postureCommandTopic, modeQos);

    startDdsWorker(domain, gaitTopic);
    RCLCPP_INFO(get_logger(),
                "P1 gait DDS bridge ready: dds=%s domain=%u cmd_vel=%s emergency_stop=%s mode_schedule=%s posture_command=%s "
                "gait_id_mapping=%s emergency_reset_mode_schedule=%s mpc_reference=%s",
                gaitTopic.c_str(), domain, cmdVelTopic.c_str(), emergencyStopTopic.c_str(), modeScheduleTopic.c_str(),
                postureCommandTopic.c_str(), gaitIdMapping.c_str(),
                emergencyResetModeSchedule_ ? "true" : "false", mpcReferenceFile_.c_str());
  }

  ~P1GaitDdsBridge() override { stopDdsWorker(); }

 private:
  void startDdsWorker(uint32_t domain, const std::string& gaitTopic) {
    int pipeFds[2];
    if (::pipe(pipeFds) != 0) {
      throw std::runtime_error("failed to create DDS worker pipe");
    }

    workerPid_ = ::fork();
    if (workerPid_ < 0) {
      ::close(pipeFds[0]);
      ::close(pipeFds[1]);
      throw std::runtime_error("failed to fork DDS worker");
    }

    if (workerPid_ == 0) {
      ::close(pipeFds[0]);
      const auto prefix = ament_index_cpp::get_package_prefix("legged_p1_hw");
      const auto workerPath = prefix + "/lib/legged_p1_hw/p1_gait_dds_worker";
      const auto outFd = std::to_string(pipeFds[1]);
      const auto domainText = std::to_string(domain);
      const std::string fastDdsLib = P1_HW_FASTDDS_LIB_DIR;
      const char* oldPath = std::getenv("LD_LIBRARY_PATH");
      const auto newPath = oldPath == nullptr ? fastDdsLib : fastDdsLib + ":" + oldPath;
      ::setenv("LD_LIBRARY_PATH", newPath.c_str(), 1);
      ::execl(workerPath.c_str(), workerPath.c_str(), "--out-fd", outFd.c_str(), "--dds-domain", domainText.c_str(),
              "--dds-gait-topic", gaitTopic.c_str(), static_cast<char*>(nullptr));
      std::perror("execl p1_gait_dds_worker");
      _exit(127);
    }

    ::close(pipeFds[1]);
    workerReadFd_ = pipeFds[0];
    workerRunning_ = true;
    workerThread_ = std::thread([this] { workerReadLoop(); });
  }

  void stopDdsWorker() {
    workerRunning_ = false;
    if (workerReadFd_ >= 0) {
      ::close(workerReadFd_);
      workerReadFd_ = -1;
    }
    if (workerThread_.joinable()) {
      workerThread_.join();
    }
    if (workerPid_ > 0) {
      ::kill(workerPid_, SIGTERM);
      ::waitpid(workerPid_, nullptr, 0);
      workerPid_ = -1;
    }
  }

  void workerReadLoop() {
    while (workerRunning_) {
      legged_p1_hw::P1GaitCommandPacket packet;
      if (!readExact(workerReadFd_, &packet, sizeof(packet))) {
        break;
      }
      handleCommand(packet);
    }
  }

  // 发布固定姿态命令(1=STAND, 2=LIE_DOWN, 0=解除)。控制器收到后绕过 MPC，直接把关节拉向 reference.info 里的固定角。
  void publishPostureCommand(int8_t posture, const char* name) {
    std_msgs::msg::Int8 msg;
    msg.data = posture;
    postureCommandPublisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published posture command: %s (%d)", name, static_cast<int>(posture));
  }

  void handleCommand(const legged_p1_hw::P1GaitCommandPacket& command) {
    const bool emergencyStop = command.emergency_stop != 0;
    std_msgs::msg::Bool stop;
    stop.data = emergencyStop;
    emergencyStopPublisher_->publish(stop);

    if (emergencyStop) {
      if (!emergencyStopActive_) {
        emergencyStopActive_ = true;
        activeGaitId_ = -1;
        lastTrigger_ = false;
        if (emergencyResetModeSchedule_ && hasDefaultModeSequenceTemplate_) {
          modeSchedulePublisher_->publish(defaultModeSequenceTemplate_);
          RCLCPP_WARN(get_logger(),
                      "Emergency stop active: active_gait_id=-1, reset MPC to defaultModeSequenceTemplate from %s.",
                      mpcReferenceFile_.c_str());
        } else {
          RCLCPP_WARN(get_logger(),
                      "Emergency stop active: active_gait_id=-1, no mode_schedule published (same as startup before gait "
                      "selection). Re-select gait after emergency_stop=false.");
        }
      }
      if (!lastCmdZero_) {
        cmdVelPublisher_->publish(geometry_msgs::msg::Twist());
        lastCmdZero_ = true;
      }
      lastTrigger_ = command.trigger != 0;
      return;
    }

    const bool trigger = command.trigger != 0;
    const bool clearingEmergency = emergencyStopActive_ && !emergencyStop;
    if (clearingEmergency) {
      emergencyStopActive_ = false;
      lastTrigger_ = false;
      if (!trigger) {
        activeGaitId_ = -1;
        RCLCPP_INFO(get_logger(), "Emergency stop released; active_gait_id=-1, re-select gait to resume.");
      }
    }

    if (trigger && !lastTrigger_) {
      if (command.gait_id >= gaits_.size()) {
        RCLCPP_WARN(get_logger(), "Ignore gait_id=%u, valid range is [0, %zu)", command.gait_id, gaits_.size());
      } else {
        activeGaitId_ = static_cast<int>(command.gait_id);
        if (activeGaitId_ == standGaitId_) {
          // 固定姿态(站立)：绕过 MPC，只发姿态命令，不发 mode_schedule。
          publishPostureCommand(1, "stand");
        } else if (activeGaitId_ == lieDownGaitId_) {
          // 固定姿态(趴下)：绕过 MPC，只发姿态命令，不发 mode_schedule。
          publishPostureCommand(2, "lie_down");
        } else {
          // 运动步态：解除固定姿态(发 0)并发布 mode_schedule 启动 MPC。
          publishPostureCommand(0, "none");
          modeSchedulePublisher_->publish(gaits_[command.gait_id]);
          RCLCPP_INFO(get_logger(), "Published gait_id=%u", command.gait_id);
        }
      }
    }

    geometry_msgs::msg::Twist twist;
    const bool velocityEnabled = std::find(locomotionGaitIds_.begin(), locomotionGaitIds_.end(), activeGaitId_) != locomotionGaitIds_.end();
    if (velocityEnabled) {
      twist.linear.x = command.velocity_x;
      twist.linear.y = command.velocity_y;
      twist.angular.z = command.yaw_rate;
    }

    // Re-publishing zero velocity continuously makes legged_target_trajectories_publisher
    // reset the target pose. Publish zero once to stop, then hold until non-zero command.
    const bool isZero = isZeroTwist(twist);
    if (isZero && lastCmdZero_) {
      lastTrigger_ = trigger;
      return;
    }
    cmdVelPublisher_->publish(twist);
    lastCmdZero_ = isZero;
    lastTrigger_ = trigger;
  }

  std::vector<ocs2_msgs::msg::ModeSchedule> gaits_;
  ocs2_msgs::msg::ModeSchedule defaultModeSequenceTemplate_;
  std::string mpcReferenceFile_;
  bool hasDefaultModeSequenceTemplate_{false};
  bool emergencyResetModeSchedule_{false};
  bool emergencyStopActive_{false};
  int standGaitId_{0};
  int lieDownGaitId_{5};
  int activeGaitId_{-1};
  std::vector<int> locomotionGaitIds_{1};
  bool lastTrigger_{false};
  bool lastCmdZero_{true};
  std::atomic<bool> workerRunning_{false};
  int workerReadFd_{-1};
  pid_t workerPid_{-1};
  std::thread workerThread_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyStopPublisher_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeSchedulePublisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr postureCommandPublisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<P1GaitDdsBridge>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("p1_gait_dds_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
