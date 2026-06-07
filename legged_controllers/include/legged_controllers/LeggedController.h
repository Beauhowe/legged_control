//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/bool.hpp>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::ControllerInterface {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> contactNames_;
  std::string imuName_{"base_imu"};

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergencyStopSubscriber_;
  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::executors::SingleThreadedExecutor rosExecutor_;
  std::thread rosSpinThread_;

 private:
  void starting(const rclcpp::Time& time);
  void resyncMpcAfterEmergencyStop();
  void setHybridJointCommand(size_t jointIndex, scalar_t posDes, scalar_t velDes, scalar_t kp, scalar_t kd, scalar_t ff);
  double getStateInterfaceValue(const std::string& interfaceName) const;

  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  std::atomic_bool mpcAdvancePaused_{false};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  std::atomic_bool emergencyStopActive_{false};
  bool emergencyStopLogged_{false};
  bool previousEmergencyStopActive_{false};
  rclcpp::Time controllerStartTime_{0, 0, RCL_ROS_TIME};
  bool controllerClockStarted_{false};
  scalar_t mrtPeriod_{0.001};
  bool mrtPeriodMismatchWarned_{false};
};


class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
