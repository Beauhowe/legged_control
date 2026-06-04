cmake_minimum_required(VERSION 3.5)
project(legged_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(grid_map_sdf REQUIRED)
find_package(convex_plane_decomposition REQUIRED)

add_library(${PROJECT_NAME}
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/constraint/SwingTrajectoryPlanner.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/SwitchedModelReferenceManager.cpp
  src/LeggedRobotPreComputation.cpp
  src/LeggedInterface.cpp
  src/perceptive/ConvexRegionSelector.cpp
  src/perceptive/PerceptiveLeggedReferenceManager.cpp
  src/perceptive/PerceptiveLeggedPrecomputation.cpp
  src/perceptive/constraint/FootPlacementConstraint.cpp
  src/perceptive/constraint/FootCollisionConstraint.cpp
  src/perceptive/constraint/SphereSdfConstraint.cpp
  src/perceptive/PerceptiveLeggedInterface.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Boost::filesystem
  Boost::system
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_self_collision
  ocs2_sphere_approximation
  convex_plane_decomposition
  grid_map_sdf
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_self_collision
  ocs2_sphere_approximation
  convex_plane_decomposition
  grid_map_sdf
  pinocchio
  Eigen3
  Boost
  PCL
)

ament_package()
