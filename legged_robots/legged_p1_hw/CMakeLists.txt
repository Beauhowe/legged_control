cmake_minimum_required(VERSION 3.10)
project(legged_p1_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(legged_hw REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(FASTDDS_INSTALL_PREFIX "" CACHE PATH "Fast-DDS install prefix")
if(NOT FASTDDS_INSTALL_PREFIX AND DEFINED ENV{FASTDDS_INSTALL_PREFIX})
  set(FASTDDS_INSTALL_PREFIX "$ENV{FASTDDS_INSTALL_PREFIX}" CACHE PATH "Fast-DDS install prefix" FORCE)
endif()
if(NOT FASTDDS_INSTALL_PREFIX)
  set(_fdd_guess "${CMAKE_CURRENT_SOURCE_DIR}/../../../env/fast_dds/local")
  if(EXISTS "${_fdd_guess}/include/fastdds/dds/domain/DomainParticipant.hpp")
    set(FASTDDS_INSTALL_PREFIX "${_fdd_guess}" CACHE PATH "Fast-DDS install prefix" FORCE)
  endif()
endif()
if(NOT FASTDDS_INSTALL_PREFIX)
  message(FATAL_ERROR "Please set FASTDDS_INSTALL_PREFIX, e.g. /workspace/src/env/fast_dds/local")
endif()

find_library(P1_HW_FASTCDR_LIB NAMES fastcdr REQUIRED NO_DEFAULT_PATH HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
find_library(P1_HW_FASTDDS_LIB NAMES fastdds REQUIRED NO_DEFAULT_PATH HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")
find_library(P1_HW_FOONATHAN_MEMORY_LIB NAMES foonathan_memory-0.7.3 foonathan_memory REQUIRED NO_DEFAULT_PATH HINTS "${FASTDDS_INSTALL_PREFIX}/lib" "${FASTDDS_INSTALL_PREFIX}/lib64")

add_library(${PROJECT_NAME} SHARED
  src/P1ContactEstimator.cpp
  src/P1DdsInterface.cpp
  src/P1HW.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  legged_hw
  ocs2_msgs
  pluginlib
  rclcpp
  std_msgs
  Eigen3
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    P1_HW_FASTDDS_LIB_DIR="${FASTDDS_INSTALL_PREFIX}/lib"
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)


add_executable(p1_gait_dds_bridge
  src/p1_gait_dds_bridge_node.cpp
)

add_executable(p1_gait_dds_worker
  src/p1_gait_dds_worker.cpp
  ../../../fastdds_bridge/dds_topic/p1_gait/p1_gaitPubSubTypes.cxx
)

add_executable(p1_dds_worker
  src/p1_dds_worker.cpp
  ../../../fastdds_bridge/dds_topic/motor_12/motor_state_12/motor_state_12PubSubTypes.cxx
  ../../../fastdds_bridge/dds_topic/motor_12/motor_state_12/motor_state_12TypeObjectSupport.cxx
  ../../../fastdds_bridge/dds_topic/motor_12/motor_command_12/motor_command_12PubSubTypes.cxx
  ../../../fastdds_bridge/dds_topic/motor_12/motor_command_12/motor_command_12TypeObjectSupport.cxx
  ../../../fastdds_bridge/dds_topic/imu_topic_base/imu_topic_basePubSubTypes.cxx
  ../../../fastdds_bridge/dds_topic/imu_topic_base/imu_topic_baseTypeObjectSupport.cxx
)

target_include_directories(p1_gait_dds_bridge
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_definitions(p1_gait_dds_bridge
  PRIVATE
    P1_HW_FASTDDS_LIB_DIR="${FASTDDS_INSTALL_PREFIX}/lib"
)

target_include_directories(p1_gait_dds_worker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${FASTDDS_INSTALL_PREFIX}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../fastdds_bridge/dds_topic/p1_gait
)

target_include_directories(p1_dds_worker
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${FASTDDS_INSTALL_PREFIX}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../fastdds_bridge/dds_topic/motor_12/motor_state_12
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../fastdds_bridge/dds_topic/motor_12/motor_command_12
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../fastdds_bridge/dds_topic/imu_topic_base
)

ament_target_dependencies(p1_gait_dds_bridge
  ament_index_cpp
  geometry_msgs
  ocs2_msgs
  rclcpp
  std_msgs
)

target_link_libraries(p1_gait_dds_worker
  ${P1_HW_FASTDDS_LIB}
  ${P1_HW_FASTCDR_LIB}
  ${P1_HW_FOONATHAN_MEMORY_LIB}
)

target_link_libraries(p1_dds_worker
  ${P1_HW_FASTDDS_LIB}
  ${P1_HW_FASTCDR_LIB}
  ${P1_HW_FOONATHAN_MEMORY_LIB}
)

pluginlib_export_plugin_description_file(hardware_interface legged_p1_hw_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS p1_gait_dds_bridge p1_gait_dds_worker p1_dds_worker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES legged_p1_hw_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  hardware_interface
  legged_hw
  ocs2_msgs
  pluginlib
  rclcpp
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
