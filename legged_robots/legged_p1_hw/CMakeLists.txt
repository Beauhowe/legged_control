cmake_minimum_required(VERSION 3.10)
project(legged_p1_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(hardware_interface REQUIRED)
find_package(legged_hw REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

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
    ${FASTDDS_INSTALL_PREFIX}/include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  legged_hw
  pluginlib
  rclcpp
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
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
  hardware_interface
  legged_hw
  pluginlib
  rclcpp
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
