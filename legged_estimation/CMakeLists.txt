cmake_minimum_required(VERSION 3.10)
project(legged_estimation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(legged_common REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system log log_setup)

add_library(${PROJECT_NAME}
        src/StateEstimateBase.cpp
        src/FromTopicEstimate.cpp
        src/LinearKalmanFilter.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        geometry_msgs
        nav_msgs
        realtime_tools
        tf2
        tf2_geometry_msgs
        tf2_ros
        legged_common
        ocs2_centroidal_model
        ocs2_legged_robot
        ocs2_pinocchio_interface
        ocs2_robotic_tools
        pinocchio
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        geometry_msgs
        nav_msgs
        realtime_tools
        tf2
        tf2_geometry_msgs
        tf2_ros
        legged_common
        ocs2_centroidal_model
        ocs2_legged_robot
        ocs2_pinocchio_interface
        ocs2_robotic_tools
        pinocchio
        )
ament_export_targets(export_${PROJECT_NAME})

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

ament_package()
