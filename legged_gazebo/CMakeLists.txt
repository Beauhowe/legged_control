cmake_minimum_required(VERSION 3.10)
project(legged_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(legged_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(legged_hw_sim SHARED
        src/LeggedHWSim.cpp
        )

target_include_directories(legged_hw_sim
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(legged_hw_sim
        angles
        gazebo_dev
        gazebo_ros
        gazebo_ros2_control
        hardware_interface
        legged_common
        nav_msgs
        pluginlib
        rclcpp
        std_msgs
        )

pluginlib_export_plugin_description_file(gazebo_ros2_control legged_hw_sim_plugins.xml)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target legged_hw_sim")
    add_clang_tooling(
            TARGETS legged_hw_sim
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS legged_hw_sim
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES legged_hw_sim_plugins.xml
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY config worlds launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_include_directories(include)
ament_export_libraries(legged_hw_sim)
ament_export_dependencies(
        angles
        gazebo_dev
        gazebo_ros
        gazebo_ros2_control
        hardware_interface
        legged_common
        nav_msgs
        pluginlib
        rclcpp
        std_msgs
        )
ament_export_targets(export_${PROJECT_NAME})

ament_package()
