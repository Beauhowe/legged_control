cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(legged_common REQUIRED)
find_package(legged_interface REQUIRED)
find_package(legged_wbc REQUIRED)
find_package(legged_estimation REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(pinocchio REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS filesystem system log log_setup)

add_library(${PROJECT_NAME} SHARED
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        src/synchronized_module/PlanarTerrainReceiver.cpp
        src/LeggedController.cpp
        src/PerceptiveController.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
        )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_lifecycle
        controller_interface
        hardware_interface
        pluginlib
        angles
        std_msgs
        visualization_msgs
        ocs2_msgs
        ocs2_ros_interfaces
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        ocs2_sqp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        grid_map_sdf
        grid_map_ros
        convex_plane_decomposition_ros
        convex_plane_decomposition_msgs
        convex_plane_decomposition
        pinocchio
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
pluginlib_export_plugin_description_file(controller_interface legged_controllers_plugins.xml)


# Target trajectories publish node
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        )
target_link_libraries(legged_target_trajectories_publisher ${PROJECT_NAME})
ament_target_dependencies(legged_target_trajectories_publisher
        rclcpp
        geometry_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        ocs2_msgs
        ocs2_ros_interfaces
        grid_map_ros
        convex_plane_decomposition_ros
        convex_plane_decomposition_msgs
        convex_plane_decomposition
        )
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS legged_target_trajectories_publisher
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

install(PROGRAMS scripts/joy_control.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME joy_control
        )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        controller_interface
        hardware_interface
        pluginlib
        angles
        std_msgs
        visualization_msgs
        ocs2_msgs
        ocs2_ros_interfaces
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        ocs2_sqp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        grid_map_sdf
        grid_map_ros
        convex_plane_decomposition_ros
        convex_plane_decomposition_msgs
        convex_plane_decomposition
        pinocchio
        )
ament_export_targets(export_${PROJECT_NAME})

ament_package()
