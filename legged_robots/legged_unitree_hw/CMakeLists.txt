cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(legged_hw REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(lib)

set(SOURCE src/UnitreeHW.cpp)

find_library(LCM_LIBRARY lcm)
set(UNITREE_TARGETS)

if (LCM_LIBRARY)
    add_library(${PROJECT_NAME}_3_3_1 SHARED
            ${SOURCE}
            )

    target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)
    target_include_directories(${PROJECT_NAME}_3_3_1
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            )
    ament_target_dependencies(${PROJECT_NAME}_3_3_1
            hardware_interface
            legged_hw
            pluginlib
            rclcpp
            sensor_msgs
            std_msgs
            )

    target_link_libraries(${PROJECT_NAME}_3_3_1
            libunitree_legged_sdk_amd64_3_3_1.so
            ${LCM_LIBRARY}
            )
    pluginlib_export_plugin_description_file(hardware_interface legged_unitree_hw_plugins.xml)
    list(APPEND UNITREE_TARGETS ${PROJECT_NAME}_3_3_1)
else ()
    message(WARNING "LCM library was not found; skipping Unitree SDK 3.3.1 hardware plugin.")
endif ()

add_library(${PROJECT_NAME}_3_8_0 SHARED
        ${SOURCE}
)

target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)
target_include_directories(${PROJECT_NAME}_3_8_0
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
ament_target_dependencies(${PROJECT_NAME}_3_8_0
        hardware_interface
        legged_hw
        pluginlib
        rclcpp
        sensor_msgs
        std_msgs
        )

target_link_libraries(${PROJECT_NAME}_3_8_0
        libunitree_legged_sdk_amd64_3_8_0.a
)

pluginlib_export_plugin_description_file(hardware_interface legged_unitree_hw_3_8_0_plugins.xml)
list(APPEND UNITREE_TARGETS ${PROJECT_NAME}_3_8_0)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_3_3_1 TARGETS ${PROJECT_NAME}_3_8_0
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${UNITREE_TARGETS}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

# Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config lib
        DESTINATION share/${PROJECT_NAME}
        )

install(FILES launch/go1.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
        )

install(FILES legged_unitree_hw_plugins.xml legged_unitree_hw_3_8_0_plugins.xml
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_3_3_1 ${PROJECT_NAME}_3_8_0)
ament_export_dependencies(
        hardware_interface
        legged_hw
        pluginlib
        rclcpp
        sensor_msgs
        std_msgs
        )
ament_export_targets(export_${PROJECT_NAME})

ament_package()
