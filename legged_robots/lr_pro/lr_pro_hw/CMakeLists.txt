cmake_minimum_required(VERSION 3.10)
project(lr_pro_hw)

# 导出 compile_commands.json，便于 clangd/IDE 识别 include 与编译选项。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(legged_hw REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# lr_pro_hw 是 ros2_control 的硬件插件库，会被 pluginlib 动态加载。
add_library(${PROJECT_NAME} SHARED
        src/LrProHW.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        legged_hw
        pluginlib
        rclcpp
)

# 将 lr_pro_hw_plugins.xml 注册给 hardware_interface，供 ros2_control 查找插件。
pluginlib_export_plugin_description_file(hardware_interface lr_pro_hw_plugins.xml)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# 安装头文件，便于其他包包含 lr_pro_hw/LrProHW.h。
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

# 安装默认配置和 launch 文件。
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE
)

# 安装插件描述文件。
install(FILES lr_pro_hw_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        hardware_interface
        legged_hw
        pluginlib
        rclcpp
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
