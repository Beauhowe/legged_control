cmake_minimum_required(VERSION 3.10)
project(lr_pro_description)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

# 安装模型、mesh 和 launch 文件，供 package://lr_pro_description/... 路径解析使用。
install(
        DIRECTORY meshes urdf launch
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE
)

# 顶层 robot.xacro 放在 share/${PROJECT_NAME} 下，方便 launch 和其它包引用。
install(
        FILES robot.xacro
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
