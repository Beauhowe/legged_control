cmake_minimum_required(VERSION 3.10)
project(legged_description)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY meshes urdf launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
