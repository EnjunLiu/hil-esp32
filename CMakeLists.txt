cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# micro-ROS reads custom message packages from extra_ros_packages/.

project(asv_esp32_firmware)
