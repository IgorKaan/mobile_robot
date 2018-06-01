cmake_minimum_required(VERSION 3.10)
project(mobile_robot_comms)

set(CMAKE_CXX_STANDARD 14)

include_directories(include/)

set(SRC src/main.cpp include/linux_serial.h src/linux_serial.cpp)

add_executable(mobile_robot_comms ${SRC})
