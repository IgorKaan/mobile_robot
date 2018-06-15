cmake_minimum_required(VERSION 3.5)
project(mobile_robot_comms)

if (CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "Debug enabled")
    add_definitions(-DDEBUG)
endif ()

set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)
find_package(catkin QUIET COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        message_generation
        message_filters
        )

include_directories(include/)

set(SRC_NO_MAIN
        include/linux_serial.h
        src/linux_serial.cpp
        include/kinematics/differential_drive.h
        src/kinematics/differential_drive.cpp
        include/kinematics/pose2d.h src/nodes/test_node.cpp)

set(LINK_LIBRARIES
        ${CMAKE_THREAD_LIBS_INIT})

set(INLCUDE_DIRECTORIES
        )

add_library(mob_comms_lib ${SRC_NO_MAIN})
target_link_libraries(mob_comms_lib ${LINK_LIBRARIES})
target_include_directories(mob_comms_lib PUBLIC ${INCLUDE_DIRECTORIES})

add_executable(mob_comms src/main.cpp)
target_link_libraries(mob_comms mob_comms_lib)

if (${roscpp_FOUND})
    message(STATUS "ROS found")

    add_message_files(
            FILES
    )

    add_service_files(
            FILES
    )

    catkin_package(
            CATKIN_DEPENDS roscpp std_msgs
    )

    include_directories(
            ${catkin_INCLUDE_DIRS}
    )

    file(GLOB ROS_NODES_SRC RELATIVE
            ${CMAKE_CURRENT_SOURCE_DIR}
            src/nodes/*.cpp)

    foreach(ros_node_src ${ROS_NODES_SRC})
        get_filename_component(node_name ${ros_node_src} NAME_WE)

        add_executable(${node_name} ${ros_node_src})
        target_link_libraries(${node_name}
                mob_comms_lib
                ${catkin_LIBRARIES})

        MESSAGE(STATUS "Generating node" node_name)
    endforeach(ros_node_src)
endif()
