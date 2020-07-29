cmake_minimum_required(VERSION 3.5)
project(mobile_robot_comms)

if (CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS "Debug enabled")
    add_definitions(-DDEBUG)
endif ()

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin QUIET COMPONENTS
        roslaunch
        roscpp
        rospy
        std_msgs
        nav_msgs
        sensor_msgs
        geometry_msgs
        message_generation
        message_filters
        tf
        )
roslaunch_add_file_check(launch)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )

set(LIB_SRC
        include/kinematics/differential_drive.h
        src/kinematics/differential_drive.cpp
        include/kinematics/pose2d.h
        include/base_controller.h
        src/base_controller.cpp
        include/odometry_publisher.h
        src/odometry_publisher.cpp
        include/state_estimation/imu_odom_ekf.h
        src/state_estimation/imu_odom_ekf.cpp
        src/
        include/imu.h
        src/imu.cpp
        include/madgwick_filter.h
        src/madgwick_filter.cpp
        include/madgwick_filter_library.h
        src/madgwick_filter_library.cpp
        include/kinematics/omniwheel_base.h
        src/kinematics/omniwheel_base.cpp)


set(LIB_LINK_LIBRARIES
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})

add_library(mob_comms ${LIB_SRC})
target_link_libraries(mob_comms ${LIB_LINK_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main mob_comms)

if (${roscpp_FOUND})
    message(STATUS "ROS found")

    add_message_files(
            FILES
    )

    add_service_files(
            FILES
    )

    catkin_package(
            CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs genmsg tf
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
                mob_comms)

        MESSAGE(STATUS "Generating ROS node " ${node_name})
    endforeach(ros_node_src)
endif()
