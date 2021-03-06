cmake_minimum_required(VERSION 2.8.3)
project(manipulator_moveit_control)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    moveit_visual_tools
    eigen_conversions
    roscpp
    rospy
    roslib
    rospack
    std_msgs
    std_srvs
    sensor_msgs
    dynamic_reconfigure
    pepper_picking_srv
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
    config/JointCtlParams.cfg
    config/impedance_control_params.cfg
    )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack moveit_core moveit_visual_tools moveit_ros_planning_interface sensor_msgs eigen_conversions pepper_picking_srv
   DEPENDS EIGEN3
   LIBRARIES ManipulatorControl
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

add_executable(JointPositionControl src/JointPositionControl.cpp)
target_link_libraries(JointPositionControl pid ${catkin_LIBRARIES} yaml-cpp)

add_library(ManipulatorControl src/ManipulatorControl.cpp)
target_link_libraries(ManipulatorControl ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(wp_manipulator_control src/WPManipulatorControl.cpp)
target_link_libraries(wp_manipulator_control ${catkin_LIBRARIES} ManipulatorControl)
