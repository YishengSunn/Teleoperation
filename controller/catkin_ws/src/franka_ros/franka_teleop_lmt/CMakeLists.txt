cmake_minimum_required(VERSION 3.4)
project(franka_teleop_lmt)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  std_msgs
  franka_gripper
  actionlib
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_message_files(FILES JointTorqueComparison.msg)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_teleop_lmt
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    actionlib
  DEPENDS Franka
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## =============================
## Controllers and Libraries
## =============================

add_library(franka_teleop_lmt src/cartesian_impedance_example_controller.cpp)
target_link_libraries(franka_teleop_lmt ${Franka_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(franka_teleop_lmt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_controllers_lib src/cartesian_impedance_example_controller.cpp)
target_link_libraries(${PROJECT_NAME}_controllers_lib ${catkin_LIBRARIES})

## =============================
## Simple Gripper Client (NO DHD)
## =============================
add_executable(simple_gripper_client src/moving_gripper.cpp)
target_link_libraries(simple_gripper_client ${catkin_LIBRARIES})
add_dependencies(simple_gripper_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## =============================
## DHD-dependent code (optional)
## =============================
find_path(DHD_INCLUDE_DIR NAMES "dhdc.h" PATHS "/home/rmil/sdk-3.14.0/include")
find_library(DHDLIB NAMES "libdhd.a" PATHS "/home/rmil/sdk-3.14.0/lib/release/lin-x86_64-gcc")

if(DHD_INCLUDE_DIR AND DHDLIB)
  message(STATUS "DHD found — building DHD nodes")
  include_directories(${DHD_INCLUDE_DIR})

  add_executable(leader_node src/leader.cpp)
  target_link_libraries(leader_node ${DHDLIB} usb-1.0 rt ncurses ${catkin_LIBRARIES})

else()
  message(WARNING "DHD library not found — leader_node will not be built")
endif()

## =============================
## LfD Node (always built)
## =============================
add_executable(LfD_node src/LfD.cpp)
target_link_libraries(LfD_node ${catkin_LIBRARIES})

## =============================
## Install & Formatting
## =============================
install(TARGETS franka_teleop_lmt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES franka_teleop_lmt_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
