# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/rmilnuc1/libfranka/include".split(';') if "${prefix}/include;/home/rmilnuc1/libfranka/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib_msgs;actionlib;controller_interface;combined_robot_hw;hardware_interface;joint_limits_interface;roscpp;std_srvs;pluginlib;urdf;franka_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_hw;-lfranka_control_services;/home/rmilnuc1/libfranka/build/libfranka.so.0.13.3".split(';') if "-lfranka_hw;-lfranka_control_services;/home/rmilnuc1/libfranka/build/libfranka.so.0.13.3" != "" else []
PROJECT_NAME = "franka_hw"
PROJECT_SPACE_DIR = "/home/rmilnuc1/catkin_ws/install"
PROJECT_VERSION = "0.10.1"
