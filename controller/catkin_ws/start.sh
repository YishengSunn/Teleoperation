#!/bin/bash

roslaunch franka_example_controllers move_to_start.launch robot_ip:=172.16.0.10 robot:=panda
roslaunch franka_teleop_lmt Teleop.launch robot_ip:=172.16.0.10
