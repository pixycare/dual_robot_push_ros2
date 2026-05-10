#!/bin/bash

colcon build
source install/local_setup.bash
ros2 run project_package project_node
