#!/bin/bash
sudo apt install curl
sudo apt install ros-noetic-desktop-full
sudo apt-get install ros-noetic-imu-tools
sudo apt-get install ros-noetic-plotjuggler-ros ros-noetic-plotjuggler
sudo apt-get install ros-noetic-robot-pose-ekf
sudo apt-get install ros-noetic-robot-localization
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-gmapping ros-noetic-amcl
sudo apt-get install pip3
sudo apt-get install python3-pip
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-gazebo ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-ifopt
sudo apt-get install ros-noetic-global-planner
sudo apt-get install ros-noetic-slam-toolbox
sudo pip3 install starlette uvicorn
sudo pip3 install luma.emulator luma
sudo pip3 install sse_starlette
sudo pip3 install luma.oled

# Version, die funktioniert hat (Linux):

# mamba create -n jupyros -c conda-forge python nodejs=14 jupyterlab=3 jupyter bqplot pyyaml ipywidgets ipycanvas ros-noetic-imu-tools ros-noetic-navigation ros-noetic-robot-localization ros-noetic-gmapping ros-noetic-global-planner ros-noetic-amcl ros-noetic-desktop ros-noetic-plotjuggler ros-noetic-plotjuggler-ros ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-gazebo ros-noetic-gazebo-ros-pkgs ros-noetic-desktop catkin_tools compilers make cmake -c robostack

# Installationsanleitung aus https://github.com/RoboStack/jupyter-ros

# Lesen: ipywidgets
