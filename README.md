# Roomba 5xx Cleaner implementation based on ROS (Robot Operating System)

Current Build status: ![Build Workflow](https://github.com/mirkosertic/roomba500/workflows/build/badge.svg) 

[Please visit my blog for more!](https://www.mirkosertic.de/blog/2022/02/roomba-series/)

ToDo's:

* Test wheel encoder odometry precision
* Test IMU measurements wrt gravity
* Test IMU-fused odometry precision
* Make IMU drift configurable
* Use fused odometry for SLAM packages
* Publish odom->base_link transform by ekf_localization_node instead of differentialodometry.py
* Verify ekf_localization_motion_model with differential drive setup (http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html#sensor-differential)
* Reduce differentialdriveodometry.py to a minimum
* Document everything as a series of blog posts
* Test navigation and SLAM precision
* Full coverage real-life test
* Enable/disable vacuum based on map
* Implement docking action