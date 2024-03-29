# Roomba 5xx Cleaner implementation based on ROS (Robot Operating System)

Current Build status: ![Build Workflow](https://github.com/mirkosertic/roomba500/workflows/build/badge.svg) 

[Please visit my blog for more!](https://www.mirkosertic.de/blog/2022/02/roomba-series/)

Depends on:

- https://github.com/alex-arnal/costmap_prohibition_layer

ToDo's:

* Verify ekf_localization_motion_model with differential drive setup (http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html#sensor-differential)
* Test different resolutions in local and global costmap
* Document everything as a series of blog posts
* Test navigation and SLAM precision
* Full coverage real-life test
* Enable/disable vacuum based on map
* Implement docking action