# norlab_icp_mapper documentation

Welcome to the norlab_icp_mapper documentation!
norlab_icp_mapper is a library for localization and mapping relying on the "Iterative Closest Point" algorithm.
It uses [libpointmatcher](https://github.com/norlab-ulaval/libpointmatcher) as backend for point cloud registration and [libnabo](https://github.com/norlab-ulaval/libnabo) for fast kd-tree search.

The library is developed by [norlab](https://norlab.ulaval.ca/), who also develops and maintains a number of other libraries for robotics and deep learning. Here's a non-exhaustive list:

+ [imu_odom](https://github.com/norlab-ulaval/imu_odom) estimates IMU poses based on ICP poses and accelerometer measurements. Useful when your robot doesn't have access to wheel odometry.
+ [norlab_controllers](https://github.com/norlab-ulaval/norlab_controllers) is a library with different control algorithms for using robots in the field.
+ [norlab_controllers_ros](https://github.com/norlab-ulaval/norlab_controllers_ros) is a ROS wrapper for the controller library.

Use the left menu to navigate the documentation.
To report bugs, please use our [github issue tracker](http://github.com/norlab-ulaval/norlab_icp_mapper/issues).

![Tunnel light](images/index_tunnel_light.png#only-light)
![Tunnel dark](images/index_tunnel_dark.png#only-dark)