# own_car
a simple car for make slam with ROS using a OpenCM 9.04 , two dynamixel motors and xbox 360 kinect
The base_controller subscribes to the rpm topic from the OpenCM and converts it into x velocity, theta velocity, xy position, and yaw. It also subscribes to the gyro topic from the android phone and combines the readings with the rpm information to produce a combined yaw. The base_controller then publishes this information in the form of odometry and tf messages.
Subscribed Topics : rpm(geometry_msgs::Vector3Stamped),
Published Topics : odom(nav_msgs:Odometry)
