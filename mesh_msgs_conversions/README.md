# mesh_msgs_conversions


Conversion functions between lvr2 and mesh_msgs.


## ROS2 Port TODOs

No global nodes anymore 

I changed
1. ros::Time::now() -> rclcpp::Time()
2. ROS_INFO -> std::cout

The first point could destroy functionallity. The second one could destroy the logging. TODO: do this correctly