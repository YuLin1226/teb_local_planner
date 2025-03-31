`odom_helper_ comes from this `base_local_planner::OdometryHelperRos` class.
Its definition can be seen in these websites:

[Header](https://docs.ros.org/en/jade/api/base_local_planner/html/odometry__helper__ros_8h_source.html)
[Implementation](https://docs.ros.org/en/jade/api/base_local_planner/html/odometry__helper__ros_8cpp_source.html)

```cpp=
void OdometryHelperRos::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
   // Set current velocities from odometry
   geometry_msgs::Twist global_vel;
   {
     boost::mutex::scoped_lock lock(odom_mutex_);
     global_vel.linear.x = base_odom_.twist.twist.linear.x;
     global_vel.linear.y = base_odom_.twist.twist.linear.y;
     global_vel.angular.z = base_odom_.twist.twist.angular.z;
 
     robot_vel.frame_id_ = base_odom_.child_frame_id;
   }
   robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
   robot_vel.stamp_ = ros::Time();
}
```

Basically, it gets robot velocity from the odom subscriber.