#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->linear.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_cmd_vel_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1000, cmd_velCallback);

  ros::spin();

  return 0;
}
