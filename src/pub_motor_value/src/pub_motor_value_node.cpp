#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_motor_value_node");
  ros::NodeHandle nh;

  ros::Publisher motor_pub = nh.advertise<std_msgs::Int32>("motor/left", 1000);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = 125;

    motor_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
