#include "ros/ros.h"
#include "std_msgs/Int32.h"

#define MOTOR_VALUE           255

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_motor_value_params_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  int motor_value;
  priv_nh.param("motor_value", motor_value, MOTOR_VALUE);

  ros::Publisher motor_pub = nh.advertise<std_msgs::Int32>("motor/left", 1000);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = motor_value;

    motor_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
