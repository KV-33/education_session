#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

#define WHEEL_BASE                0.184     // база колесная в метрах
#define WHEEL_DIAMETER            0.084     // диаметр колеса в метрах

double linear = 0.0;
double angular = 0.0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  linear = msg->linear.x;
  angular = msg->angular.z;
  ROS_INFO("Linear: [%f], Angular: [%f]", msg->linear.x, msg->angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1000, cmd_velCallback);
  ros::Publisher motor_left_pub = nh.advertise<std_msgs::Int32>("motor/left", 1000);
  ros::Publisher motor_right_pub = nh.advertise<std_msgs::Int32>("motor/right", 1000);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    double V = linear;                      //линейная скорость
    double W = angular;                     //угловая скорость
    double r = WHEEL_DIAMETER/2.0;            //радиус колеса
    double d = WHEEL_BASE;                  //база робота

    std_msgs::Int32 msg_left;
    std_msgs::Int32 msg_right;

    msg_left.data = ((1.0 / r) * V - (d / r) * W) * 11;
    msg_right.data = ((1.0 / r) * V + (d / r) * W) * 11;

    motor_left_pub.publish(msg_left);
    motor_right_pub.publish(msg_right);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
