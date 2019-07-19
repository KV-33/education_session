#ifndef _ROS_green_robot_SensorBoolStamped_h
#define _ROS_green_robot_SensorBoolStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace green_robot
{

  class SensorBoolStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _left_type;
      _left_type left;
      typedef bool _center_type;
      _center_type center;
      typedef bool _right_type;
      _right_type right;

    SensorBoolStamped():
      header(),
      left(0),
      center(0),
      right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left);
      union {
        bool real;
        uint8_t base;
      } u_center;
      u_center.real = this->center;
      *(outbuffer + offset + 0) = (u_center.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->center);
      union {
        bool real;
        uint8_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        bool real;
        uint8_t base;
      } u_center;
      u_center.base = 0;
      u_center.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->center = u_center.real;
      offset += sizeof(this->center);
      union {
        bool real;
        uint8_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right = u_right.real;
      offset += sizeof(this->right);
     return offset;
    }

    const char * getType(){ return "green_robot/SensorBoolStamped"; };
    const char * getMD5(){ return "c1e628620f7b17973fdf2e478d56500a"; };

  };

}
#endif
