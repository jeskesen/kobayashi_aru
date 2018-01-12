/*
 * FlowOdometry.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Justin Eskesen
 */

#include "FlowOdometry.h"

const float FlowOdometry::_degrees_per_pixel(4.2);


FlowOdometry::FlowOdometry(ros::NodeHandle *nh, char const* name, uint8_t cspin) :
    ROSDriver(nh, name, 50),
    _pmw3901(cspin),
    _twistPublisher(name, &_twist_msg)
{
  _height = 0.1;
}

bool FlowOdometry::init()
{
  _isInitialized = _pmw3901.begin();

  if(_isInitialized)
  {
    _nh->advertise(_twistPublisher);
  }
  return _isInitialized;
}

void FlowOdometry::runOnceInternal()
{
  float x_vel, y_vel;
  read_vel(x_vel, y_vel);
  _twist_msg.linear.x = x_vel;
  _twist_msg.linear.y = y_vel;
  _twistPublisher.publish(&_twist_msg);
}

void FlowOdometry::read_vel(float& x, float& y)
{
  int16_t deltaX, deltaY;
  _pmw3901.readMotionCount(&deltaX, &deltaY);
  x = _height * tanf(_degrees_per_pixel * deltaX);
  y = _height * tanf(_degrees_per_pixel * deltaY);

}

