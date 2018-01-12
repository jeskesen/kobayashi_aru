/*
 * FlowOdometry.h
 *
 *  Created on: Jan 11, 2018
 *      Author: Justin Eskesen
 */

#ifndef FLOWODOMETRY_H_
#define FLOWODOMETRY_H_
#include "ROSDriver.h"
#include "Bitcraze_PMW3901.h"
#include <geometry_msgs/Twist.h>

class FlowOdometry : ROSDriver
{
public:
  FlowOdometry(ros::NodeHandle *nh, char const* name, uint8_t cspin);
  virtual ~FlowOdometry(){  }
  virtual bool init();

protected:
  virtual void runOnceInternal();
  void read_vel(float& x, float& y);
  Bitcraze_PMW3901 _pmw3901;
  float _height;
  bool _is_initialized;
  static const float _degrees_per_pixel;

  geometry_msgs::Twist _twist_msg;
  ros::Publisher _twistPublisher;

};

#endif /* FLOWODOMETRY_H_ */
