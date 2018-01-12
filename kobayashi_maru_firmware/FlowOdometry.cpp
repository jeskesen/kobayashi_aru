/*
 * FlowOdometry.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Justin Eskesen
 */

#include "FlowOdometry.h"

const float FlowOdometry::_degrees_per_pixel(4.2);

FlowOdometry::FlowOdometry(uint8_t cspin) :
    _pmw3901(cspin)
{
  _height = 0.1;
  _is_initialized = false;
}

bool FlowOdometry::init()
{
  return (_is_initialized = _pmw3901.begin());
}

void FlowOdometry::read_vel(float& x, float& y)
{

  if (_is_initialized)
  {
    int16_t deltaX, deltaY;
    _pmw3901.readMotionCount(&deltaX, &deltaY);
    x = _height * tanf(_degrees_per_pixel * deltaX);
    y = _height * tanf(_degrees_per_pixel * deltaY);
  }
}

