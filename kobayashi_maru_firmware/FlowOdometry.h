/*
 * FlowOdometry.h
 *
 *  Created on: Jan 11, 2018
 *      Author: Justin Eskesen
 */

#ifndef FLOWODOMETRY_H_
#define FLOWODOMETRY_H_
#include "Bitcraze_PMW3901.h"

class FlowOdometry
{
public:
  FlowOdometry(uint8_t cspin);
  virtual ~FlowOdometry()
  {
  }
  bool init();
  bool getIsInitialized() const
  {
    return _is_initialized;
  }
  void read_vel(float& x, float& y);
private:
  Bitcraze_PMW3901 _pmw3901;
  float _height;
  bool _is_initialized;
  static const float _degrees_per_pixel;
};

#endif /* FLOWODOMETRY_H_ */
