/*
 * FlowOdometry.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Justin Eskesen
 */

#include "FlowOdometry.h"

#define PMW3901_DEGREES_PER_PIXEL 4.2

FlowOdometry::FlowOdometry(uint8_t cspin) :
	_pmw3901(cspin)
{
	_height = 0.1;
  _is_initialized = false;
}

bool FlowOdometry::init()
{
	return _is_initialized = _pmw3901.begin();
}

void FlowOdometry::read_data()
{
	if(_is_initialized)
	{
		int16_t deltaX,deltaY;
		_pmw3901.readMotionCount(&deltaX, &deltaY);
	}
}

