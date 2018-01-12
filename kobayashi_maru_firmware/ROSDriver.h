/*
 * ROSDriver.h
 *
 *  Created on: Jan 12, 2018
 *      Author: Justin Eskesen
 */

#ifndef ROSDRIVER_H_
#define ROSDRIVER_H_

#include <ros.h>
#include <Metro.h>
#include <string>

class ROSDriver
{
public:
  ROSDriver(ros::NodeHandle *nh, char const* name,  unsigned long run_interval=100): _timer(run_interval, true)
  {
    _nh=nh;
    strcpy(_name, name);
    _isInitialized=false;
  }

  virtual ~ROSDriver(){}
  virtual bool init(){return true;}
  void runOnce()
  {
    if(isInitialized() && _timer.check())
    {
      runOnceInternal();
    }

  }
protected:
  virtual void runOnceInternal()=0;
  virtual bool isInitialized() {return _isInitialized;}
  ros::NodeHandle *_nh;
  Metro _timer;
  bool _isInitialized;
  char _name[64];
};

#endif /* ROSDRIVER_H_ */
