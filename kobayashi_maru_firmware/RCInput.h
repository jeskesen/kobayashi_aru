/*
 * RCInput.h
 *
 *  Created on: Nov 19, 2017
 *      Author: Justin Eskesen
 */

#ifndef RCINPUT_H_
#define RCINPUT_H_

#include <PulsePosition.h>
class RCInput {
public:
	enum RC_CHANNEL
	{
		RC_ELEVATOR=0,
		RC_RUDDER,
		RC_THROTTLE,
		RC_AILERON,
		RC_GEAR,
		RC_AUX,
		RC_N_CHANNELS
	};
	RCInput(){}
	virtual ~RCInput(){}
	void init(uint8_t pin){rcIn.begin(pin);}
	bool read()
	{
	  if(rcIn.available() == 6)
	  {
	    for(uint8_t i=0; i<RC_N_CHANNELS; i++)
	    {
	    	buffer[i]=rcIn.read(i+1);
	    }
	    return true;
	  }
	  return false;
	}
	bool isManual() {return buffer[RC_GEAR] < DEFAULT_PULSE_WIDTH + 200;}
	float getSteering(){ return buffer[RC_RUDDER]; }
	float getThrottle(){ return buffer[RC_ELEVATOR]; }
private:
	PulsePositionInput rcIn;
	float buffer[6];

};

#endif /* RCINPUT_H_ */
