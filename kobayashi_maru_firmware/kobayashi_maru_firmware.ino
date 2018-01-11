#include "Arduino.h"
#include "PinDefs.h"

#include <Servo.h>
Servo steering, throttle;

#include "RCInput.h"
RCInput rc;

void setup()
{
	Serial.begin(115200);
	rc.init(CPPM_PIN);

	throttle.attach(THROTTLE_ESC_PIN, 1000, 2000);
	throttle.writeMicroseconds(DEFAULT_PULSE_WIDTH);

	steering.attach(STEERING_SERVO_PIN, 1000, 2000);
	steering.writeMicroseconds(DEFAULT_PULSE_WIDTH);

}

void loop()
{
	if (rc.read())
	{
		if (rc.isManual())
		{
			steering.writeMicroseconds((int) rc.getSteering());
			throttle.writeMicroseconds((int) rc.getThrottle());
		}
		else
		{
			steering.writeMicroseconds(DEFAULT_PULSE_WIDTH);
			throttle.writeMicroseconds(DEFAULT_PULSE_WIDTH);
		}
	}
	else
	{
		steering.writeMicroseconds(DEFAULT_PULSE_WIDTH);
		throttle.writeMicroseconds(DEFAULT_PULSE_WIDTH);
	}
}

