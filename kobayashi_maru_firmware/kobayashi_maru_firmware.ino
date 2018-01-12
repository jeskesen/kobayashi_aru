#include "Arduino.h"
#include "PinDefs.h"

#include <ros.h>
ros::NodeHandle nh;

#include "FlowOdometry.h"
FlowOdometry flow(PMW3901_CS);
#include <geometry_msgs/Twist.h>
geometry_msgs::Twist flow_twist_msg;
ros::Publisher flow_twist("flow_twist", &flow_twist_msg);

// ACTUATION
#include <Servo.h>
Servo steering_servo, throttle_servo;

#include <std_msgs/UInt16.h>
std_msgs::UInt16 steering_pwm_msg, throttle_pwm_msg;
ros::Publisher steering_pwm("steering_pwm", &steering_pwm_msg);
ros::Publisher throttle_pwm("throttle_pwm", &throttle_pwm_msg);

#include "RCInput.h"
RCInput rc;

void setup()
{
  nh.initNode();
  nh.advertise(steering_pwm);
  nh.advertise(throttle_pwm);

  if(flow.init())
    nh.advertise(flow_twist);

  rc.init(CPPM_PIN);

  throttle_servo.attach(THROTTLE_ESC_PIN, 1000, 2000);
  throttle_servo.writeMicroseconds(DEFAULT_PULSE_WIDTH);

  steering_servo.attach(STEERING_SERVO_PIN, 1000, 2000);
  steering_servo.writeMicroseconds(DEFAULT_PULSE_WIDTH);

}

void loop()
{
  if (flow.getIsInitialized())
  {
    float x_vel, y_vel;
    flow.read_vel(x_vel, y_vel);
    flow_twist_msg.linear.x = x_vel;
    flow_twist_msg.linear.y = y_vel;
    flow_twist.publish(&flow_twist_msg);

  }

  if (rc.read())
  {
    if (rc.isManual())
    {
      steering_pwm_msg.data = (uint16_t) rc.getSteering();
      throttle_pwm_msg.data = (uint16_t) rc.getThrottle();
    }
    else
    {
      steering_pwm_msg.data = (uint16_t) DEFAULT_PULSE_WIDTH;
      throttle_pwm_msg.data = (uint16_t) DEFAULT_PULSE_WIDTH;
    }
  }
  else
  {
    steering_pwm_msg.data = (uint16_t) DEFAULT_PULSE_WIDTH;
    throttle_pwm_msg.data = (uint16_t) DEFAULT_PULSE_WIDTH;
  }

  steering_servo.writeMicroseconds(steering_pwm_msg.data);
  throttle_servo.writeMicroseconds(throttle_pwm_msg.data);
  steering_pwm.publish(&steering_pwm_msg);
  throttle_pwm.publish(&throttle_pwm_msg);

  nh.spinOnce();

}

