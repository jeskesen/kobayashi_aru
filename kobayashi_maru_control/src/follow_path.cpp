#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

class PathFollower
{
public:
  PathFollower();

  void spin();
protected:
  void init();
  void spinOnce();
  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Publisher rightSteeringPub, leftSteeringPub, tireVelPub;
  sensor_msgs::JointState rightSteeringMsg, leftSteeringMsg, tireVelMsg;
  ros::Rate rate;
};

PathFollower::PathFollower() :
    rate(10.0)
{
}

void PathFollower::init()
{
  rightSteeringPub = node.advertise<sensor_msgs::JointState>("right_steering_joint", 1000);
  leftSteeringPub = node.advertise<sensor_msgs::JointState>("left_steering_joint", 1000);
  tireVelPub = node.advertise<sensor_msgs::JointState>("tire_rotation", 1000);


  rightSteeringMsg.position.push_back(0.0);
  rightSteeringMsg.name.push_back("right_steering_joint");
  leftSteeringMsg.position.push_back(0.0);
  leftSteeringMsg.name.push_back("left_steering_joint");
  tireVelMsg.velocity.push_back(0.0); // so that it's allocated
}

void PathFollower::spin()
{
  ros::Duration(1.0).sleep();

  init();

  while (node.ok())
  {
    try
    {
      listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    spinOnce();

    rate.sleep();
  }
}

void PathFollower::spinOnce()
{

  tireVelMsg.velocity[0]=1.0;
  rightSteeringMsg.position[0]=1.0;
  leftSteeringMsg.position[0]=1.0;

  tireVelPub.publish(tireVelMsg);
  rightSteeringPub.publish(rightSteeringMsg);
  leftSteeringPub.publish(leftSteeringMsg);

  ROS_INFO("%f %f %f",
      transform.getOrigin().getX(),
      transform.getOrigin().getY(),
      transform.getOrigin().getZ());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_path");

  PathFollower follower;
  follower.spin();
  return 0;
}

