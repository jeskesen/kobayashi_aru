#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <MiniPID/MiniPID.h>

class PathFollower
{
public:
  PathFollower();

  void spin();
protected:
  void init();
  void followPath();
  void onNewPathReceived(const nav_msgs::Path::ConstPtr& msg);
  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Publisher rightSteeringPub, leftSteeringPub, tireVelPub;
  ros::Subscriber pathSubscriber;

  sensor_msgs::JointState rightSteeringMsg, leftSteeringMsg, tireVelMsg;
  ros::Rate rate;

  /// this is the path we will follow
  nav_msgs::Path path;
  /// this holds our progress as we iterate through the path.
  std::vector<geometry_msgs::PoseStamped>::iterator currentPose;
  MiniPID pid;

  double captureRadius;
  double speed;
};

PathFollower::PathFollower() :
    rate(10.0),
    pid(1.0,0.0,0.0) // setting these now because it has no default constructor, set to better values later.
{
  currentPose=path.poses.end();
  captureRadius = 0.1;
  speed = 1.0;
}

void PathFollower::init()
{
  rightSteeringPub = node.advertise<sensor_msgs::JointState>("right_steering_joint", 1000);
  leftSteeringPub = node.advertise<sensor_msgs::JointState>("left_steering_joint", 1000);
  tireVelPub = node.advertise<sensor_msgs::JointState>("tire_rotation", 1000);
  pathSubscriber = node.subscribe("path_to_follow", 1000, &PathFollower::onNewPathReceived, this);


  rightSteeringMsg.position.push_back(0.0);
  rightSteeringMsg.name.push_back("right_steering_joint");
  leftSteeringMsg.position.push_back(0.0);
  leftSteeringMsg.name.push_back("left_steering_joint");
  tireVelMsg.velocity.push_back(0.0); // so that it's allocated

  pid.setSetpoint(0.0); // dead reckon
  pid.setOutputLimits(angles::from_degrees(-30), angles::from_degrees(30));
  //pid.setOutputRampRate(0.1);
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

    followPath();

    tireVelPub.publish(tireVelMsg);
    rightSteeringPub.publish(rightSteeringMsg);
    leftSteeringPub.publish(leftSteeringMsg);

    ros::spinOnce();
    rate.sleep();
  }
}


void PathFollower::followPath()
{

  // are we done? if so, stop.
  if(currentPose==path.poses.end())
  {
    tireVelMsg.velocity[0]=0.0;
    rightSteeringMsg.position[0]=0.0;
    leftSteeringMsg.position[0]=0.0;
    return;
  }

  // are we at our current goal point?  if so, increment our goal iterator, and recurse.
  tf::Vector3 currentGoalPosition(currentPose->pose.position.x, currentPose->pose.position.y, 0.0);
  double distanceToGo = currentGoalPosition.distance(transform.getOrigin());
  if(distanceToGo < captureRadius)
  {
    currentPose++;
    return followPath();
  }


  // THIS IS WHERE THE CONTROL HAPPENS
  double currentDesiredXVel = speed;

  tf::Vector3 robotRelativeGoal=transform.invXform(currentGoalPosition);  // transform current goal into robot frame
  double angleToGoal = -atan2(robotRelativeGoal.getY(), robotRelativeGoal.getX());
  //ROS_INFO("Angle %f", angleToGoal);

  // do a k-turn if goal is behind us.
  if (fabs(angleToGoal) > angles::from_degrees(90))
  {
      currentDesiredXVel *= -1;
      angleToGoal *= -1;  // ToDo: probably want to do something more clever to clear PID when switching directions.
  }

  double steeringAngle = pid.getOutput(angleToGoal);
  rightSteeringMsg.position[0]=steeringAngle;
  leftSteeringMsg.position[0]=steeringAngle;
  tireVelMsg.velocity[0]=currentDesiredXVel/ 0.14605;

}



void PathFollower::onNewPathReceived(const nav_msgs::Path::ConstPtr& msg)
{
   //ROS_INFO("HERE");
  path = *(msg.get());
  currentPose = path.poses.begin();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_path");

  PathFollower follower;
  follower.spin();
  return 0;
}

