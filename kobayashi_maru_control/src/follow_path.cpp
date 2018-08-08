#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
class PathFollower
{
public:
  PathFollower();

  void spin();
protected:
  void init();
  void spinOnce();
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

  double captureRadius;
  double xVel;
};

PathFollower::PathFollower() :
    rate(10.0)
{
  currentPose=path.poses.end();
  captureRadius = 1.0;
  xVel = 1.0;
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

    tireVelPub.publish(tireVelMsg);
    rightSteeringPub.publish(rightSteeringMsg);
    leftSteeringPub.publish(leftSteeringMsg);

    ros::spinOnce();
    rate.sleep();
  }
}


void PathFollower::spinOnce()
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
  ROS_INFO("Distance %f", currentGoalPosition.distance(transform.getOrigin()));

  if(currentGoalPosition.distance(transform.getOrigin()) < captureRadius)
  {
    currentPose++;
    return spinOnce();
  }

  tireVelMsg.velocity[0]=xVel/ 0.14605;
  rightSteeringMsg.position[0]=0.0;
  leftSteeringMsg.position[0]=0.0;



}

void PathFollower::onNewPathReceived(const nav_msgs::Path::ConstPtr& msg)
{
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

