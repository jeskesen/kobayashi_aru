#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

class kobayahsi_maru_simulation_node
{
public:
    kobayahsi_maru_simulation_node();
    ~kobayahsi_maru_simulation_node(){}
    void spin();
private:
    void init();
    void spinOnce();

    void rightSteeringCB(const sensor_msgs::JointState& msg);
    void tireRotationCB(const sensor_msgs::JointState& msg);

    double theta; // keep track of theta to save us from having to constantly convert from quaternion.
    double rightFrontAngle;
    double tireVel;
    ros::NodeHandle nodeHandle;
    ros::Subscriber rightSteeringSub, tireVelSub;
    ros::Publisher odomPublisher;

    geometry_msgs::TransformStamped odomTransform;
    
    tf::TransformBroadcaster odomTFBroadcaster;
    tf::TransformListener TFlistener;

    tf::Transform transform;
    
    ros::Time currentTime, lastTime;
    ros::Rate rate;
};

kobayahsi_maru_simulation_node::kobayahsi_maru_simulation_node() :
        rate(100)
{
    theta=0.0;
    rightFrontAngle=0.0;
    tireVel=0.0;

}

void kobayahsi_maru_simulation_node::init()
{
    ///Todo: read in physical characteristics of robot as params.  Hard-coded for now.

    rightSteeringSub = nodeHandle.subscribe("right_steering_joint", 10, &kobayahsi_maru_simulation_node::rightSteeringCB, this);
    tireVelSub = nodeHandle.subscribe("tire_rotation", 10, &kobayahsi_maru_simulation_node::tireRotationCB, this);
    odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 50);

    tf::Transform junk;
    //junk.setIdentity();
    //tf::transformTFToMsg(junk, odomTransform.transform);
    odomTransform.transform.translation.x = 0.0;
    odomTransform.transform.translation.y = 0.0;
    odomTransform.transform.translation.z = 0.0;
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    currentTime = lastTime = ros::Time::now();
}

void kobayahsi_maru_simulation_node::rightSteeringCB(
        const sensor_msgs::JointState& msg)
{
	rightFrontAngle = msg.position[0];
}

void kobayahsi_maru_simulation_node::tireRotationCB(
        const sensor_msgs::JointState& msg)
{
	tireVel = msg.velocity[0];
}


void kobayahsi_maru_simulation_node::spinOnce()
{

   /*  Was having trouble using tf, so tried another way.
    tf::StampedTransform rightWheelTransform, leftWheelTransform;
    try{
      TFlistener.lookupTransform( "/right_front_wheel", "/right_rear_wheel", ros::Time(0), rightWheelTransform);
      TFlistener.lookupTransform("/left_rear_wheel", "/left_front_wheel", ros::Time(0), leftWheelTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    */

    // I'm only going to worry about the right wheel for now.  Eventually, I will incorporate both.
    double rightWheelSpacing = 0.335;
    double tireDiameter = 0.14605;

    //compute odometry using the turning radius
    double dt = (currentTime - lastTime).toSec();
    double vx = tireVel * tireDiameter ;
    double vy = 0.0;
    double delta_theta = 0; //rightFrontAngle / dt;
    double dist = vx * dt;

    
    if( rightFrontAngle != 0.0)
    {
        // calculate turning radius
        double rightTurningRadius = rightWheelSpacing / tan(rightFrontAngle);
        // calclulate circumference of circle that would be swept
        double rightTurningCircumference = 2 * M_PI *rightTurningRadius;
        // what fraction of the above circle will I drive in this dt?
        double rightFractionOfCircumferenceSwept = dist / rightTurningCircumference;
        // what angle will I sweep in that time?
        double rightTurnAngleSwept = rightFractionOfCircumferenceSwept * (2*M_PI);
        // for now, we'll just consider this the angle we turned.
        delta_theta = rightTurnAngleSwept;
        //ROS_INFO("turnRadius: [%f]", rightTurningRadius);
    }

    theta += delta_theta;
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    odomTransform.header.stamp = currentTime;


    odomTransform.transform.translation.x += delta_x;
    odomTransform.transform.translation.y += delta_y;
    odomTransform.transform.translation.z += 0.0;
    odomTransform.transform.rotation = odom_quat;

    //send the transform
    odomTFBroadcaster.sendTransform(odomTransform);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odomTransform.transform.translation.x;
    odom.pose.pose.position.y = odomTransform.transform.translation.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = delta_theta / dt;

    //publish the message
    odomPublisher.publish(odom);
}


void kobayahsi_maru_simulation_node::spin()
{

    init();

    while (nodeHandle.ok())
    {

        currentTime = ros::Time::now();
        spinOnce();
        lastTime = currentTime;
        ros::spinOnce(); // check for incoming messages
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_simulator");

    kobayahsi_maru_simulation_node simNode;
    simNode.spin();
    return 0;
};

