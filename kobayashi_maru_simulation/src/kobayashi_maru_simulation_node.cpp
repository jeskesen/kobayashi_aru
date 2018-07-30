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

    void joint_states_callback(const sensor_msgs::JointState& msg);
    void trans_vel_callback();

    double theta; // keep track of theta to save us from having to constantly convert from quaternion.
    ros::NodeHandle nodeHandle;
    ros::Subscriber jointStatesSub;
    ros::Publisher odomPublisher;

    tf::TransformBroadcaster odomTFBroadcaster;
    tf::TransformListener TFlistener;
    geometry_msgs::TransformStamped odomTransform;

    tf::Transform transform;

    ros::Time currentTime, lastTime;
    ros::Rate rate;
};

kobayahsi_maru_simulation_node::kobayahsi_maru_simulation_node() :
        rate(100)
{
    theta=0.0;
}

void kobayahsi_maru_simulation_node::init()
{
    ///Todo: read in physical characteristics of robot as params.  Hard-coded for now.

    jointStatesSub = nodeHandle.subscribe("joint_states", 10, &kobayahsi_maru_simulation_node::joint_states_callback, this);
    odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 50);

    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    currentTime = lastTime = ros::Time::now();
}

void kobayahsi_maru_simulation_node::joint_states_callback(
        const sensor_msgs::JointState& msg)
{
}

void kobayahsi_maru_simulation_node::spinOnce()
{

    tf::StampedTransform rightWheelTransform, leftWheelTransform;
    try{
      TFlistener.lookupTransform("right_rear_wheel", "right_front_wheel",
                               ros::Time(0), rightWheelTransform);
      TFlistener.lookupTransform("left_rear_wheel", "left_front_wheel",
                               ros::Time(0), leftWheelTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // I'm only going to worry about the right wheel for now.  Eventually, I will incorporate both.
    double rightAngle = rightWheelTransform.getRotation().getZ();
    double rightWheelSpacing = rightWheelTransform.getOrigin().getX();

    //compute odometry using the turning radius
    double dt = (currentTime - lastTime).toSec();
    double vx = 0.1;
    double vy = 0.0;
    double delta_theta = 0.0;
    double dist = vx * dt;

    if( rightAngle != 0.0)
    {
        // calculate turning radius
        double rightTurningRadius = rightWheelSpacing * sin(rightAngle);
        // calclulate circumference of circle that would be swept
        double rightTurningCircumference = 2 * M_PI *rightTurningRadius;
        // what fraction of the above circle will I drive in this dt?
        double rightFractionOfCircumferenceSwept = dist / rightTurningCircumference;
        // what angle will I sweep in that time?
        double rightTurnAngleSwept = rightFractionOfCircumferenceSwept * (2*M_PI);
        // for now, we'll just consider this the angle we turned.
        delta_theta = rightTurnAngleSwept;
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

        ros::spinOnce(); // check for incoming messages
        currentTime = ros::Time::now();
        spinOnce();
        lastTime = currentTime;
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
