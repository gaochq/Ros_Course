#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>
#include "math.h"

typedef std::pair<tf::Vector3, double> Pose_Type;

void Reset_MoveCmd(geometry_msgs::Twist &Move_cmd)
{
    Move_cmd.linear.x = 0.0;
    Move_cmd.linear.y = 0.0;
    Move_cmd.linear.z = 0.0;

    Move_cmd.angular.x = 0.0;
    Move_cmd.angular.y = 0.0;
    Move_cmd.angular.z = 0.0;
}

// Get the current Position and Yaw angular of the robot
Pose_Type Get_RobotPose()
{
    tf::TransformListener listener;
    tf::StampedTransform  transform;
    tf::Quaternion Orientation;
    tf::Vector3 Position;
    bool flag = true;
    double Roll, Pitch, Yaw;

    try
    {
        if(flag)
        {
            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(3.0));
            flag = false;
        }
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Position = transform.getOrigin();
    Orientation = transform.getRotation();
    tf::Matrix3x3(Orientation).getRPY(Roll, Pitch, Yaw);

    return std::make_pair(Position, Yaw);
}

double Normalize_angle(double angle)
{
    double Yaw = angle;
    while(Yaw>M_PIl)
        Yaw -= 2.0*M_PIl;
    while(Yaw<-M_PIl)
        Yaw += 2.0*M_PIl;
    return Yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdomOutBack_pub");
    ros::NodeHandle n;
    ros::Publisher OdomOutBack_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
    float linear_speed = 0.2;
    double goal_distance = 1.0;
    float angular_speed = 0.4;
    double goal_angle = M_PIl;
    double angular_tolerance = 0.040;

    float linear_duration = goal_distance/linear_speed;
    int rate = 100;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        Reset_MoveCmd(Move_cmd);
        for (int i = 0; i < 2; ++i)
        {
            Move_cmd.linear.x = linear_speed;
            Pose_Type Pose_tmp;
            Pose_tmp = Get_RobotPose();

            double Distance = 0;
            // Move forward
            while(Distance < goal_distance && !ros::isShuttingDown())
            {
                OdomOutBack_pub.publish(Move_cmd);
                loop_rate.sleep();
                Pose_Currenrt = Get_RobotPose();
                Distance = sqrt(pow(Pose_Currenrt.first.x() - Pose_tmp.first.x(), 2) +
                                pow(Pose_Currenrt.first.y() - Pose_tmp.first.y(), 2));
            }

            // Stop the robot
            Reset_MoveCmd(Move_cmd);
            OdomOutBack_pub.publish(Move_cmd);
            // Sleep for one second
            ros::Duration(1).sleep();

            // Rotate left 180 degrees
            Move_cmd.angular.z = angular_speed;
            double last_angle = Pose_Currenrt.second;
            double turn_angle = 0.0;
            double delta_angle;

            while(fabs(turn_angle + angular_tolerance) < fabs(goal_angle) && !ros::isShuttingDown())
            {
                OdomOutBack_pub.publish(Move_cmd);
                //loop_rate.sleep();

                Pose_Currenrt = Get_RobotPose();
                delta_angle = Normalize_angle(Pose_Currenrt.second - last_angle);
                turn_angle += delta_angle;
                last_angle = Pose_Currenrt.second;
            }
            // Stop the robot
            Reset_MoveCmd(Move_cmd);
            OdomOutBack_pub.publish(Move_cmd);
            ros::Duration(1).sleep();
        }
        // Close the node
        ros::shutdown();
    }
    return 0;
}