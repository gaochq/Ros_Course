#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "math.h"


void Reset_MoveCmd(geometry_msgs::Twist &Move_cmd)
{
    Move_cmd.linear.x = 0.0;
    Move_cmd.linear.y = 0.0;
    Move_cmd.linear.z = 0.0;

    Move_cmd.angular.x = 0.0;
    Move_cmd.angular.y = 0.0;
    Move_cmd.angular.z = 0.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TimeOutBack_pub");
    ros::NodeHandle n;
    ros::Publisher TimeOutBack_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    geometry_msgs::Twist Move_cmd;
    float linear_speed = 0.2;
    float goal_distance = 1.0;
    float angular_speed = 1.0;
    float goal_angle = M_PIl;
    float linear_duration = goal_distance/linear_speed;
    int rate = 50;

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        Reset_MoveCmd(Move_cmd);
        for (int i = 0; i < 2; ++i)
        {
            Move_cmd.linear.x = linear_speed;
            // Move forward
            int ticks = static_cast<int>(linear_duration*rate);
            for (int j = 0; j < ticks; ++j)
            {
                TimeOutBack_pub.publish(Move_cmd);
                //  Sleep for the time remaining to let us hit our 10Hz publish rate
                loop_rate.sleep();
            }

            // Stop the robot
            Reset_MoveCmd(Move_cmd);
            TimeOutBack_pub.publish(Move_cmd);
            // Sleep for one second
            ros::Duration(1).sleep();

            // Rotate left 180 degrees
            Move_cmd.angular.z = angular_speed;
            ticks = static_cast<int>(goal_angle*rate);
            for (int k = 0; k < ticks; ++k)
            {
                TimeOutBack_pub.publish(Move_cmd);
                loop_rate.sleep();
            }

            // Stop the robot
            Reset_MoveCmd(Move_cmd);
            TimeOutBack_pub.publish(Move_cmd);
            ros::Duration(1).sleep();
        }
        // Close the node
        ros::shutdown();
    }
    return 0;
}
