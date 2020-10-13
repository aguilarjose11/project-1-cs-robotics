// execution monitor for project 2.
#include "ros/ros.h"
#include "reactive_robot/Cartesian_Odom.h"

#define DEBUG true
#define HZ_ROS 15

/*
    This node will take care of monitoring the progress of the turtlebot
    developed for project 2.
*/

void cartesianCallback(const reactive_robot::Cartesian_Odom msg)
{
    if(DEBUG) ROS_INFO("Cartesian Location: (%d, %d) | Angle: %f", (int)msg.x, (int)msg.y, (float)msg.dirrection);
}

int main(int argc, char **argv)
{
    ROS_INFO("Debug Mode %s", (DEBUG)?("On"):("Off."));
    ros::init(argc, argv, "execution_monitor");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cartesian_odom", 1000, cartesianCallback);
    ros::Rate loop_rate(HZ_ROS); // Still to see what is optimal
    ros::spin();

}