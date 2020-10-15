// execution monitor for project 2.
#include "ros/ros.h"
#include "reactive_robot/Cartesian_Odom.h"
#include "reactive_robot/User_Task.h"
#include <stdio.h>

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
    ros::init(argc, argv, "user_input");
    ros::NodeHandle n;
    ros::Publisher user_dirs = n.advertise<reactive_robot::User_Task>("/robot_planner/user_dirrections", 1000);
    
    ros::Rate loop_rate(HZ_ROS); // Still to see what is optimal
    printf("Robot task input\n");
    printf("Enter task in the following format:\n\n");
    printf("Where x_1, x_2 are the start points and x_2, y_2 are the end points as ints.\n");
    printf("(Example) task >>>: ((x_1, y_1), (x_2, y_2))\n\n");
    while(ros::ok())
    {
        printf("task >>>: ");
        int x_1, x_2, y_1, y_2;
        int flag = scanf(" ((%d, %d), (%d, %d))", &x_1, &y_1, &x_2, &y_2);
        if(flag == EOF)
        {
            // end of file aka. ^D entered.
            break;
        }
        else if(flag != 4)
        { // incorrect input's format.
            printf("Incorrect input. Input should be of the format ((x_1, y_1), (x_2, y_2)). All ints\n");
        }
        else
        {
            if(DEBUG) ROS_INFO("Origin location: (%d, %d) | Destination location: (%d, %d).\n", x_1, y_1, x_2, y_2);
            reactive_robot::User_Task msg;
            msg.x_1 = x_1;
            msg.y_1 = y_1;
            msg.x_2 = x_2;
            msg.y_2 = y_2;
            user_dirs.publish(msg);
        }

        loop_rate.sleep();
    }

}