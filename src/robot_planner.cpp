// task planner for project 2.
#include <queue>
#include "ros/ros.h"
#include "reactive_robot/Cartesian_Odom.h"
#include "reactive_robot/User_Task.h"

#include "std_msgs/Bool.h"

#define DEBUG true
#define HZ_ROS 15

/*
    This node will take care of monitoring the progress of the turtlebot
    developed for project 2.
*/

class RobotPlanner
{
    public:
        // This function handles the callback for a new user-specified coordinate
        void newCoordinateCallBack(reactive_robot::User_Task msg);
        void taskAchivalCallBack(std_msgs::Bool msg);
        RobotPlanner();
        bool isTaskAvailable();
        int curr_task_x, curr_task_y;
    private:
        std::queue<int> plan_x, plan_y;
        // Wether there is any possible tasks currently.
        bool plan_flag;
};

// constructor.
RobotPlanner::RobotPlanner()
{
    this->curr_task_x = 0;
    this->curr_task_y = 0;
    this->plan_flag = false;
}

void RobotPlanner::newCoordinateCallBack(reactive_robot::User_Task msg)
{
    // may not work if call back keeps fetching the same message!
    /*
        The default planning strategy is to follow the points
        as they are inputed.
    */
    this->plan_x.push(msg.x_1);
    this->plan_x.push(msg.x_2);
    this->plan_y.push(msg.y_1);
    this->plan_y.push(msg.y_2);
    this->plan_flag = true;
}

void RobotPlanner::taskAchivalCallBack(std_msgs::Bool msg)
{
    if(msg.data && !(this->plan_x.empty()))
    { // if we got a heads up to move to next task and still have some left
        this->curr_task_x = this->plan_x.front();
        this->curr_task_y = this->plan_y.front();
        this->plan_x.pop();
        this->plan_y.pop();
        this->plan_flag = true;
    }
    else if(this->plan_x.empty())
    { // if we got no more tasks left
        this->plan_flag = false;
    }
}

bool RobotPlanner::isTaskAvailable()
{
    return this->plan_flag;
}

int main(int argc, char **argv)
{
    ROS_INFO("Debug Mode %s", (DEBUG)?("On"):("Off."));
    ros::init(argc, argv, "robot_planner");
    RobotPlanner planner;
    ros::NodeHandle n;
    ros::Publisher planner_pub = n.advertise<reactive_robot::Cartesian_Odom>("/robot_planner/planner/current_task", 1000);
    // published wether there is a task in wait.
    ros::Publisher task_pub = n.advertise<std_msgs::Bool>("/robot_planner/planner/task_available", 1000);
    // subscribes to recieve any new user point inputs.
    ros::Subscriber task_sub = n.subscribe("/robot_planner/user_dirrections", 1000, &RobotPlanner::newCoordinateCallBack, &planner);
    // Subscribes to recieve wether the task was achieved, so to move to next task.
    ros::Subscriber task_a_sub = n.subscribe("/robot_planner/planner/task_achieved", 1000, &RobotPlanner::taskAchivalCallBack, &planner);
    
    ros::Rate loop_rate(HZ_ROS); // Still to see what is optimal
    while(ros::ok())
    {
        // the node will keep sending the same coordinate location
        // until fail or success in reaching it.
        if(planner.isTaskAvailable())
        { // if we have an available task, publish it.
            // Notify that there is a task available
            std_msgs::Bool msg;
            msg.data = true;
            task_pub.publish(msg);

            // Send the available task
            reactive_robot::Cartesian_Odom c_msg;
            c_msg.x = planner.curr_task_x;
            c_msg.y = planner.curr_task_y;
            planner_pub.publish(c_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}