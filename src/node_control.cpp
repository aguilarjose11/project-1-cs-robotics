// Project 1 Inteligent Robotics
// Follow schema theory.
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"

#include <math.h>
#include <stdlib.h>
#include <vector>

#define DEBUG true
#define OFFSET_TBII double(0.5)
// Offset of the turtlebot's kinect laser scan. (it cannot detect anything below 0.5: too close!)
// Topics to use:
/*
    /mobile_base/events/bumper
     - gives wether we have come across an object.
    /odom
     - odometry data to know where the robot is at.
    
/*
1.- halt if colision detected by bumper
2. accept keyboard movement commands from human user: use the given teleops?
3. escape from symmetric obstacles within 1 ft in front (anything closer than 1ft we shoul turn away from it)
4. avoid asymmetric obstacles within 1ft (avoid anything within 1 ft like 3)
5. turn randomly as the cruise behaviour (random inside 0-15 degrees) after every 1ft of movement
6. drive forward as a part of cruise behaviour
*/

// Global variables:
bool BUMP_FLAG = false;
bool OBSTACLE_FLAG = false;
bool ESCAPE_FLAG = false; // symmetrical obstacle
bool AVOID_FLAG = false; //asymmetric obstacle
bool LEFT_OBSTACLE = false;
bool RIGHT_OBSTACLE = false;

// function to return the smallest values within an array of unknown size
// but assumed to not be empty, null, nor undefined.
// Returns by refference the place where it was found in the array.
template <class T>
T getMinVal(std::vector<T> values, unsigned int* place)
{
    int len = values.size();
    T min_val = float(10.0); // default: given from the max range possible. see msg
    *place = 0;
    for(int i = 0; i < len; i++)
    {
        if(values.at(i) != NAN && values.at(i) < min_val)
        {
            min_val = values.at(i);
            *place = i;
        }
    }
    ROS_INFO("min distance: %f", float(min_val));
    return min_val;
}

// Cruise mode is the default modality for the robot
// This functions returns wether to continue doing cruise
// This is the case ALWAYS since this is the default
// activity the robot is expected to do; hence the return true.
bool cruise_mode()
{
    // Always return true.
    return true;
}

// Returns wether we have bumped against something.
void bumpCheck(const kobuki_msgs::BumperEvent msg)
{
    // If we bump into something, call command.
     BUMP_FLAG = (msg.state)?(true):(false);
     ROS_INFO("bump state: %d", msg.state);
}
/*
This next section will talk about the if the robot gets into a collison 
and what to do if that happens 
global variables:
escapeAlert // this will be the flag that goes true if there is a need to esacpe else this is false

escapeSense // This will basically say where the collison is located 
escapeCommand // this will  tell which way to go to leave the collision
*/
void escape()
{
/*
This method will use  awhile loop
This method will check all of the bumpers on the bot 
if any bumper detetcts anything:
    escape alert = True
    else escape_alert = False
*/
    return;
}

void escapeLocator()
{
    /*

    Note of changed:
    The message in charge of the bumper doesn't carry multiple bumpers, but
    rather it simply tells wether the bumper hit something. no need to be 
    checking which bumper was activated. once we bump on something, state 
    in the mesage turns to 1, 0 otherwise.

    This method will basically detect where the collision is based off off 
    of which bumper is sending a signal 
    This method will have a while loop 
    This method will check each bumper
    if right and left bumpers are detecting something:
        escapseSense = front collision
    else if bumpLeft:
        escapseSense = left collision
    else if bump right:
        escapeSense = right collison
    else if bumpBack:
        escapeSense = rear colliosion
        */
    return;
}

void escapeMotorDirection() // might change function name later
{
/*
This method will tell the motor which way to go to escape the collision 
This method will have  a while loop 
first checks if escape alert is on:
    then checks which bumper detects something using if statements 
    then if any bumper is on: 
    3 things happen:
    escape alert is set to true 
    a dircetion to escape the collision is given and then a sleep method is 
    called such as sleep(# in sec I beleive)
    Logic:
    if escapeSense = forward collison:
        escape alert = true 
        command is to go back
        then sleep for .2 sec
        then command is to go left 
        then sleep .2 sec
    else if escapeSense = left collision:
        escape alert = true
        command is to go right 
        then sleep .2 sec
    else if escapeSense = right collision:
        esacpeAlert  = true
        command is to go left
        then sleep .2 sec
    else if escapeSense = rear collision:
        escapeAlert = true 
        command is go left 
        then sleep .2 sec 
*/
    return;
}
/*
These following methods will correlate to sensing and avoiding 
the obstacles
global variable names we will use :
avoid_alert // this willed for the direction to go to avoid 
avoid vision // This wil be the alert saying that something needs to be avoided 
avoid // this will be usl be for sensors if an obstacle is spotted 
*/
// Call back

class RobotObstacleDetection
{
    public:
        RobotObstacleDetection();
        // Takes care of the call backs
        void CallBack(sensor_msgs::LaserScan msg);
        float getClosestValue() {return this->min_val;}
        float getAngleOfClosest() {return this->angle_rad;}
        float getAreaVision() {return this->total_area_vision;}
        int getNextObstacle(sensor_msgs::LaserScan msg);
    private:
        float angle_increment, min_val, angle_rad, total_area_vision;
        unsigned int angle_loc, ranges_len;
        bool continual_flag;
};

int RobotObstacleDetection::getNextObstacle(sensor_msgs::LaserScan msg)
{
    // only returns minima within 1 foot.
    bool obst_flag = false;
    while(this->angle_loc < this->ranges_len && isnan(msg.ranges.at(this->angle_loc))) // check that we arent already in a minima from a previous obstacle
    {
        (this->angle_loc)++;
    } // go until we either get to the end of the vector or get outside the 1 ft distance
    while(this->angle_loc < this->ranges_len && this->continual_flag && msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII))
    {
        (this->angle_loc)++; // we have already found an obstacle. probably this is it still that one.
    }
    int loc_obst = -1;
    for(; this->angle_loc < ranges_len; (this->angle_loc) += 1) // move until the end of the vector. may not reach it tho.
    {
        
        if(msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII) && !obst_flag) // if we have found a close obstacle
        {
            obst_flag = true; // start checking until we find something not as a minimum
            loc_obst =  this->angle_loc;
            continue;
        }
        if(msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII) && obst_flag) // only smaller numbers from now on.
        {
            if(msg.ranges.at(loc_obst) >= msg.ranges.at(this->angle_loc)) // we find a new smaller value
            {
                loc_obst = this->angle_loc; // set new obstacle
                continue;
            }
            else
            {
                this->continual_flag = true;
                break; // new small value is not smaller than previous. we passed the pinnacle. break.
            }  
        }
    }
    return loc_obst;
}

RobotObstacleDetection::RobotObstacleDetection()
{
    // grab a single message
    ros::NodeHandle m;
    boost::shared_ptr<sensor_msgs::LaserScan const> sharedEdge;
    sensor_msgs::LaserScan msg;
    sharedEdge = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", m);
    if(sharedEdge != NULL)
    {
        msg = *sharedEdge;
    }
    this->angle_increment = msg.angle_increment;
    this->angle_loc = 0;
    this->min_val = msg.ranges.at(this->angle_loc);
    this->angle_rad = msg.angle_min + (this->angle_loc * this->angle_increment); // where was the closest object found
    this->total_area_vision = abs(msg.angle_min) + abs(msg.angle_max);
    this->ranges_len = sizeof(msg.ranges)/sizeof(msg.ranges[0]);
}
void RobotObstacleDetection::CallBack(sensor_msgs::LaserScan msg)
{
   std::vector<double> obstacle_locations;
    this->angle_increment = msg.angle_increment;
    this->angle_loc = 0;
    this->min_val = msg.ranges.at(this->angle_loc);
    this->angle_rad = msg.angle_min + (this->angle_loc * this->angle_increment); // where was the closest object found
    this->total_area_vision = abs(msg.angle_min) + abs(msg.angle_max);
    this->ranges_len = msg.ranges.size();
    this->continual_flag = false;
    while(true) // get all posible obstacles.
    {
        int val = this->getNextObstacle(msg);
        if(val == -1)
        {
            break;
        }
        obstacle_locations.push_back((double(val) * this->angle_increment) + msg.angle_min);
    }
    // reset flags
    LEFT_OBSTACLE = false;
    RIGHT_OBSTACLE = false;
    ESCAPE_FLAG = false;
    AVOID_FLAG = false;
    ROS_INFO("Number of Obstacles: %d", (int) obstacle_locations.size());
    if(obstacle_locations.size() == 0)
    {
        return; // if no obstacles, then, keep flags false
    }
    for(int i = 0; i < obstacle_locations.size(); i++)
    {
        if(obstacle_locations.at(i) < 0) // mainly on the left.
        {
            LEFT_OBSTACLE = true;
        }
        else // mainly on the right.
        {
            RIGHT_OBSTACLE = true;
        }
        
    }
    if(LEFT_OBSTACLE && RIGHT_OBSTACLE)
    {
        ESCAPE_FLAG = true;
    }
    else if(LEFT_OBSTACLE || RIGHT_OBSTACLE)
    {
        AVOID_FLAG = true;
    }
    ROS_INFO("Obstacles | Left: %d, Right: %d", (LEFT_OBSTACLE)?(1):(0), (RIGHT_OBSTACLE)?(1):(0));
    //
    return;

}

void obstacleDetector(sensor_msgs::LaserScan msg)
{
/*
This will be to detect a obstacle, it will have a variable that will be 
equal to the ir detector
 {there will be a while loop used in this method} **not anymore. loop will be in main.**
if the detector detects something left or right or in front it will alert 
the variable called obstacle alert making it True
else 
obstacle alert will stay at False
*/
    
    return;
}
void avoidDirection()
{
    /*
    This method will have a while loop(so while(1)), it will aslo use
    the ir detector, then there will be nested if-else statemnets 
    it will be as such:
    while(1)
    {
        if avoid_alert is true, then it will check each direction
        if left = alert then the left ir sensor comes on
        else if front = alert then front sensor turns on 
        else if right = alert then right sensor turns on
        May need to add an else at the end not sure yet if so then 
        im assuming that would be else avoid_alert = false
    }
    */
    return;
}
void motorDirection()
{
    // I took the liberty to change some of the varibales in the exercise i will write the 
    // names here and explain 
    // avoidCommand = gTW (This menas go this way so it tells the motor which way to go)
    // avoidPercept = obstLoc (This notifiyng the motor where the obstacle is )
    // obstacleAhead = forwardSpot (says if obsatcle is ahead)
    // obstacleLeft = leftSpot 
    // obstacleRight = rightSpot
  /*
  This method will tell the motor which way the bot needs to go 
  will have  while loop 
  inside of while loop:
  if alert is true:
    if obstLoc = fowardSpot:
        gTW = left
    else if obstLoc = leftSpot:
        gTW = right
    else if obstLoc = rightSpot:
        gTW = left
  */
    return;
}

void motorDriver(ros::Publisher pub, geometry_msgs::Twist msg)
{
    /*
    will take care of doing the movement of the wheels. will recieve the amount to be
    moved and will convert into a message and then publish to the respective topic

    Publishes to: /mobile_base/commands/velocity
    msg type: geometry_msgs/Twist
    */
   pub.publish(msg);
   return;
}

class RobotOdometry
{
    public:
        // Recieve message from subscribed node and store in pose variable 
        void recieveMsg(const nav_msgs::Odometry msg)
        {
            // Store most recent position
            this->curr_loc = msg.pose;
        }  
        
        // sets the latest location as the new saved reference position.
        void setLoc()
        {
            this->saved_loc = this->curr_loc;
        }
        // get distance traveled from the point set to the given by recieve Msg
        double getDistanceTraveled()
        {
            double x_c = this->curr_loc.pose.position.x, y_c = this->curr_loc.pose.position.y;
            double x_s = this->saved_loc.pose.position.x, y_s = this->saved_loc.pose.position.y;
            ROS_INFO("saved: x: %f, y: %f", this->saved_loc.pose.position.x, this->saved_loc.pose.position.y);

            // distance formula: assuming a 2d movement coordinated
            //   _________________________________
            // \/ (x_2 - x_1)^2 + (y_2 - y_1)^2   |
            return sqrt(pow(x_c - x_s, 2) + pow(y_c - y_s, 2));
        }
        // Constructor
        RobotOdometry();
        

    private:
        // curr_loc contains the updated location.
        // saved_loc contains the last point stored to get the distance.
        geometry_msgs::PoseWithCovariance curr_loc, saved_loc;
        // Sets the current given location. CALLBACK FUNCTION
        void setLoc(const nav_msgs::Odometry msg)
        {
            // store the new position to be used as a reference.
            this->saved_loc = msg.pose;

        }
};

/*
Note on subscribe:
Subscribe can be used in two modalities:
regular callback static function:
subscribe(topic, queue size, callback)
member function callback:
subscribe(topic, queue size, pointer to general function, object to call with)
*/

RobotOdometry::RobotOdometry()
{
    ros::NodeHandle m;
    boost::shared_ptr<nav_msgs::Odometry const> sharedEdge;
    nav_msgs::Odometry edge;
    sharedEdge = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", m);
    if(sharedEdge != NULL)
    {
        edge = *sharedEdge;
    }
    this->saved_loc = edge.pose;
    this->curr_loc = edge.pose;
    ROS_INFO("Constructor initial values: x: %f, y: %f", this->saved_loc.pose.position.x, this->saved_loc.pose.position.y);
}

/*
Note: The schema theory program depicted in Ex. 4 uses a parallel approach.
Instead
*/

int main(int argc, char **argv)
{
        // Initializes this ros node
        ros::init(argc, argv, "node_control");
        
        // Used to create the publisher and the subscribers.
        ros::NodeHandle n;
        
        RobotOdometry robot_odom;
        RobotObstacleDetection robot_obst;

        // motor_pub: to allow us to move the turtlebot. Uses geometry_msgs/Twist
        ros::Publisher motor_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);


        // odom_sub: gives us the location and movement information of the turtlebot. uses nav_msgs/Odometry
        ros::Subscriber odom_sub = n.subscribe("/odom", 1000, &RobotOdometry::recieveMsg, &robot_odom);
        // bump_sub: its state variable is 1 when bumps, 0 when not.
        ros::Subscriber bump_sub = n.subscribe("/mobile_base/events/bumper", 1000, bumpCheck);
        // scan_sub: the laser scan topic contains the information on the closeness of objects.
        ros::Subscriber scan_sub = n.subscribe("/scan", 1000, &RobotObstacleDetection::CallBack, &robot_obst);


        // rate at which the program is running/publishing: 5 hz
        // ;Therefore, 5 revolutions per second: 1/5 seconds
        const int HZ_ROS = 5; // 
        const double ROBOT_SPEED = 0.25; // whats robot speed 0 - 1.
        const double RADIAN_TOP_SPEED = ((HZ_ROS * 15 * M_PI)/180); // multiply

        ros::Rate loop_rate(HZ_ROS); // Still to see what is optimal
        unsigned short count = 0; // counts the number of times we went thourgh a loop.
        int pos_or_neg = 0;
        int timer  = 0; // This will be the counter for checking the time

        /*
        we can only tell how fast to go forward/backward or turning.
        */

        while(ros::ok()) // ros::ok allows to loop while ros is still "fine" (covers exiting by ctrl+c)
        {
            /*
            Here is where the we will Run each command checker and check the "Global flags"
            to see what is the motor to do next.
            
            */
           // this contains what the robot does next
           /*
           geometry_msgs/Twist:
            // we actually use 2
           angular {x, y, z} ONLY z
           linear {x, y, z} ONLY x 
           */
           geometry_msgs::Twist msg;
           // Run every single command to check wether the robot is all good.

           // check the global variables and "override" the msg variable that contains the directions 
           // of movement that the robot will do next.
           if(cruise_mode()) // if(true)
           {
               // rotation is measured in radias/second
               // between 0 - 15 degrees: 0 rads - 0.261799 rads (15{degrees}*PI/180)
               // velocity: 0 - 0.261799 * 5 = 75*PI/180
                msg.linear.x = double(ROBOT_SPEED);
                // check if we have moved 1 foot to know if we need 
                // keep turning for the next two cycles.
                // getDistanceTraveled == 0.0
                if(robot_odom.getDistanceTraveled() >= double(0.3048) || count != 0) // if we have traveled more than a foot.
                {
                    if(count == 0)
                    {
                        pos_or_neg = (int)(rand()%2);
                    }
                    msg.linear.x = double(0.0);
                    msg.angular.z = double((pos_or_neg == 0)?(1):(-1));
                    robot_odom.setLoc(); // reset travel distance to 0
                }
           }

            // symmetrical. turn around 180 degrees
            if(ESCAPE_FLAG || timer)
            {
                
             if (timer == 0)
             {

                 timer = 15; // This should give us close to 180 degrees if i use the ratio ised earlier

             }
             else 
             {
                 timer--; // timer will start to go down this affects 
             }
             msg.linear.x = double(0.0);
             msg.angular.z = double(1.0);
            }

            // asymmetrical turn away from the danger
            if(AVOID_FLAG && !timer)
            {
                // first thing is to see if obstacle is left or right 
                if(LEFT_OBSTACLE)
                {
                    
                    msg.angular.z = double(-1);
                    msg.linear.x = double(0.0);
                     //makes it go right
                
                }
                else if (RIGHT_OBSTACLE)
                {
                    msg.angular.z = double(1); // this is for when the robot is trying to turn left 
                    msg.linear.x = double(0.0);
                }
            }
               // go away of the obstacle!
/*               msg.linear.x = double(0.0);
               msg.angular.z = double(-1.0);*/
           // If we have bumped with something: stop
           if(BUMP_FLAG)
           {
               msg.linear.x = double(0.0);
               msg.angular.z = double(0.0);
           }

            ROS_INFO("Distance Traveled from origin: %f", robot_odom.getDistanceTraveled());

            motorDriver(motor_pub, msg); // officially submit what the robot will do next
            ros::spinOnce(); // call the subscriber functions to update the flags.
            loop_rate.sleep();
        }
        
        return 0;
}
