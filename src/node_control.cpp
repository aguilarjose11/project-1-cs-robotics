// Project 1 Inteligent Robotics
// Follows schema theory.
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"
#include "reactive_robot/Cartesian_Odom.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <math.h>
#include <stdlib.h>
#include <vector>


// Debuging mode
#define DEBUG true
// Offset of the kinect inside the turtlebot
#define OFFSET_TBII double(0.3)
// starting location of robot (2, 2).
const int START_LOCATION[] = {2, 2};


// Offset of the turtlebot's kinect laser scan. (it cannot detect anything below 0.5: too close!)
// Topics to use:
/*
    /mobile_base/events/bumper
     - gives wether we have come across an object.
    /odom
     - odometry data to know where the robot is at.
    /scan
     - Gives data from laser as of proximity of objects.
    
Required actions:
1.- halt if colision detected by bumper
2. accept keyboard movement commands from human user: use the given teleops?
3. escape from symmetric obstacles within 1 ft in front (anything closer than 1ft we shoul turn away from it)
4. avoid asymmetric obstacles within 1ft (avoid anything within 1 ft like 3)
5. turn randomly as the cruise behaviour (random inside 0-15 degrees) after every 1ft of movement
6. drive forward as a part of cruise behaviour
*/


/*
    Notes on Project 2:
    
    /odom topic can be pretty useful to identify where to go, where we are at.
    info:
        - Uses quartenion coordinates. formulas:
        angle: (arccos(w)*2)*180/PI -> angle in degrees.
        back to quartelion: cos((angle * PI) / 180 * 2) = cos(angle * PI / 360)
        - pose.pose.orientation
            - 360 or 0
                - +x
            - 270
                - +y
            - 180
                - -x
            - 90
                - -y
            

*/


// Global variables:
// these flags are used as schema theory.
// as they get activated, so the command to be executed
bool BUMP_FLAG = false;
bool OBSTACLE_FLAG = false;
bool ESCAPE_FLAG = false; // symmetrical obstacle
bool AVOID_FLAG = false; //asymmetric obstacle
bool LEFT_OBSTACLE = false;
bool RIGHT_OBSTACLE = false;
// KEY TELEOP.
// These are special flag as to tell which keys were selected in the teleop
bool MOVE_FORWARD = false;
bool MOVE_BACKWARDS = false;
bool TURN_LEFT = false;
bool TURN_RIGHT = false;
// Task planner global variables.
bool TASK_AVAILABLE = false;
int CURR_TASK_X = 100;
int CURR_TASK_Y = 100;

// function to return the smallest values within an array of unknown size
// Used to create the obstacle "map"
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
    // Always return true. always on.
    return true;
}

// Returns wether we have bumped against something.
void bumpCheck(const kobuki_msgs::BumperEvent msg)
{
    // If we bump into something, call command.
     BUMP_FLAG = (msg.state)?(true):(false);
     ROS_INFO("bump state: %d", msg.state);
}


// Call back

/*
This class stores and updates the global flags. the data that is 
handled is mainly laser scan comming from the /scan topic
*/
class RobotObstacleDetection
{
    public:
        // constructor.
        RobotObstacleDetection();
        // Takes care of the call backs from topic
        void CallBack(sensor_msgs::LaserScan msg);
        // accessor methods
        float getClosestValue() {return this->min_val;}
        float getAngleOfClosest() {return this->angle_rad;}
        float getAreaVision() {return this->total_area_vision;}
        // As the data is analysed, returns "peaks" of close obstacles
        int getNextObstacle(sensor_msgs::LaserScan msg);
    private:
        float angle_increment, min_val, angle_rad, total_area_vision;
        unsigned int angle_loc, ranges_len;
        bool continual_flag;
};

/*
As data is analysed left to right, the function looks at the change in distance of the obstacles
Once the function detects an obstacle closer than 1 foot, it keeps scanning until the distance starts
increasing: which means that we reached the closest point of the obstacle, so we can assume that
we have detected a whole obstacle.
*/
int RobotObstacleDetection::getNextObstacle(sensor_msgs::LaserScan msg)
{
    // only returns minima within 1 foot.
    bool obst_flag = false;
    while(this->angle_loc < this->ranges_len && std::isnan(msg.ranges.at(this->angle_loc))) // check that we arent already in a minima from a previous obstacle
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
                loc_obst = this->angle_loc; // set new obstacle's location
                continue;
            }
            else
            {
                this->continual_flag = true; // This lets us know that we have encounter an obstacle, so more could come.
                break; // new small value is not smaller than previous. we passed the pinnacle. break.
            }  
        }
    }
    return loc_obst;
}

/*
Constructor.
Initializes the member variables and fetches a first message from the odometry topic
*/
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

// The call back function. Grabs message, disects it, and places data where it belongs. Turns flags on or off given the data.
void RobotObstacleDetection::CallBack(sensor_msgs::LaserScan msg)
{
    // Vector to contain the locations(ray beam number) of every obstacle
    std::vector<double> obstacle_locations;
    // get the increment per location
    this->angle_increment = msg.angle_increment;
    // reset the initial scanning variable for the data back to the begining (leftmost beam)
    this->angle_loc = 0;
    // reset the min_val to leftmost beam (similar to a minimum algorithm)
    this->min_val = msg.ranges.at(this->angle_loc);
    // initializes the initial angle in radians
    this->angle_rad = msg.angle_min + (this->angle_loc * this->angle_increment);
    // Gets the total range of vision
    this->total_area_vision = abs(msg.angle_min) + abs(msg.angle_max);
    // gets the number of beams that were shot
    this->ranges_len = msg.ranges.size();
    // resets the continual flag (turn on when we have found an obstacle and more could come.)
    this->continual_flag = false;
    while(true) // get all posible obstacles.
    {
        int val = this->getNextObstacle(msg);
        if(val == -1)
        {
            break;// getNextObstacle returns -1 when no new obstacles were found
        }
        obstacle_locations.push_back((double(val) * this->angle_increment) + msg.angle_min);
    }
    // reset flags
    LEFT_OBSTACLE = false;
    RIGHT_OBSTACLE = false;
    ESCAPE_FLAG = false;
    AVOID_FLAG = false;
    //if(DEBUG) ROS_INFO("Number of Obstacles: %d", (int) obstacle_locations.size());
    if(obstacle_locations.size() == 0)
    {
        return; // if no obstacles, then, keep flags false
    }
    for(int i = 0; i < obstacle_locations.size(); i++) // for every obstacle that was detected...
    {
        if(obstacle_locations.at(i) < 0) // mainly on the left.
        {
            RIGHT_OBSTACLE = true;
        }
        else // mainly on the right.
        {
            LEFT_OBSTACLE = true;
        }
        
    }
    if(LEFT_OBSTACLE && RIGHT_OBSTACLE) // if obstacles on both sides, we have a symmetrical situation
    {
        ESCAPE_FLAG = true;
    }
    else if(LEFT_OBSTACLE || RIGHT_OBSTACLE) // if obstacles mainly on one side, we have an asymmetrical situation
    {
        AVOID_FLAG = true;
    }
    if(DEBUG) ROS_INFO("Obstacles | Left: %d, Right: %d", (LEFT_OBSTACLE)?(1):(0), (RIGHT_OBSTACLE)?(1):(0));
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

/*
this class takes care of the gathering, analysis, and subsequent raise of flags for required commands.
It works by initially storing an initial position and keeping track of subsequent positions. returns
the distance between the initial and the subsequent via the getDistanceTraveled.
The initial location is reset to the current via the setLoc.
*/
class RobotOdometry
{
    public:
        // Recieve message from subscribed node and store in pose variable 
        void recieveMsg(const nav_msgs::Odometry msg)
        {
            // Store most recent position
            this->curr_loc = msg.pose;
            float feet_x = msg.pose.pose.position.x * 3.2808, feet_y = msg.pose.pose.position.y * 3.2808;
            // if > 0.5, then, we are closer to the next point than the main digit is
            this->loc_x = round(feet_x);
            this->loc_y = round(feet_y);
        }  
        
        // sets the latest location as the new saved reference position.
        void setLoc()
        {
            this->saved_loc = this->curr_loc;
        }
        // get distance traveled from the point set to the given by recieve Msg
        double getDistanceTraveled()
        {
            // get locations for the current position
            double x_c = this->curr_loc.pose.position.x, y_c = this->curr_loc.pose.position.y;
            // get location of the stored position.
            double x_s = this->saved_loc.pose.position.x, y_s = this->saved_loc.pose.position.y;
            ROS_INFO("saved: x: %f, y: %f", this->saved_loc.pose.position.x, this->saved_loc.pose.position.y);

            // distance formula: assuming a 2d movement coordinated
            //   _________________________________
            // \/ (x_2 - x_1)^2 + (y_2 - y_1)^2   |
            return sqrt(pow(x_c - x_s, 2) + pow(y_c - y_s, 2));
        }
        // Get the angle faced in degrees
        double getFacedAngle()
        {
            // (arccos(w)*2)*180/PI = acos(w) * 360 / PI
            /*
            - 360 or 0
                - +x
            - 270
                - +y
            - 180
                - -x
            - 90
                - -y
            */
           double w_val = this->curr_loc.pose.orientation.w;
           double normalizing_factor = this->curr_loc.pose.orientation.z;
           if(normalizing_factor < 0)
           {
               return (acos(-w_val) * 360.0) / M_PI;
           }
           else
           {
               return (acos(w_val) * 360.0) / M_PI;
           }
           
            
        }
        void getCartesianPos(int c_pos[])
        {
            // created static to avoid removal from the stack once function is done.
            c_pos[0] = this->loc_x;
            c_pos[1] = this->loc_y;
            return;
        }
        // Constructor
        float getAngleToPoint(int x, int y);
        RobotOdometry();
        

    private:
        // curr_loc contains the updated location.
        // saved_loc contains the last point stored to get the distance.
        geometry_msgs::PoseWithCovariance curr_loc, saved_loc;
        // The cartesian location for the robot.
        int loc_x, loc_y;
        // Sets the current given location. CALLBACK FUNCTION
        void setLoc(const nav_msgs::Odometry msg)
        {
            // store the new position to be used as a reference.
            this->saved_loc = msg.pose;

        }
};

/*
    Returns the angle to the point from the current position.
*/
float RobotOdometry::getAngleToPoint(int x, int y)
{
    int loc[2];
    getCartesianPos(loc);
    float angle_rad = atan2(y - loc[1], x - loc[0]);
    if(angle_rad >= 0)
    {
        return angle_rad * 180 / M_PI;
    }
    else
    {
        return (2*M_PI + angle_rad)*180/M_PI;
    }
}


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
    // Fetch a message
    ros::NodeHandle m;
    boost::shared_ptr<nav_msgs::Odometry const> sharedEdge;
    nav_msgs::Odometry edge;
    sharedEdge = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", m);
    if(sharedEdge != NULL)
    {
        edge = *sharedEdge;
    }
    // updates the member pointers with the initial data
    this->saved_loc = edge.pose;
    this->curr_loc = edge.pose;
    this->loc_x = START_LOCATION[0];
    this->loc_y = START_LOCATION[1];
    if(DEBUG) ROS_INFO("Constructor initial values: x: %f, y: %f", this->saved_loc.pose.position.x, this->saved_loc.pose.position.y);
}

/*
Note: The schema theory program depicted in Ex. 4 uses a parallel approach.
Instead
*/

// callback function for the teleoperative panel
void teleop_callback(geometry_msgs::Twist msg)
{
    MOVE_BACKWARDS = false; // +1
    MOVE_FORWARD = false; // -1
    TURN_LEFT = false; // 1
    TURN_RIGHT = false; // -1
    if(msg.linear.x > 0) // move forward
    {
        MOVE_FORWARD = true;
    }
    if(msg.linear.x < 0) // move backwards
    {
        MOVE_BACKWARDS = true;
    }
    if(msg.angular.z > 0) // turning left
    {
        TURN_LEFT = true;
    }
    if(msg.angular.z < 0) // turning right
    {
        TURN_RIGHT = true;
    }
    return;
}


/*
    function used to update task available flag
*/
void tAvailCallBack(std_msgs::Bool msg)
{
    TASK_AVAILABLE = msg.data;
}

/*
    Function used to update the current task being followed.
*/
void taskCallBack(reactive_robot::Cartesian_Odom msg)
{
    if(TASK_AVAILABLE)
    {
        CURR_TASK_X = msg.x;
        CURR_TASK_Y = msg.y;
    }
    // if(DEBUG) ROS_INFO("Task: (%d, %d) - Actual (zzzzz", CURR_TASK_X, CURR_TASK_Y);
}


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
        // cartesian_odom: Allows to communicate the cartesian location with the monitor.
        ros::Publisher cartesian_odom = n.advertise<reactive_robot::Cartesian_Odom>("/cartesian_odom", 1000);
        // Publish on progress with tasks to planner.
        ros::Publisher task_progress = n.advertise<std_msgs::Bool>("/robot_planner/planner/task_achieved", 1000);
        //
        ros::Publisher monitor_pub = n.advertise<std_msgs::String>("/robot_planner/planner/monitor", 1000);
        //
        ros::Publisher task_error = n.advertise<std_msgs::Bool>("/robot_planner/planner/task_error", 1000);

        // odom_sub: gives us the location and movement information of the turtlebot. uses nav_msgs/Odometry
        ros::Subscriber odom_sub = n.subscribe("/odom", 1000, &RobotOdometry::recieveMsg, &robot_odom);
        // bump_sub: its state variable is 1 when bumps, 0 when not.
        ros::Subscriber bump_sub = n.subscribe("/mobile_base/events/bumper", 1000, bumpCheck);
        // scan_sub: the laser scan topic contains the information on the closeness of objects.
        ros::Subscriber scan_sub = n.subscribe("/scan", 1000, &RobotObstacleDetection::CallBack, &robot_obst);
        // teleop_sub subscribes to the "remapped" topic for the turtlebot_teleop package.
        ros::Subscriber teleop_sub = n.subscribe("/teleop_control/key_capturer/cmd_vel", 1000, teleop_callback);
        // Used to update wether we got a task available
        ros::Subscriber t_avail_sub = n.subscribe("/robot_planner/planner/task_available", 1000, tAvailCallBack);
        // 
        ros::Subscriber task_sub = n.subscribe("/robot_planner/planner/current_task", 1000, taskCallBack);
        
        // rate at which the program is running/publishing: 5 hz
        // ;Therefore, 5 revolutions per second: 1/5 seconds
        const int HZ_ROS = 15; // 
        const double ROBOT_SPEED = 0.25; // whats robot speed 0 - 1.
        const double RADIAN_TOP_SPEED = ((HZ_ROS * 15 * M_PI)/180); // multiply
        const float ERROR_VAL = 2; // acceptable error in dirrection.

        ros::Rate loop_rate(HZ_ROS); // Still to see what is optimal
        unsigned short count = 0; // counts the number of times we went thourgh a loop.
        int pos_or_neg = 0;
        int timer  = 0; // This will be the counter for checking the time
        int timer_2 = 0;
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
               if(TASK_AVAILABLE && !AVOID_FLAG && !ESCAPE_FLAG)
               {// gotta follow a point in map.
                    // calculate the direction to face.
                    float angle = robot_odom.getAngleToPoint(CURR_TASK_X, CURR_TASK_Y);
                    if(DEBUG) printf("Global X: %d | Global Y: %d\n", CURR_TASK_X, CURR_TASK_Y);
                    float curr_angle = robot_odom.getFacedAngle();
                    // if(abs(robot_odom.getFacedAngle()) > ERROR_VAL)
                    geometry_msgs::Twist tmp_msg;
                    // Calculate the direction of turn.
                    float diff_angle = curr_angle + 180;
                    float differential = 1;
                    float offset = 0;
                    if(diff_angle > 360)
                    {
                        offset = abs(360 - diff_angle); // 0 -> offset
                        if(angle >= curr_angle || angle <= offset)
                        {
                            // in positive area
                            differential = 1;
                        }
                        else
                        {
                            // in negative area
                            differential = -1;
                        }
                    }
                    else
                    {
                        if(angle >= curr_angle && angle <= diff_angle)
                        {
                            // in positive area
                            differential = 1;
                        }
                        else
                        {
                            // in negative area
                            differential = -1;
                        }
                        
                    }
                    
                    
                    tmp_msg.angular.z = double(differential);
                    while(abs(robot_odom.getFacedAngle() - angle) > ERROR_VAL && ros::ok())
                    {
                        if(DEBUG) printf("Robot Angle: %f | target: %f\n", robot_odom.getFacedAngle(), angle);
                        motorDriver(motor_pub, tmp_msg);
                        ros::spinOnce();
                    }
                    tmp_msg.angular.z = double(0.0);
                    motorDriver(motor_pub, tmp_msg);
                    int robot_loc[2];
                    robot_odom.getCartesianPos(robot_loc);
                    if(abs(robot_loc[0] - CURR_TASK_X) <= 1 && abs(robot_loc[1] - CURR_TASK_Y) <= 1 && !timer_2--)
                    {
                        // We have made it to the location!
                        timer_2 = 5;
                        std_msgs::Bool b_msg;
                        b_msg.data = true;
                        task_progress.publish(b_msg);
                        ros::spinOnce();
                    }
                    else
                    { // Still traveling...
                        msg.linear.x = double(ROBOT_SPEED);
                    }

               }
           }

            // symmetrical. turn around 180 degrees
            if(ESCAPE_FLAG || timer)
            {
                if(TASK_AVAILABLE)
                {
                    // Whoops, we have had an issue! report to say so!
                    std_msgs::String msg_str;
                    std_msgs::Bool msg_err;
                    msg_err.data = true;
                    msg_str.data = "Task unable to be fullfilled!\n";
                    monitor_pub.publish(msg_str);
                    task_error.publish(msg_err);

                }
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
            // Just because there is an obstacle does it mean that we are doomed!
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
            // teleop commands
           if(MOVE_FORWARD)
           {
               msg.linear.x = double(ROBOT_SPEED);
               msg.angular.z = double(0);
           }
           if(MOVE_BACKWARDS)
           {
               msg.linear.x = double(-ROBOT_SPEED);
               msg.angular.z = double(0);
           }
           if(TURN_LEFT)
           {
               msg.angular.z = double(1.0);
               msg.linear.x = float(0);
           }
           if(TURN_RIGHT)
           {
               msg.angular.z = double(-1.0);
               msg.linear.x = float(0);
           }
            // If we have bumped with something: stop
           if(BUMP_FLAG)
           {
                msg.linear.x = double(0.0);
                msg.angular.z = double(0.0);
                if(TASK_AVAILABLE)
                {
                    // Whoops, we have had an issue! report to say so!
                    std_msgs::String msg_str;
                    std_msgs::Bool msg_err;
                    msg_err.data = true;
                    msg_str.data = "Task unable to be fullfilled!\n";
                    monitor_pub.publish(msg_str);
                    task_error.publish(msg_err);
                }
           }

            //if(DEBUG) ROS_INFO("Distance Traveled from origin: %f", robot_odom.getDistanceTraveled());

            motorDriver(motor_pub, msg); // officially submit what the robot will do next
            reactive_robot::Cartesian_Odom c_msg;
            c_msg.dirrection = robot_odom.getFacedAngle();
            int c_pos[2];
            robot_odom.getCartesianPos(c_pos);
            c_msg.x = c_pos[0];
            c_msg.y = c_pos[1];
            cartesian_odom.publish(c_msg);
            ros::spinOnce(); // call the subscriber functions to update the flags.
            loop_rate.sleep();
        }
        
        return 0;
}
