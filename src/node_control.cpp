// Project 1 Inteligent Robotics
// Follow schema theory.
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


// Topics to use:
/*
    /mobile_base/events/bumper
     - gives wether we have come across an object.
    /odom
     - odometry data to know where the robot is at.
    
/*
1.- halt if colision detected by bumper
2. accept keyboard movement commands from human user
3. escape from symmetric obstacles within 1 ft in front (anything closer than 1ft we shoul turn away from it)
4. avoid asymmetric obstacles within 1ft (avoid anything within 1 ft like 3)
5. turn randomly as the cruise behaviour (random inside 0-15 degrees) after every 1ft of movement
6. drive forward as a part of cruise behaviour
*/

// Global Variables


void cruise_mode()
{
    // 

}
void bumpCheck()
{
    
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
}

void escapeLocator()
{
    /*
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
}
/*
These following methods will correlate to sensing and avoiding 
the obstacles
global variable names we will use :
avoid_alert // this willed for the direction to go to avoid 
avoid vision // This wil be the alert saying that something needs to be avoided 
avoid // this will be usl be for sensors if an obstacle is spotted 
*/
void obstacleDetector()
{
/*
This will be to detect a obstacle, it will have a variable that will be 
equal to the ir detector
there will be a while loop used in this method 
if the detector detects something left or right or in front it will alert 
the variable called obstacle alert making it True
else 
obstacle alert will stay at False
*/
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
}

float getMovedDistance(geometry_msgs::Pose prev_loc, geometry_msgs::Pose prev_loc)
{ // note: Use a class???
    /*
    return the distance traveled from the previous point to the current one.
    return in ft. REMEMVER TO CONVERT TO ft FROM m (the default SI unit used in ros)
    */
    
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
        
        // motor_pub: to allow us to move the turtlebot. Uses geometry_msgs/Twist
        ros::Publisher motor_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
        // rate at which the program is running/publishing: 10 hz
        ros::Rate loop_rate(10); // Still to see what is optimal
        unsigned long long count = 0; // counts the number of times we went thourgh a loop.
        while(ros::ok()) // ros::ok allows to loop while ros is still "fine" (covers exiting by ctrl+c)
        {
            /*
            Here is where the we will Run each command checker and check the "Global flags"
            to see what is the motor to do next.
            
            */
           geometry_msgs::Twist msg;
           // Run every single command to check wether the robot is all good.

           // check the global variables and "override" the msg variable that contains the directions 
           // of movement that the robot will do next.
           if()
            
            motorDriver(motor_pub, msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
}
         /*       // create a message to publish.
                geometry_msgs::Twist msg;
                msg.linear.x = double(1.0);
                msg.angular.z = double(1.0);
                // Say on the console running the node the info.
                ROS_INFO("Not Sure how to convert into str.");
                // actually publish
                chatter_pub.publish(msg);
                // used for call backs.
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        }
        return 0;
}
*/