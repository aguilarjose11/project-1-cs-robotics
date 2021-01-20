// execution monitor for project 2.
#include "ros/ros.h"
#include "reactive_robot/Cartesian_Odom.h"
#include "reactive_robot/User_Task.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <vector>

#define DEBUG true
#define HZ_ROS 15
#define V 11

/*
    This node will take care of monitoring the progress of the turtlebot
    developed for project 2.
*/

void dijkstra(int graph[V][V], int src, int j);
void printPath(int parent[], int j);

/**
 * 1 -> Nielsen hall
 * 2 -> Elaine Bizzell Thompson Garden
 * 3 -> Richards hall
 * 4 -> OU Logo
 * 5 -> Front of Bizzell library
 * 6 -> Entrance to Bizzell Library
 * 7 -> Far view of Price College of Business
*/
const std::vector<int> GOAL_POINTS = {0, 2, 3, 5, 6, 9, 10, 8};

const std::vector<std::vector<int>> MAP_POINTS = {
    {2, 24, 2 , 25, 42, 42, 40, 58, 58, 24, 24},
    {2, 4 , 40, 12, 4 , 16, 40, 40, 76, 40, 76}
};

int DISTANCE_MATRIX[V][V] = {
    {0 , 24, 40, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
    {24, 0 , 0 , 12, 16, 0 , 0 , 0 , 0 , 0 , 0},
    {40, 0 , 0 , 36, 0 , 0 , 0 , 0 , 0 , 22, 0},
    {0 , 12, 36, 0 , 0 , 0 , 30, 0 , 0 , 0 , 0},
    {0 , 16, 0 , 0 , 0 , 16, 0 , 0 , 0 , 0 , 0},
    {0 , 0 , 0 , 0 , 16, 0 , 24, 0 , 0 , 0 , 0},
    {0 , 0 , 0 , 30, 0 , 24, 0 , 22, 0 , 12, 0},
    {0 , 0 , 0 , 0 , 0 , 0 , 22, 0 , 36, 0 , 0},
    {0 , 0 , 0 , 0 , 0 , 0 , 0 , 36, 0 , 0 , 34},
    {0 , 0 , 22, 0 , 0 , 0 , 12, 0 , 0 , 0 , 36},
    {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 34, 36, 0},
};

std::vector<int> CURRENT_PATH;

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
    unsigned int main_menu_sel = 0;
    printf("Touring Robot Interface\n");
    while(ros::ok())
    {
        printf("Please enter Task to perform:\n");
        printf("(1) Follow Touring Program.\n\t(2) Get Directions.\n\t\t(3) Exit.\n\n>>>: ");
        int scan_flag = scanf("%u", &main_menu_sel);
        if(scan_flag == EOF)
        {
            break; // end of file.
        }
        else if(scan_flag == 0)
        {
            // No matching in input.
            ROS_INFO("Invalid input. Try again.\n");
            continue;
        }
        switch(main_menu_sel)
        {
            case 1:
            {
                // Follow Touring Program
                printf("On development\n");
                break;
            }
            case 2:
            {
                // get somewhere
                printf("Select Destination:\n(0) Go back\n(1) Got to Starting Point\n(2) Nielsen Hall\n(3) Elaine Bizzell Thompson Garden\n(4) Richards hall\n(5) OU logo\n(6) Best View of Bizzell Library\n(7) Front Entrance ot Bizzell Library\n(8) Far View of Price College of Business\n\n>>>: ");
                unsigned int goal_place = 0;
                scan_flag = scanf("%u", &goal_place);
                if(scan_flag == EOF || scan_flag == 0 || goal_place > 8)
                { 
                    printf("Invalid input. Invalid choice.\n");
                    break;
                }
                else if(!goal_place)
                {
                    // Return to main menu!.
                    break;
                }
                // lets find the goal place
                int goal_node = GOAL_POINTS.at(goal_place-1);
                // Find the closest location
                // get current odom location
                boost::shared_ptr<nav_msgs::Odometry const> sharedEdge;
                nav_msgs::Odometry edge;
                sharedEdge = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
                if(sharedEdge != NULL)
                {
                    edge = *sharedEdge;
                }
                else
                {
                    // Odometry was  not obtained!
                    ROS_ERROR("Odometry Error!");
                    return(1);
                }
                double curr_x = edge.pose.pose.position.x, curr_y = edge.pose.pose.position.y;
                // loop through every possible point and check which one is closes to current location
                double min_dist = INFINITY;
                int closest_node = 0;
                for(int i = 0; i < V; i++)
                {
                    if(std::sqrt(std::pow((MAP_POINTS[0][i] - curr_x), 2) + std::pow((MAP_POINTS[1][i] - curr_y), 2)) < min_dist)
                    {
                        min_dist = std::sqrt(std::pow((MAP_POINTS[0][i] - curr_x), 2) + std::pow((MAP_POINTS[1][i] - curr_y), 2));
                        closest_node = i;
                    }
                }
                // if robot is not at hoped for location, we will start by going there!
                // Find fastest path.
                dijkstra(DISTANCE_MATRIX, closest_node, goal_node);
                // command for fastest path.
                for(int point_set = 0; point_set < CURRENT_PATH.size(); point_set++)
                {
                    int x = MAP_POINTS.at(0).at(CURRENT_PATH.at(point_set));
                    int y = MAP_POINTS.at(1).at(CURRENT_PATH.at(point_set));
                    if(DEBUG) ROS_INFO("Destination location: (%d, %d).\n", x, y);
                    reactive_robot::User_Task msg;
                    msg.x_1 = x;
                    msg.y_1 = y;
                    msg.x_2 = x;
                    msg.y_2 = y;
                    user_dirs.publish(msg);
                }

                // We published all of the goal points, go back to menu.


                break;
            }
            case 3:
            {
                // exit
                printf("Exiting program...\n");
                return 0;
            }
            default:
            {
                // Invalid input
                fprintf(stderr, "Invalid input %u. Use a valid option.\n", main_menu_sel);
                break;
            }
        }
        if(main_menu_sel == 1)
        {
            // on development
            continue;
        }

        loop_rate.sleep();
    }

}

int minDistance(int dist[], bool sptSet[]) 
{ 
      
    // Initialize min value 
    int min = INT_MAX, min_index; 
  
    for (int v = 0; v < V; v++) 
        if (sptSet[v] == false && 
                   dist[v] <= min) 
            min = dist[v], min_index = v; 
  
    return min_index; 
} 


void dijkstra(int graph[V][V], int src, int j)
{
    // The output array. dist[i] 
    // will hold the shortest 
    // distance from src to i 
    int dist[V];  
  
    // sptSet[i] will true if vertex 
    // i is included / in shortest 
    // path tree or shortest distance  
    // from src to i is finalized 
    bool sptSet[V]; 
  
    // Parent array to store 
    // shortest path tree 
    int parent[V]; 
  
    // Initialize all distances as  
    // INFINITE and stpSet[] as false 
    for (int i = 0; i < V; i++) 
    { 
        parent[0] = -1; 
        dist[i] = INT_MAX; 
        sptSet[i] = false; 
    } 
  
    // Distance of source vertex  
    // from itself is always 0 
    dist[src] = 0; 
  
    // Find shortest path 
    // for all vertices 
    for (int count = 0; count < V - 1; count++) 
    { 
        // Pick the minimum distance 
        // vertex from the set of 
        // vertices not yet processed.  
        // u is always equal to src 
        // in first iteration. 
        int u = minDistance(dist, sptSet); 
  
        // Mark the picked vertex  
        // as processed 
        sptSet[u] = true; 
  
        // Update dist value of the  
        // adjacent vertices of the 
        // picked vertex. 
        for (int v = 0; v < V; v++) 
  
            // Update dist[v] only if is 
            // not in sptSet, there is 
            // an edge from u to v, and  
            // total weight of path from 
            // src to v through u is smaller 
            // than current value of 
            // dist[v] 
            if (!sptSet[v] && graph[u][v] && 
                dist[u] + graph[u][v] < dist[v]) 
            { 
                parent[v] = u; 
                dist[v] = dist[u] + graph[u][v]; 
            }  
    } 
  
    // print the constructed 
    // distance array 
    // Create vector containing path:
    CURRENT_PATH.clear();
    printPath(parent, j);

}

void printPath(int parent[], int j) 
{ 
      
    // Base Case : If j is source 
    if (parent[j] == - 1) 
        return; 
  
    printPath(parent, parent[j]); 
  
    CURRENT_PATH.push_back(j); 
} 