Project 1 -- CS-4023
====================

Notes:
-------

1. The tutorials are _actually_ pretty helpfull.

TODO:
-----

 - [x] Create Package and basic folders.
 - [x] Figure out how to create a world as specified by the project (See Note 1).
 - [x] Add basic C++ code (Figure out how, Note 1).
 - [x] Follow Dr. hougen's HW example from 2003 and see if it is actually useful.
 - [x] Other?

Project 2 -- cs-4023
=====================

TODO:
-----

 - change default action of robot to go forward and report new location based on the movement
  - Check when we reach a foot of movement forward or we reach a new point?
 - have a new node start to monitor progress (execution_monitor)
 - create a new node to accept new location inputs (task_input)
 - connect nodes together
 - in main program, create a function to calculate the direction to which to move to in a
   straight line. forget about walls for now.
 - if bounce to wall, do action and immediatelly recalculate current location and direction to
 travel
 - test with a few experimental places.



Installing the package:
-----------------------
1. inside catkin workspace, go to src. (see the tutorials from homework 1)
2. Do `git clone https://github.com/aguilarjose11/project-1-cs-robotics.git`
3. Rename the folder to "reactive_robot"
4. go back to the catkin workspace and do `catkin_make`

__It is possible that you may run accross issues. If so, I will try to help__


Good videos:
------------
 - [The construct: Launch a world with gazebo](https://www.youtube.com/watch?v=qi2A32WgRqI)
 - [The construct: Add models to gazebo](https://www.youtube.com/watch?v=tIJRxkaAZtA)
 - [The construct: Spawn a robot](https://www.youtube.com/watch?v=dy3JKUtH5zk)

Useful Commands:
----------------

`catkin_make`
 - Used for rebuilding packages.
 
 Design Notes:
 -------------
 
 We will create a .launch file that will launch the following in order:
 1. Launch the turtlebot_gazebo with the specified world.
 2. Launch our main node where we will do the solution in c++ for the project.
 
 #### The solution for the project.
 
 The following are topics that we may need.
 * /mobile_base/events/bumper
  * Detects bumps with the `kobuki_msgs/BumperEvent`
  * message details:
    * state changes to 1 when bumped, 0 otherwise.
    * bumper stays as 1
 
 * /camera/depth/points?
  * We need to figure out how to get depth.
  
