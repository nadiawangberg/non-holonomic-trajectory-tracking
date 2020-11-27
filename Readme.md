
# G2 - Trajectory tracking with mobile robots
Project for course TEK4030 at University of Oslo. Goal is to implement a trajectory tracking algorithm for ROS turtlebot, using algorithms from chapter 11 in Robotics Modelling, Planning and Control by Siciliano et al. 

Turtlebot was simulated in gazebo using https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulate-in-various-world. Although I mainly used http://wiki.ros.org/turtlebot3_fake and http://wiki.ros.org/turtlebot_teleop/Tutorials/Keyboard%20Teleop for manual control. 


### Path Planning
from chapter 11.5.3 Path Planning in the book Robotis Modelling,Planning and Control. The "Planning via Cartesian polynomials " cubic polynomial method was implemented. 

### Timing Law for the trajectory
To make the path into a trajectory a timing law is needed. From chapter "11.5.4 Trajectory Planning" a timing law was implemented. 

### Trajectory tracking
From chapter "11.6.1 Trajectory Tracking" a control algorithm was implemented that made the mobile robot track the trajectory. 

### How to run

`roslaunch turtlebot3_fake turtlebot3_fake.launch`

`rosrun traj_gen traj_gen.py 5 10 10 #meaning T=5, ki=10, kf=10`
