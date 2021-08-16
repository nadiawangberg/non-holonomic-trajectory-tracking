
# G2 - Trajectory tracking with mobile robots
Project for course TEK4030 at University of Oslo. Goal is to implement a trajectory tracking algorithm for ROS turtlebot, using algorithms from chapter 11 in Robotics Modelling, Planning and Control by Siciliano et al. All formulaes and figures below are from chapter 11 in this book. **Please see the full report TEK4030_project.pdf for further explanation**.

Turtlebot was simulated in gazebo using the environment from [simulate-in-various-worlds tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulate-in-various-world). Although I mainly used [turtlebot3_fake](http://wiki.ros.org/turtlebot3_fake) with [keyboard teleop](http://wiki.ros.org/turtlebot_teleop/Tutorials/Keyboard%20Teleop) for manual control. 

### Path Planning
from chapter 11.5.3 Path Planning in the book Robotis Modelling,Planning and Control. The "Planning via Cartesian polynomials " cubic polynomial method was implemented.


![image](https://user-images.githubusercontent.com/29915643/129525653-98ff262a-03c5-4662-8525-ca72c6cad831.png)

![image](https://user-images.githubusercontent.com/29915643/129529741-cab4328e-7616-4c48-bc57-5829967778a0.png)



### Timing Law for the trajectory
To make the path into a trajectory a timing law is needed. From chapter "11.5.4 Trajectory Planning" a timing law was implemented. This allowed the robot to complete the the trajectory within a given time. 

![image](https://user-images.githubusercontent.com/29915643/129525822-5ca20e00-dd4d-4428-b2b9-eb51a6ba344e.png)



### Trajectory tracking
From chapter "11.6.1 Trajectory Tracking" a control algorithm was implemented that made the mobile robot track the trajectory. In other words, based on the planned trajectory, reference speed and angle were calculated and sent to the robot. 

![image](https://user-images.githubusercontent.com/29915643/129526082-81237965-2513-4e9d-9771-7d96b5af0187.png)
![image](https://user-images.githubusercontent.com/29915643/129526089-209449c7-aafe-49e5-bb90-c1d3d7773525.png)

See Robotis Modelling,Planning and Control by Siciliano et al. to better understand the above formulae or the written report TEK4030_report.pdf.

### How to run

How to run sim: 

`roslaunch turtlebot3_fake turtlebot3_fake.launch`

How to run trajectory tracker: 

`rosrun traj_gen traj_gen.py 5 10 10 #T=5, ki=10, kf=10`
