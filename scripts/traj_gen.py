#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist, PoseArray
from pyquaternion import Quaternion
import numpy as np

class Odom: #for a 3DOF mobile robot
    x = None
    y = None #no z pos
    x_dot = None
    y_dot = None
    theta = None # heading initialized to 0
    theta_dot = None

curr_odom = Odom()
goal_odom = Odom()
no_path = True

def quat_to_yaw(quat): #works as the quaternion represents rotations around z axis only (only yaw)
    q = Quaternion(quat.w, quat.x, quat.y, quat.z)
    return q.radians #can also do q.radians (TODO, should double check that this work / use ssa(angle))

def odomCB(odom_msg):
    global curr_odom
    curr_odom.x = odom_msg.pose.pose.position.x
    curr_odom.y = odom_msg.pose.pose.position.y
    curr_odom.x_dot = odom_msg.twist.twist.linear.x 
    curr_odom.y_dot = odom_msg.twist.twist.linear.y
    curr_odom.theta = quat_to_yaw(odom_msg.pose.pose.orientation)
    curr_odom.theta_dot = odom_msg.twist.twist.angular.z #angular velocity

def goalCB(goal_msg):
    global goal_odom
    goal_odom.x = goal_msg.pose.position.x
    goal_odom.y = goal_msg.pose.position.y
    goal_odom.theta = quat_to_yaw(goal_msg.pose.orientation)

def CubicCartesianPolynomial(init_pose, goal_pose, s_arr, k): #find path
    backward = False # k in (0,1)
    N = len(s_arr)

    x_arr = [None]*N
    y_arr = [None]*N
    th_arr = [None]*N

    xi = init_pose.x
    yi = init_pose.y
    thi = init_pose.theta

    xf = goal_pose.x
    yf = goal_pose.y
    thf = goal_pose.theta

    alpha_x = k*np.cos(thf) - 3*xf
    alpha_y = k*np.sin(thf) - 3*yf
    beta_x = k*np.cos(thi) + 3*xi
    beta_y = k*np.sin(thi) + 3*yi

    for i in range(N):
        s = s_arr[i]
        x_arr[i] = s**3*xf - (s-1)**3*xi + alpha_x*s**2*(s-1) + beta_x*s*(s-1)**2
        y_arr[i] = s**3*yf - (s-1)**3*yi + alpha_y*s**2*(s-1) + beta_y*s*(s-1)**2
        th_arr[i] = np.arctan2(y_arr[i], x_arr[i]) + backward*np.pi
    return x_arr, y_arr, th_arr

def noe(s_arr):
    N = len(s_arr)
    th_arr = [None]*N
    for i in range(len(s_arr)):
        th


def talker():
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    traj_debug_pub = rospy.Publisher('traj_debug', PoseArray, queue_size=10)

    rospy.init_node('traj_gen', anonymous=True)
    
    #Subscrubur
    rospy.Subscriber("odom", Odometry, odomCB)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCB)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if goal_odom.x != None: #We have a goal
            error_float = goal_odom.x - curr_odom.x
            cmd_vel = Twist()
            Kp = 2
            cmd_vel.linear.x = Kp*error_float
            cmd_vel_pub.publish(cmd_vel)

            global no_path
            if no_path:
                #create path

                s_arr = list(np.arange(0,1,0.01))
                x_arr, y_arr, th_arr = CubicCartesianPolynomial(curr_odom, goal_odom, s_arr, 1)
                pose_arr = []

                #Quaternion(axis=(1.0, 0.0, 0.0), radians=math.pi/2)

                for i in range(len(x_arr)):
                    pos = Pose()
                    pos.position.x = x_arr[i] #i
                    pos.position.y = y_arr[i] #1
                    pose_arr.append(pos) 
                

                discrete_path = PoseArray()
                discrete_path.header.frame_id = 'odom'
                discrete_path.poses = pose_arr
                traj_debug_pub.publish(discrete_path)
                no_path = False
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
