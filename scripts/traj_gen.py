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
    y = None 
    x_dot = None
    y_dot = None
    theta = None
    theta_dot = None

curr_odom = Odom() #curr pose
goal_odom = Odom() #end of trajectory pose
init_odom = Odom() #start of trajectory pose
curr_waypoint = Odom() #pose of current trajectory waypoint

path_index = 0
step = 0

init_time = None

def quaternion_to_euler(quaternion):
    quaternion_squared = quaternion ** 2
    phi = np.arctan2(2*quaternion[3]*quaternion[2]+2*quaternion[0]*quaternion[1],quaternion_squared[0] - quaternion_squared[1] - quaternion_squared[2] + quaternion_squared[3])  # TODO: Convert from quaternion to euler angles
    theta = np.arcsin(2*(quaternion[0]*quaternion[2]-2*quaternion[1]*quaternion[3]))  # TODO: Convert from quaternion to euler angles
    psi =  np.arctan2(2*quaternion[1]*quaternion[2]+2*quaternion[0]*quaternion[3],quaternion_squared[0] + quaternion_squared[1] - quaternion_squared[2] - quaternion_squared[3])

    euler_angles = np.array([phi, theta, psi])
    
    return euler_angles #np.array

def quat_to_yaw(quat): #works as the quaternion represents rotations around z axis only (only yaw)
    q = Quaternion(quat.w, quat.x, quat.y, quat.z)
    q_np = np.array([quat.w, quat.x, quat.y, quat.z])
    #print(q.axis)

    euler_angles = quaternion_to_euler(q_np)
    return euler_angles[2] #q.radians#can also do q.radians (TODO, should double check that this work / use ssa(angle))

def odomCB(odom_msg):
    global curr_odom
    curr_odom.x = odom_msg.pose.pose.position.x
    curr_odom.y = odom_msg.pose.pose.position.y
    curr_odom.x_dot = odom_msg.twist.twist.linear.x 
    curr_odom.y_dot = odom_msg.twist.twist.linear.y
    curr_odom.theta = quat_to_yaw(odom_msg.pose.pose.orientation)
    curr_odom.theta_dot = odom_msg.twist.twist.angular.z #angular velocity

def goalCB(goal_msg):
    global goal_odom, init_odom, step
    init_time = rospy.Time.now()

    goal_odom.x = goal_msg.pose.position.x
    goal_odom.y = goal_msg.pose.position.y
    goal_odom.theta = quat_to_yaw(goal_msg.pose.orientation)
    
    init_odom.x = curr_odom.x
    init_odom.y = curr_odom.y
    init_odom.theta = curr_odom.theta
    step = 0

def CubicCartesianPolynomial(init_pose, goal_pose, s_arr, k): #find path
    backward = False # k in (0,1)
    N = len(s_arr)

    x_arr = [None]*N
    y_arr = [None]*N
    th_arr = [None]*N
    x_dash_arr = [None]*N
    y_dash_arr = [None]*N
    v_tilde_arr = [None]*N
    w_tilde_arr = [None]*N

    #consts
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

    #initial values
    x_dash_arr[0] = k*np.cos(thi)
    y_dash_arr[0] = k*np.sin(thi)
    v_tilde_arr[0] = k
    w_tilde_arr[0] = w_tilde_arr[1] = 0 #TODO, this is just a guess
    th_arr[0] = thi

    pose_list = []

    for i in range(N): #i is the index of s
        s = s_arr[i]
        x_arr[i] = s**3*xf - (s-1)**3*xi + alpha_x*s**2*(s-1) + beta_x*s*(s-1)**2
        y_arr[i] = s**3*yf - (s-1)**3*yi + alpha_y*s**2*(s-1) + beta_y*s*(s-1)**2
        
        if i > 0:
            ds = (s_arr[i]-s_arr[i-1])
            x_dash = (x_arr[i] - x_arr[i-1])/ds
            y_dash = (y_arr[i] - y_arr[i-1])/ds
            th_arr[i] = np.arctan2(y_dash, x_dash) + backward*np.pi

            x_dash_arr[i] = x_dash
            y_dash_arr[i] = y_dash
            y_dash2x = (y_dash_arr[i] - y_dash_arr[i-1])/ds
            x_dash2x = (x_dash_arr[i] - x_dash_arr[i-1])/ds

            v_tilde_arr[i] = (-1)**backward * np.sqrt(x_dash**2 + y_dash**2) #samme som v_d bortsett fra at pos derivert brukt i steden
            w_tilde_arr[i] = (y_dash2x*x_dash-x_dash2x*y_dash)/(x_dash**2+y_dash**2)

        #generate array of poses
        pos = toPose(x_arr[i], y_arr[i], th_arr[i])
        pose_list.append(pos)
        pose_arr = toPoseArr(pose_list)

    #convert to ros pose array
    return pose_arr, v_tilde_arr, w_tilde_arr 

def ssa(angle): #in radian
    return (angle+np.pi)%(2*np.pi)-np.pi

def toPose(x,y,theta):
    pos = Pose()
    pos.position.x = x
    pos.position.y = y

    orient = Quaternion(axis = (0.0, 0.0, 1.0), radians = theta)
    pos.orientation.w = orient.w
    pos.orientation.x = orient.x
    pos.orientation.y = orient.y
    pos.orientation.z = orient.z
    return pos

def toPoseArr(pose_list):
    pose_arr = PoseArray()
    pose_arr.header.frame_id = 'odom'
    pose_arr.header.stamp = rospy.Time.now()
    pose_arr.poses = pose_list
    return pose_arr

def finderror(xd, yd, thd, x,y,th): #input given in world coordinates
    # error in world frame (cartesian error)
    e1_w = xd - x
    e2_w = yd - y
    e3_w = thd - th

    e1 = e1_w*np.cos(th) + e2_w*np.sin(th)
    e2 = -e1_w*np.sin(th) + e2_w*np.cos(th)
    e3 = e3_w

    return e1, e2, e3 #output rotated to align with body

def planner():
    T = 10.0 #path should take 5seconds
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    traj_debug_pub = rospy.Publisher('traj_debug', PoseArray, queue_size=10)

    rospy.init_node('planner_node', anonymous=True)
    
    #Subscrubur
    rospy.Subscriber("odom", Odometry, odomCB)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCB)
    time_i = rospy.Time.now().to_sec()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if goal_odom.x != None: #We have a goal
            #PLANNING
            s_arr = list(np.arange(0,1,1/T))
            path_k = 15 #tuning param for path
            pose_arr, v_arr, w_arr = CubicCartesianPolynomial(init_odom, goal_odom, s_arr, path_k)
            traj_debug_pub.publish(pose_arr)

            #CONTROL
            """
            e1, e2, e3 = finderror(goal_odom.x, goal_odom.y, goal_odom.theta, curr_odom.x, curr_odom.y, curr_odom.theta)
            zeta = 1 #relative damping ratio
            a = 1 #w0, natural frequency
            k1 = 2*zeta*a
            k3 = k1
            k2 = (a**2 - wd**2) / vd
            u1 = -k1*e1
            u2 = -k2*e2 -k3*e3
            cmd_v = vd*np.cos(e3)-u1
            cmd_w = wd-u2
            """

            #Naive controller
            cmd_vel = Twist()
            timepassed = rospy.Time.now().to_sec() - time_i
            print(timepassed)
            global step
            if (timepassed >= 1.0 and step < T-1): #1 sek between
                step += 1
                time_i = rospy.Time.now().to_sec()
                vd = (1/T)*v_arr[step]
                wd = (1/T)*w_arr[step]
                cmd_vel.linear.x = vd
                cmd_vel.angular.z = wd
                cmd_vel_pub.publish(cmd_vel)



        rate.sleep()

if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
