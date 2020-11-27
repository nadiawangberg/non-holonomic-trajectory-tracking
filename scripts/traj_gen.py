#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist, PoseArray
import numpy as np

class Odom: #for a 3DOF mobile robot
    x = None
    y = None 
    theta = None

curr_odom = Odom() #curr pose, X
odom_d = Odom() # reference odom, Xd

goal_odom = Odom() #end of trajectory pose, Xf
init_odom = Odom() #start of trajectory pose, X0

vd, wd = 0,0 #desired vel and angular vel in body frame
step = 0


#Quaterion functions
def quaternion_to_euler(quaternion):
    quaternion_squared = quaternion ** 2
    phi = np.arctan2(2*quaternion[3]*quaternion[2]+2*quaternion[0]*quaternion[1],quaternion_squared[0] - quaternion_squared[1] - quaternion_squared[2] + quaternion_squared[3])
    theta = np.arcsin(2*(quaternion[0]*quaternion[2]-2*quaternion[1]*quaternion[3]))
    psi =  np.arctan2(2*quaternion[1]*quaternion[2]+2*quaternion[0]*quaternion[3],quaternion_squared[0] + quaternion_squared[1] - quaternion_squared[2] - quaternion_squared[3])

    euler_angles = np.array([phi,theta,psi])
    return euler_angles

def quat_to_yaw(quat): 
    #assumes rot around z only
    q_np = np.array([quat.w, quat.x, quat.y, quat.z])
    euler_angles = quaternion_to_euler(q_np)
    return euler_angles[2]

def angleaxis_to_quat(axis, angle):
    ax,ay,az = axis
    qx = ax*np.sin(angle/2)
    qy = ay*np.sin(angle/2)
    qz = az*np.sin(angle/2)
    qw = np.cos(angle/2)

    return [qw,qx,qy,qz]

#ROS callback functions
def odomCB(odom_msg):
    global curr_odom
    curr_odom.x = odom_msg.pose.pose.position.x
    curr_odom.y = odom_msg.pose.pose.position.y
    curr_odom.theta = quat_to_yaw(odom_msg.pose.pose.orientation)

def goalCB(goal_msg):
    global goal_odom, init_odom, step

    goal_odom.x = goal_msg.pose.position.x
    goal_odom.y = goal_msg.pose.position.y
    goal_odom.theta = quat_to_yaw(goal_msg.pose.orientation)
    
    init_odom.x = curr_odom.x
    init_odom.y = curr_odom.y
    init_odom.theta = curr_odom.theta
    step = 0

def CubicCartesianPolynomial(init_pose, goal_pose, s_arr, kf, ki, backward): #find path
    # Ch11.5.3 - Path planning via Cartesian polynomials
    N = len(s_arr)

    x_arr = [None]*N
    y_arr = [None]*N
    th_arr = [None]*N
    x_dash_arr = [None]*N
    y_dash_arr = [None]*N
    v_tilde_arr = [None]*N
    w_tilde_arr = [None]*N

    #consts
    xi, yi, thi = init_pose.x, init_pose.y, init_pose.theta
    xf, yf, thf = goal_pose.x, goal_pose.y, goal_pose.theta 

    alpha_x = kf*np.cos(thf) - 3*xf
    alpha_y = kf*np.sin(thf) - 3*yf

    beta_x = ki*np.cos(thi) + 3*xi
    beta_y = ki*np.sin(thi) + 3*yi

    #initial values
    x_dash_arr[0] = ki*np.cos(thi)
    y_dash_arr[0] = ki*np.sin(thi)
    v_tilde_arr[0] = ki
    w_tilde_arr[0] = w_tilde_arr[1] = 0
    th_arr[0] = thi

    pose_list = []
    for i in range(N):
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
        pos = toPose(x_arr[i], y_arr[i], ssa(th_arr[i]))
        pose_list.append(pos)

    return pose_list, v_tilde_arr, w_tilde_arr 

def ssa(angle):
    #finds the closest signed angle (radians)
    return (angle+np.pi)%(2*np.pi)-np.pi

def toPose(x,y,theta):
    pos = Pose()
    pos.position.x = x
    pos.position.y = y

    orient = angleaxis_to_quat([0,0,1], theta)
    pos.orientation.w = orient[0]
    pos.orientation.x = orient[1]
    pos.orientation.y = orient[2]
    pos.orientation.z = orient[3]
    return pos

def toPoseArr(pose_list):
    pose_arr = PoseArray()
    pose_arr.header.frame_id = 'odom'
    pose_arr.header.stamp = rospy.Time.now()
    pose_arr.poses = pose_list
    return pose_arr

def find_error(ref_odom, curr_odom):
    # Transforms an error in world frame, to an error aligned with body frame

    xd, yd, thd = ref_odom.x, ref_odom.y, ref_odom.theta
    x, y, th = curr_odom.x, curr_odom.y, curr_odom.theta

    e1_w = xd - x
    e2_w = yd - y
    e3_w = ssa(thd - th)

    e1 = e1_w*np.cos(th) + e2_w*np.sin(th)
    e2 = -e1_w*np.sin(th) + e2_w*np.cos(th)
    e3 = e3_w

    return e1,e2,e3

def control_lin(e1,e2,e3,  zeta,a,  wd,vd):
    #Control based o approximate linearization, 11.6.1

    #(11.69)
    k1 = 2*zeta*a
    k2 = (a**2 - wd**2) / vd
    k3 = k1

    u1 = -k1*e1 #(11.66)
    u2 = -k2*e2 -k3*e3 #(11.67)

    cmd_v = vd*np.cos(e3)-u1 #(11.62)
    cmd_w = wd-u2 #(11.63)

    return cmd_v,cmd_w


def planner(T, ki, kf, backward):
    rospy.init_node('planner_node', anonymous=True)

    #Publishers
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    traj_debug_pub = rospy.Publisher('traj_debug', PoseArray, queue_size=10)
    
    #Subscribers
    rospy.Subscriber("odom", Odometry, odomCB)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCB)
    time_i = rospy.Time.now().to_sec()

    hz = 30
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        if goal_odom.x != None: #We have a goal
            
            #PLANNING
            ds = 1/(hz*T)
            dsdtau = 1/T
            s_arr = list(np.arange(0,1,ds))
            N = len(s_arr)
           
            pose_list, v_tilde_arr, w_tilde_arr = CubicCartesianPolynomial(init_odom, goal_odom, s_arr, kf, ki, backward)
            traj_debug_pub.publish(toPoseArr(pose_list))

            #CONTROL
            timepassed = rospy.Time.now().to_sec() - time_i
            global odom_d,vd,wd,step
            if (timepassed >= 1/hz and step < N-1):
                step += 1
                time_i = rospy.Time.now().to_sec()

                #set new refernce waypoint
                odom_d.x = pose_list[step].position.x
                odom_d.y = pose_list[step].position.y
                odom_d.theta = quat_to_yaw(pose_list[step].orientation)

                w_tild = w_tilde_arr[step]
                v_tild = v_tilde_arr[step]
                
                #Timing law
                vd = dsdtau*1/T*v_tild # (11.54)
                wd = dsdtau*1/T*w_tild # (11.55)


            #Feedback controller
            zeta = 1 # 1 for critical;  < 1 more stable more lame overdamped;   > 1 more unstable but faster underdamped
            a = 1 # natural frequency; 
            e1,e2,e3 = find_error(odom_d, curr_odom)
            cmd_v,cmd_w = control_lin(e1,e2,e3,  zeta,a,  wd,vd)

            cmd_vel = Twist()
            cmd_vel.linear.x = cmd_v
            cmd_vel.angular.z = cmd_w
            cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv) > 3:
            print("Using specified parameters")
            T = float(sys.argv[1])
            ki = float(sys.argv[2])
            kf = float(sys.argv[3])

            if (ki<0) and (kf<0):
                backward = True
            elif (ki>0) and (kf>0):
                backward = False
            else:
                print("ki and kf must have same sign!")
                backward = False
        else:
            print("Default parameters used")
            T = 8.0
            ki = 10.0
            kf = 10.0
            backward = False      
        print("T:" + str(T) + " ki:" +str(ki) + " kf:" + str(kf))
        planner(T, ki, kf, backward)
    except rospy.ROSInterruptException:
        pass
