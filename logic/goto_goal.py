#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan # for the laser scanner
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry # to read the status
from tf import transformations
import math

current_position = Point()
katefthinsi = 0
robot_state = 0 # 0 if needs to fix katefthinsi, 1 if needs to go straight, 2 if reached goal

# goal position
goal_position = Point()
goal_position.x = 8
goal_position.y = 8
goal_position.z = 0
# parameters
katefthinsi_precision = math.pi / 90
dist_precision = 0.5

# publishers
pub = None

<<<<<<< HEAD
=======

>>>>>>> 025817999e1e9c468a4f5808d587c0b60ed83918
def fix_katefthinsi(goal_pos):
    global katefthinsi, pub, katefthinsi_precision, robot_state
    desired_katefthinsi = math.atan2(goal_pos.y - current_position.y, goal_pos.x - current_position.x)
    err_katefthinsi = desired_katefthinsi - katefthinsi
    
    twist_msg = Twist()
    if math.fabs(err_katefthinsi) > katefthinsi_precision:
        twist_msg.angular.z = -0.3 if err_katefthinsi > 0 else 0.3
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_katefthinsi) <= katefthinsi_precision:
        print ('katefthinsi error: [%s]' % err_katefthinsi)
        change_state(1)

def go_straight_ahead(goal_pos):
    global katefthinsi, pub, katefthinsi_precision, robot_state
    desired_katefthinsi = math.atan2(goal_pos.y - current_position.y, goal_pos.x - current_position.x)
    err_katefthinsi = desired_katefthinsi - katefthinsi
    err_pos = math.sqrt(pow(goal_pos.y - current_position.y, 2) + pow(goal_pos.x - current_position.x, 2))
    
    if err_pos > dist_precision:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_katefthinsi) > katefthinsi_precision:
        print ('katefthinsi error: [%s]' % err_katefthinsi)
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


# The change_state function, that provides us a log message and changes the global state var
def change_state(state):
    global robot_state
    robot_state = state
    print('State changed to [%s]' % robot_state)

# The callback for the odometry reading
def clbk_odom(msg):
    global current_position
    global katefthinsi
    
    # position
    current_position = msg.pose.pose.position
    
    # katefthinsi
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    katefthinsi = euler[2]


rospy.init_node('goto_goal')

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

rate = rospy.Rate(20)

while not rospy.is_shutdown():
    
    if robot_state == 0:
        print("kate")
        fix_katefthinsi(goal_position)
    elif robot_state == 1:
        print("ahead")
        go_straight_ahead(goal_position)
    elif robot_state == 2:
        print("done")
        done()
        pass
    rate.sleep()
