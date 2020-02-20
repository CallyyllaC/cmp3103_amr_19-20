#https://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
 
wheel_radius = 0.06
robot_radius = 0.2

cmd_v = [0.1, 0.6]
pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

def callback(data):
    print "callback start"
    (v, a) = forward_kinematics(data.data, 0)

    t = Twist()
    t.linear.x = v * cmd_v[0]
    t.angular.y = a * cmd_v[1]
    pub.publish(t)

    #get the input we need
    (w_l, w_r) = inverse_kinematics(0.0, 1.0)
    print "w_l = %f,\tw_r = %f" % (w_l, w_r)

    #what we get from this input
    (v, a) = forward_kinematics(w_l, w_r)
    print "v = %f,\ta = %f" % (v, a)

    (w_l, w_r) = inverse_kinematics_from_twist(t)
    print "w_l = %f,\tw_r = %f" % (w_l, w_r)

def listener():
    print "listener start"
    rospy.Subscriber("/wheel_vel_left", Float32, callback)

    print "listener spinning"
    rospy.spin()
    print "listener end"

 # computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / robot_radius#(2 * robot_radius)
    return (v, a)
 
# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v + (robot_radius * a) /2
    c_r = v - (robot_radius * a) /2
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)

# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

if _name_ == '_main_':
    rospy.init_node('workshop4', anonymous=True)
    listener()