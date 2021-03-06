import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
 
wheel_radius = 0.03
robot_radius = 0.16 

pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 1)

def callback(data):
    (v, a) = forward_kinematics(data.data, 0)

    t = Twist()
    t.linear.x = v
    t.angular.z = a
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
    rospy.Subscriber("/wheel_vel_left", Float32, callback)

    rospy.spin()

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

if __name__ == '__main__':
    rospy.init_node('workshop4', anonymous=True)
    listener()
