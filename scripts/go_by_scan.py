#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    r = data.ranges[len(data.ranges)/2]
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel = Twist()
    
    if r >= 2.0:
        rospy.loginfo("OK")
        vel.linear.x = 0.2
    else:
        rospy.loginfo("NG")
        vel.linear.x = 0

    pub.publish(vel)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('go_by_scan', anonymous=True)

    rospy.Subscriber("base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()