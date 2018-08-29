#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('switchbot_controller')
    pub = rospy.Publisher('switchbot_action',String)
    r = rospy.Rate(1)
    while True:
        r.sleep()
        pub.publish('action')
    