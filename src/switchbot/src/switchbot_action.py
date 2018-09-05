#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def switchbot_action():
    rospy.init_node('switchbot_controller')
    pub = rospy.Publisher('switchbot_action',String, queue_size = 10)
    r = rospy.Rate(1)
    while True:
        pub.publish('action')
        r.sleep()

if __name__ == '__main__':

    try:
        switchbot_action()
    except rospy.ROSInterruptException:
        pass
 
