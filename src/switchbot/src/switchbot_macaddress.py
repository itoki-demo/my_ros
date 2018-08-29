#!/usr/bin/env python
import binascii
from bluepy.btle import Peripheral
import rospy
from std_msgs.msg import String

def callback(msg):
    p = Peripheral("C4:D4:53:AB:ED:5B", "random")
    hand_service = p.getServiceByUUID("cba20d00-224d-11e6-9fb8-0002a5d5c51b")
    hand = hand_service.getCharacteristics("cba20002-224d-11e6-9fb8-0002a5d5c51b")[0]
    hand.write(binascii.a2b_hex("570100"))
    p.disconnect()

if __name__ == '__main__':
    rospy.init_node('switchbot')
    sub = rospy.Subscriber('switchbot_action',String,callback)
    rospy.spin()
    