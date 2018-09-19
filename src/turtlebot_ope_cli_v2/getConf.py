#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import requests
from std_msgs.msg import String

NODE_NAME = "get_conf"


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    host = '192.168.20.59' #サーバのIPアドレス
    php_get_reserv_num = 'get_reservation_num.php'
    url_get_reserv_num = 'http://{}/{}'.format(host,php_get_reserv_num)
    pub_reservation_num=rospy.Publisher(NODE_NAME+ "/reservation_num", String, queue_size = 10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            reservation_num = requests.post(url_get_reserv_num).text.replace('\n','').replace('\r','')
            if (reservation_num!='no input'):
                rospy.loginfo("reservation_num:{}".format(reservation_num))
                break
            r.sleep()
            rospy.loginfo("no input")
        pub_reservation_num.publish(reservation_num)
