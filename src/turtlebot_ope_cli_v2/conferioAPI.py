#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import requests
from std_msgs.msg import String
from conferio_msgs.msg import Conferio
from rospy_message_converter import message_converter

NODE_NAME = "conferioAPI"

class conferioAPI:
    def __init__(self):
        rospy.init_node(NODE_NAME)
        host = '192.168.20.59' #サーバのIPアドレス
        php_conf_info = 'conferioAPI.php'
        url_conf_info = 'http://{}/{}?b_Num='.format(host,php_conf_info)
        self.pub_conf_info=rospy.Publisher(NODE_NAME+ "/conf_info", Conferio, queue_size = 10)
        self.r = rospy.Rate(10)
        sub = rospy.Subscriber('/get_conf/reservation_num', String, self.callback)
        self.reservation_num = ""
        self.callback_flag = 0
    def run(self):
        while not rospy.is_shutdown():
            while(self.callback_flag == 0):
                self.r.sleep()
            self.callback_flag = 0
            conf_info = str(requests.post(url_conf_info+self.reservation_num).text)
            conf_info_msg = json_message_converter.convert_json_to_ros_message('std_msgs/String', conf_info)
            pub_conf_info.publish(conf_info_msg)
            rospy.loginfo('publish conf')
    def callback(self, msg):
        self.reservation_num = msg.data
        self.callback_flag = 1

if __name__ == '__main__':
    a = conferioAPI()
    a.run()
