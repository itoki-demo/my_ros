#!/usr/bin/env python
# -*- coding:utf-8 -*-

import socket
import rospy
from std_msgs.msg import String

NODE_NAME = "smartswitch"
switch_list = ['#DEVICE,0x01e36d23,141,7,1\r\n',
               '#DEVICE,0x01e36d23,141,7,2\r\n',
               '#DEVICE,0x01e36d23,141,7,3\r\n',
               '#DEVICE,0x01e36d23,141,7,4\r\n']
def callback(msg):
    turtlebot_status_list = msg.data.split("_")
    if(turtlebot_status_list[2] == "door"):
        client.send(switch_list[0])
        client.send('nwk\R\n')
if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    host = "192.168.250.1" #お使いのサーバーのホスト名を入れます
    port = 23 #適当なPORTを指定してあげます
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #オブジェクトの作成をします
    client.connect((host, port))
    client.recv(4096) #これでサーバーに接続します
    client.send('nwk\r\n') #password
    sub = rospy.Subscriber('/operator/turtlebot_status', String, callback)
    rospy.spin()


