#!/usr/bin/env python
# -*- coding:utf-8 -*-

import socket
import rospy
from std_msgs.msg import String

NODE_NAME = "smartswitch"

"""
ROSプログラム
ノード名　smartswitch

/operator/turtlebot_statusを購読し、
会議室の扉前に来た時にスマートスイッチで会議室の照明を点灯する
スマートスイッチはソケット通信を用いて所定のメッセージを送信して操作する

改良案
指定の位置に来たら点灯、消灯するようにJsonファイルなどでカスタマイズ可能にする
"""


class SmartSwitch:
    def __init__(self):
        #smartswicthの各ボタンの操作するメッセージ
        self.switch_list = ['#DEVICE,0x01e36d23,141,7,1\r',
                            '#DEVICE,0x01e36d23,141,7,2\r',
                            '#DEVICE,0x01e36d23,141,7,3\r',
                            '#DEVICE,0x01e36d23,141,7,4\r']
        #ノードの初期化
        rospy.init_node(NODE_NAME)
        sub = rospy.Subscriber('/operator/turtlebot_status', String, self.callback)
        #スマートスイッチにアクセスするためのアドレス
        host = "192.168.250.1
        #ポート番号
        port = 23
        #オブジェクトの作成
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.client.connect((host, port))
        self.client.send('nwk\r') #password

    """
    /operator/turtlebot_statusを購読したときのコールバック関数
    turtlebotのステータスがdoorの場合にスイッチの0番を押す
    """
    def callback(self,msg):
        turtlebot_status_list = msg.data.split("_")
        if(turtlebot_status_list[2] == "door"):
            self.client.send(self.switch_list[0])

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    S = SmartSwitch()
    S.run()


