#!/usr/bin/env python
# -*- coding:utf-8 -*-
import requests
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
import time

callback_flag = 0
def callback(msg):
    global callback_flag
    #msg.status_listの最新ステータスはリスト末尾にある
    #latest_status : 0 待ち
    #latest_status : 3 ゴール
    #latest_status : １ 移動中
    #latest_status = msg.status_list[len(msg.status_list)-1].status
    #if((latest_status==0) or (latest_status==3)):
    #    callback_flag = 1
    #if(latest_status==1):
    #    callback_flag = 0
    callback_flag=1

if __name__ == '__main__':
    global callback_flag
    next_goal = ''
    host = '192.168.10.225' #サーバのIPアドレス
    server_roomname = 'roomname.php'
    status_roomname=''
    url_roomname = 'http://{}/{}'.format(host,server_roomname)#目標地点名受け取りURL

    server_arrive   = 'arrive.php'
    status_arrive = 'arrived'
    url_arrive = 'http://{}/{}?status={}'.format(host,server_arrive,status_arrive)#到達情報送信URL
    requests.post(url_arrive)#目標地点初期化　場所なし

    rospy.init_node('client_controller')
    pub=rospy.Publisher('controller', String)
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        #目標地点名をサーバから受け取る
        while not rospy.is_shutdown():
            str_in = requests.post(url_roomname).text
            if (str_in!='no goal'):
                print(str_in)
                break
            r.sleep()
        pub.publish(str_in)
        #到達待ち
        sub = rospy.Subscriber('arrive_flag', String, callback)#'/move_base/status',GoalStatusArray, callback)

        while(callback_flag == 0):
            r.sleep()
        callback_flag = 0
        #到達情報をサーバへ送信
        requests.post(url_arrive)
        print("arrived")
         

