#!/usr/bin/env python
# -*- coding:utf-8 -*-
import requests
import rospy
from std_msgs.msg import String

class main:
    def __init__(self):
        self.callback_flag = 0
        host = '192.168.20.42' #サーバのIPアドレス
        server_roomname = 'room_name.php'
        status_roomname=''
        self.url_roomname = 'http://{}/{}'.format(host,server_roomname)#目標地点名受け取りURL

        server_update_status   = 'update_status.php'
        self.status_update_status = 'arrived'
        self.url_update_status = 'http://{}/{}?status='.format(host,server_update_status)#到達情報送信URL

        server_get_status   = 'get_status.php'
        self.url_get_status = 'http://{}/{}'.format(host,server_get_status)#iPadStatus情報信受信RL

    def callback(self,msg):
        #msg.status_listの最新ステータスはリスト末尾にある
        #latest_status : 0 待ち
        #latest_status : 3 ゴール
        #latest_status : １ 移動中
        #latest_status = msg.status_list[len(msg.status_list)-1].status
        #if((latest_status==0) or (latest_status==3)):
        #    callback_flag = 1
        #if(latest_status==1):
        #    callback_flag = 0
        self.status_update_status = msg.data
        self.callback_flag=1

    def Main(self):
        rospy.init_node('client')
        pub_goal=rospy.Publisher('next_goal', String)
        pub_start=rospy.Publisher('start_flag', String)
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            #目標地点名をサーバから受け取る
            while not rospy.is_shutdown():
                str_in = requests.post(self.url_roomname).text.replace('\n','').replace('\r','')
                if (str_in!='no goal'):
                    rospy.loginfo("next_goal is {}".format(str_in))
                    break
                r.sleep()
                rospy.loginfo("no goal")
            pub_goal.publish(str_in)
            #到達待ち
            while not rospy.is_shutdown():
                sub = rospy.Subscriber('turtlebot_status', String, self.callback)#'/move_base/status',GoalStatusArray, callback)

                while(self.callback_flag == 0):
                    rospy.loginfo("navigation")
                    r.sleep()
                self.callback_flag = 0
                #到達情報をサーバへ送信
                requests.post(self.url_update_status+self.status_update_status)
                rospy.loginfo("arrived {}".format(self.status_update_status))
                if(self.status_update_status == 'reception'):
                    break

                #iPadからの入力待ち
                while not rospy.is_shutdown():
                    str_in = requests.post(self.url_get_status).text.replace('\n','').replace('\r','')
                    if (str_in=='navigation'):
                        break
                    r.sleep()
                    rospy.loginfo("waiting for input from iPad")
                pub_start.publish('true')
        return

if __name__ == '__main__':
    a = main()
    a.Main()


