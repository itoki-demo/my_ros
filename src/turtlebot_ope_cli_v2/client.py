#!/usr/bin/env python
# -*- coding:utf-8 -*-
import requests
import rospy
import json
from std_msgs.msg import String
from conferio_msgs.msg import Conferio
from rospy_message_converter import json_message_converter

NODE_NAME = "client"
"""
Conferio msg
string b_Num
string b_ID
string b_Name
string b_RoomName
string b_Date
string b_StartTime
string b_FinishTime
string b_Time
string b_Address

ROSプログラム
iPadはrosmsgを直接受け取ることができないため、
iPadとその他デバイスをrosmsgを用いて通信するための中継プログラム
iPadからの入力情報とその他デバイスから受け取ったrosmsgを双方に変換する
raspberry piに構成したphpサーバに双方のデータを格納し通信する

publish msg
client/conf_info
Conferio型のmsg
iPadから入力された予約番号をconferioサーバに照合しphpサーバに格納された予約情報を配信する
client/start_flag
String型のmsg
iPadからの入力待ち状態時にiPadから入力があった時に配信する


subscribe msg
operator/turtlebot_status
String型のmsg
Turtlebotの到着した場所を購読する

phpサーバとデータを送受信するurl
url_conf_info
予約情報を受け取るurl
予約情報がない状態（予約番号入力待ち状態）の時はb_NumがNone
url_update_status
Turtlebotが到着した場所を送るurl
url_get_status
iPadからの入力待ち状態時にiPadからの入力を受け取るurl

"""
class Client:
    def __init__(self):
        self.callback_flag = 0
        #urlを初期化
        host = '192.168.20.42' #サーバのIPアドレス
        server_conf_info = 'get_conf_info.php'
        self.url_conf_info = 'http://{}/{}'.format(host,server_conf_info)#予約情報受け取りURL

        server_update_status   = 'update_status.php'
        self.status_update_status = 'reception'
        self.url_update_status = 'http://{}/{}?status='.format(host,server_update_status)#Turtlebot到達情報送信URL

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

    def run(self):
        #rosノードの初期化とpublisher,subscriberの初期化
        rospy.init_node(NODE_NAME)
        pub_conf_info=rospy.Publisher(NODE_NAME + '/conf_info', Conferio, queue_size = 10)
        pub_start=rospy.Publisher(NODE_NAME + '/start_flag', String, queue_size = 10)
        sub = rospy.Subscriber('operator/turtlebot_status', String, self.callback)#'/move_base/status',GoalStatusArray, callback)

        #phpサーバにTurtlebotの初期状態(Reception)をセットする
        requests.post(self.url_update_status+self.status_update_status)
        #sleepする時間を指定(Hz)
        sleep_rate = 2
        r = rospy.Rate(sleep_rate)

        while not rospy.is_shutdown():
            #Conferioの予約情報をサーバから受け取る
            while not rospy.is_shutdown():
                conf_info = requests.post(self.url_conf_info).text.replace('\n','').replace('\r','')
                #b_NumがNoneの場合は予約情報なし
                if (json.loads(conf_info)['b_Num']!=None):
                    break
                r.sleep()
                rospy.loginfo("no input")

            #予約番号入力直後にTurtlebotが動き出さないようにするための遅延させるプロセス
            #iPad側の「動きます」画面から「案内中」画面に遷移する時間に連動
            while not rospy.is_shutdown():
                response = requests.post(self.url_get_status).text.replace('\n','').replace('\r','')
                if (response=='navigation'):
                    break
                r.sleep()

            #Conferioの予約情報を配信
            rospy.loginfo("publish conf_info")
            conf_info_msg = json_message_converter.convert_json_to_ros_message('conferio_msgs/Conferio', conf_info)
            pub_conf_info.publish(conf_info_msg)

            #Turtlebotの到着待ち
            while not rospy.is_shutdown():
                while((self.callback_flag == 0) and not rospy.is_shutdown()):
                    rospy.loginfo("navigation")
                    r.sleep()
                self.callback_flag = 0
                #到着した場所情報をサーバへ送信
                requests.post(self.url_update_status+self.status_update_status)
                rospy.loginfo("arrived {}".format(self.status_update_status))

                #到着した場所がreceptionの場合は予約番号入力待ちへ戻る
                if(self.status_update_status == 'reception'):
                    break

                #iPadからの入力待ち
                while not rospy.is_shutdown():
                    response = requests.post(self.url_get_status).text.replace('\n','').replace('\r','')
                    if (response=='navigation'):
                        break
                    r.sleep()
                    rospy.loginfo("waiting for input from iPad")
                pub_start.publish('true')
        return

if __name__ == '__main__':
    a = Client()
    a.run()


