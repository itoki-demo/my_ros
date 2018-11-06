#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import smtplib
from email.mime.text import MIMEText
from email.utils import formatdate
from std_msgs.msg import String
from conferio_msgs.msg import Conferio

NODE_NAME = 'send_mail'
FROM_ADDRESS = 'tsukishimaturtlebot@gmail.com'
MY_PASSWORD = 'tsukishima'
SUBJECT = 'A visiter came.'

"""
ROSプログラム
ノード名 send_mail

Conferioの予約情報を購読し、担当者メールアドレスに
会議室の場所と開始時間とお客様が来たことを送信する。
送信元アドレスはtsukishimaturtlebot@gmail.com

subscribe msg
client/conf_info
Conferioの予約情報を購読
"""

"""
メッセージを作成する関数
from_addr 送信元アドレス
to_addr　送信先アドレス
subject 件名
body　本文

"""
def create_message(from_addr, to_addr,　subject, body):
    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = from_addr
    msg['To'] = to_addr
    msg['Date'] = formatdate()
    return msg


"""
メッセージを送信する関数
from_addr 送信元アドレス
passwd　送信元アドレスのアカウントのパスワード
to_addrs　送信先アドレス
msg　create_messageで作成したメッセージ
"""
def send(from_addr, passwd, to_addrs, msg):
    smtpobj = smtplib.SMTP('smtp.gmail.com', 587)
    smtpobj.ehlo()
    smtpobj.starttls()
    smtpobj.ehlo()
    smtpobj.login(from_addr, passwd)
    smtpobj.sendmail(from_addr, to_addrs, msg.as_string())
    smtpobj.close()

"""
/client/conf_infoを購読した時のコールバック関数
受け取った予約情報の担当者アドレスを指定し、会議の場所時間をメール本文に入れてメッセージを作成送信する
"""
def callback(cb_msg):
    #予約情報の担当者アドレスを指定
    to_addr = cb_msg.b_Address
    #会議の場所と時間をメール本文に入れる
    body = "Conference will start in "+cb_msg.b_RoomName+" at "+cb_msg.b_StartTime+"."
    #create_message関数でメッセージを作成
    msg = create_message(FROM_ADDRESS, to_addr, SUBJECT, body)
    #メールを送信
    send(FROM_ADDRESS, MY_PASSWORD, to_addr, msg)

if __name__ == '__main__':
    #ノードの初期化
    rospy.init_node(NODE_NAME)
    sub = rospy.Subscriber('/client/conf_info', Conferio, callback)
    #購読待ちのためのループ
    rospy.spin()

