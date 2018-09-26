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
TO_ADDRESS = 'mizutani83ha@gmail.com'
BCC = ''
SUBJECT = 'A visiter came.'
BODY = 'pythonでメール送信'


def create_message(from_addr, to_addr, bcc_addrs, subject, body):
    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = from_addr
    msg['To'] = to_addr
    msg['Bcc'] = bcc_addrs
    msg['Date'] = formatdate()
    return msg


def send(from_addr, passwd, to_addrs, msg):
    smtpobj = smtplib.SMTP('smtp.gmail.com', 587)
    smtpobj.ehlo()
    smtpobj.starttls()
    smtpobj.ehlo()
    smtpobj.login(from_addr, passwd)
    smtpobj.sendmail(from_addr, to_addrs, msg.as_string())
    smtpobj.close()

def callback(cb_msg):
    to_addr = cb_msg.b_Address
    body = "Conference will start in "+cb_msg.b_RoomName+" at "+cb_msg.b_StartTime+"."
    msg = create_message(FROM_ADDRESS, to_addr, BCC, SUBJECT, body)
    send(FROM_ADDRESS, MY_PASSWORD, to_addr, msg)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    sub = rospy.Subscriber('/client/conf_info', Conferio, callback)
    rospy.spin()

