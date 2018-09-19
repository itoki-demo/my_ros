#!/usr/bin/env python
# -*- coding:utf-8 -*-

import socket

host = "192.168.250.1" #お使いのサーバーのホスト名を入れます
port = 23 #適当なPORTを指定してあげます

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #オブジェクトの作成をします

client.connect((host, port))
print(client.recv(4096)) #これでサーバーに接続します

client.send('nwk\n') #適当なデータを送信します（届く側にわかるように）
switch_list = ['#DEVICE,0x01e36d23,141,7,1\r\n','#DEVICE,0x01e36d23,141,7,2\r\n','#DEVICE,0x01e36d23,141,7,3\r\n','#DEVICE,0x01e36d23,141,7,4\r\n']
while True:
    print("botton number :")
    str_in = raw_input()
    num = int(str_in)
    client.send(switch_list[num-1])
    print(switch_list[num-1])
    client.send('nwk\n')
