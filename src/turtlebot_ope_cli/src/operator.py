#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Int32
import json
import collections

#目標地点リスト　名前, 座標, 向き jsonファイルで読み込み
decoder = json.JSONDecoder(object_pairs_hook=collections.OrderedDict)
room_waypoints_jsonfile_path = "/home/turtlebot/catkin_ws/src/icclab_turtlebot/maps/modified_lobby_waypoints.json"
with open(room_waypoints_jsonfile_path) as f:
    df = decoder.decode(f.read())
initial_point = df["initial_point"]
room_waypoints = df["room_waypoints"]

#room_waypoints = {
#    "Room01":[["door_key_1", (-1.4, -2.7), (0.0, 0.0, 0.149230403361, 0.988802450802)],
#              ["room", ( 2.9,  0.0), (0.0, 0.0, 0.974797896522, 0.223089804646)],
#              ["door_key_2", (-1.4, -2.7), (0.0, 0.0, 0.149230403361, -0.988802450802)]],
#    "Room02":[["door_free_1", (-1.4, -2.7), (0.0, 0.0, 0.149230403361, 0.988802450802)],
#              ["room", ( 2.9,  0.0), (0.0, 0.0, 0.974797896522, 0.223089804646)],
#              ["door_free_2", (-1.4, -2.7), (0.0, 0.0, 0.149230403361, -0.988802450802)]]
#}
#initialpoint = [(-1.09, 2.48), (0.0, 0.0, -0.739811508606, 0.672814188119)]

#waypoints = [
#    ["Room01", (-1.4, -2.7), (0.0, 0.0, 0.149230403361, 0.988802450802)],
#    ["Room02", (2.9, 0.0), (0.0, 0.0, 0.974797896522, 0.223089804646)],
#    ["Room03", (-1.09, 2.48), (0.0, 0.0, -0.739811508606, 0.672814188119)]
#]
#waypointsから目標地点名のみ抽出
room_names = []
for w in room_waypoints:
    room_names.append(w)

class Waypoint(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success'])

        #move_baseをクライアントとして定義
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        #目標地点を定義
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = position[2]
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        #目標地点を送信し結果待ち
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return 'success'

#次の目標地点名の受信待ち
class Reception(State):
    def __init__(self):
        State.__init__(self,outcomes=room_names)
        self.callback_flag= 0
        self.next_goal = ''
        self.r = rospy.Rate(1)
        self.status = "reception"
        self.pub=rospy.Publisher('turtlebot_status', String, queue_size=10)
    def execute(self,userdata):
        self.pub.publish(self.status)
        sub = rospy.Subscriber('next_goal',String, self.callback)
        while(self.callback_flag == 0):
            self.r.sleep()
        self.callback_flag =0
        return self.next_goal
        
    def callback(self,msg):
        if (msg.data in room_names and self.next_goal != msg.data):
            self.next_goal = msg.data
            self.callback_flag = 1

#start_flag待ち
class WaitStartFlag(State):
    def __init__(self,status):
        State.__init__(self,outcomes=['success'])
        self.status = status
        self.callback_flag= 0
        self.r = rospy.Rate(1)
        self.pub=rospy.Publisher('turtlebot_status', String, queue_size=10)
    def execute(self,userdata):
        self.pub.publish(self.status)
        sub = rospy.Subscriber('start_flag',String, self.callback)
        while(self.callback_flag == 0):
            self.r.sleep()
        self.callback_flag =0
        return 'success'
    def callback(self,msg):
        self.callback_flag = 1

#部屋まで移動
class MoveToRoom(State):
    def __init__(self):
        State.__init__(self,outcomes=['success'])
    def execute(self,userdata):
        return 'success'

#AreaScanをして扉が開いているかどうか判断する
class AreaScan(State):
    def __init__(self, room):
        State.__init__(self,outcomes=['success'])
	self.pub = rospy.Publisher('call_area_scan', String, queue_size = 10)
        self.callback_flag = 0
        self.r = rospy.Rate(10)
	self.room = room
    def execute(self,userdata):
	self.pub.publish(self.room)
        sub = rospy.Subscriber('area_scan', String, self.callback)
        while(self.callback_flag):
            self.r.sleep()
        self.callback_flag = 0
        return 'success'
    def callback(self,msg):
        if(msg.data == "True"):
            self.callback_flag = 1

def main():
    rospy.init_node('operator')
    operator = StateMachine(['success','reception','move_to_reception'] + room_names)
    reception_transitions={}
    for r in room_names:
        reception_transitions[r] = r
    with operator:
        #受けつけ、受付まで移動状態を追加
        StateMachine.add('move_to_reception',
                         Waypoint(initial_point["position"],
                                  initial_point["orientation"]),
                         transitions={'success':'reception'})
        StateMachine.add('reception',Reception(),
                         transitions=reception_transitions)

        for r in room_names:
            waypoints = room_waypoints[r]
            next_move_state_names = []# [Navigate_Room01_door_key_1, Room01_room]
            next_wait_state_names = []# [Navigate_Room01_door_key_1_wait, Room01_room_wait]
            for w_n in waypoints:
                    next_move_state_names.append('Navigate_'+r+'_'+w_n)#Navigate_
                    next_wait_state_names.append('Navigate_'+r+'_'+w_n+'_wait')#Navigate_
                    rospy.loginfo('Navigate_'+r+'_'+w_n)#Navigate_
            operator.register_outcomes(next_move_state_names+next_wait_state_names)
            next_move_state_names.append('move_to_reception')

            StateMachine.add(r,MoveToRoom(),transitions={'success':next_move_state_names[0]})
            for i, (w_n,w) in enumerate(waypoints.items()):
                w_n_split = w_n.split("_")
                if(w_n_split[0] == "room"):
                    StateMachine.add(next_move_state_names[i],
                                     Waypoint(w["position"],
                                              w["orientation"]),
                                     transitions={'success':next_wait_state_names[i]})
                    StateMachine.add(next_wait_state_names[i],
                                     WaitStartFlag(next_move_state_names[i]),
                                     transitions={'success':next_move_state_names[i+1]})
                elif(w_n_split[0] == "door"):
                    if(w_n_split[1] == "areascan"):
                        areascan_state_scan_name = next_move_state_names[i] + "_scan"
                        areascan_state_move_name = next_move_state_names[i] + "_move"
                        operator.register_outcomes([areascan_state_scan_name,
                                                    areascan_state_move_name])
                        StateMachine.add(next_move_state_names[i],
                                         Waypoint(w[0]["position"],
                                                  w[0]["orientation"]),
                                         transitions={'success':next_wait_state_names[i]})
                        StateMachine.add(next_wait_state_names[i],
                                         WaitStartFlag(next_move_state_names[i]),
                                         transitions={'success':areascan_state_move_name})
                        StateMachine.add(areascan_state_move_name,
                                         Waypoint(w[1]["position"],
                                                  w[1]["orientation"]),
                                         transitions={'success':areascan_state_scan_name})
                        StateMachine.add(areascan_state_scan_name,
                                         AreaScan(r),
                                         transitions={'success':next_move_state_names[i+1]})
                    else:
                        StateMachine.add(next_move_state_names[i],
                                         Waypoint(w[0]["position"],
                                                  w[0]["orientation"]),
                                         transitions={'success':next_wait_state_names[i]})
                        StateMachine.add(next_wait_state_names[i],
                                         WaitStartFlag(next_move_state_names[i]),
                                         transitions={'success':next_move_state_names[i+1]})
    operator.execute()
    return

if __name__ == '__main__':
    main()

