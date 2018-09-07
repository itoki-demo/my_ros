#!/usr/bin/env python
#coding:utf-8

import rospy
import tf
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32, String
import math
import numpy as np


#scan_area = [[min_x, min_y], [max_x, max_y]]
DEFAULT_AREA = np.array([[-6.0, 0.0],
                         [-5.0, 1.0]])
SCAN_AREA_0 = np.array([[-1.0, 1.0],
                        [0.3, 3.0]])
SCAN_AREA_1 = np.array([[0.5, -0.5],
                        [1.5, 0.5]])
# keyはステータスを表すトピックの内容そのまま
KEY_LIST = ["default", "Room01"]
AREA_DICT = {KEY_LIST[0]:DEFAULT_AREA, KEY_LIST[1]:SCAN_AREA_0}
FUNC_DICT = {KEY_LIST[0]:"wait", KEY_LIST[1]:"check_obs"}


# マーカートピック配信用クラス
class Marker2D(object):
    def __init__(self, topic_name, r, g, b, marker_type=Marker.CUBE):
        self.marker_array = MarkerArray()
        self.pub = rospy.Publisher(topic_name, MarkerArray, queue_size = 1)
        self.r = r
        self.g = g
        self.b = b
        self.scale = np.array([0.05, 0.05, 0.05])
        self.pos = np.array([0.0, 0.0, 0.0])
        self.ori = np.array([0.0, 0.0, 0.0, 0.0])
        """
        uint8 ARROW=0
        uint8 CUBE=1
        uint8 SPHERE=2
        uint8 CYLINDER=3
        uint8 LINE_STRIP=4
        uint8 LINE_LIST=5
        uint8 CUBE_LIST=6
        uint8 SPHERE_LIST=7
        uint8 POINTS=8
        """
        self.set_marker = self._set_cube
        if marker_type != Marker.CUBE:
            if marker_type == Marker.LINE_STRIP:
                self.set_marker = self._set_line_strip
                self.scale /= 5
            if marker_type == Marker.ARROW:
                self.set_marker = self._set_arrow

    def set_pos(self, x, y, z):
        self.pos[0] = x
        self.pos[1] = y
        self.pos[2] = z

    def set_ori(self, x, y, z, w):
        self.ori[0] = x
        self.ori[1] = y
        self.ori[2] = z
        self.ori[3] = w

    def set_scale(self, x, y, z):
        self.scale[0] = x
        self.scale[1] = y
        self.scale[2] = z

    # 配信実行関数　xy座標のリストを引数とする
    def publish_marker(self, x_list, y_list):
        self.marker_array = MarkerArray()
        self.set_marker(x_list, y_list, len(x_list))
        self.pub.publish(self.marker_array)
    
    # CUBE用のパラメータ設定
    def _set_cube(self, x_list, y_list, length):
        for i in range(length):
            marker = Marker()
            self.__set_marker_param(marker)
            marker.type = marker.CUBE
            marker.id = i
            marker.pose.position.x = x_list[i]
            marker.pose.position.y = y_list[i]
            marker.pose.position.z = 0.2
            self.marker_array.markers.append(marker)

    # LINE_STRIP用のパラメータ設定
    def _set_line_strip(self, x_list, y_list, length):
        marker = Marker()
        self.__set_marker_param(marker)
        marker.type = marker.LINE_STRIP
        marker.id = 1
        marker.points = []
        for i in range(length):
            temp_point = Point()
            temp_point.x = x_list[i]
            temp_point.y = y_list[i]
            temp_point.z = 0.2
            marker.points.append(temp_point)
        self.marker_array.markers.append(marker)
    
    # ARROW用 
    def _set_arrow(self, x_list, y_list, length):
        for i in range(length):
            marker = Marker()
            self.__set_marker_param(marker)
            marker.type = marker.ARROW
            marker.id = i
            marker.pose.position.x = x_list[i]
            marker.pose.position.y = y_list[i]
            marker.pose.position.z = 0.2
            self.marker_array.markers.append(marker)

    # 共通設定
    def __set_marker_param(self, marker):
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.action = marker.ADD
        marker.pose.position.x = self.pos[0]
        marker.pose.position.y = self.pos[1]
        marker.pose.position.z = self.pos[2]
        marker.pose.orientation.x=self.ori[0]
        marker.pose.orientation.y=self.ori[1]
        marker.pose.orientation.z=self.ori[2]
        marker.pose.orientation.w=self.ori[3]
        marker.color.r = self.r
        marker.color.g = self.g
        marker.color.b = self.b
        marker.color.a = 1.0
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.lifetime = rospy.Duration(0.5)


class Function(object):
    def __init__(self, node_name, area, sc):
        self.node_name = node_name
        self.area = area
        self.pub = rospy.Publisher(node_name + "/area_scan", String, queue_size = 1)
        self.CM0 = Marker2D("AreaScanMarker/obstacle_marker", 1.0, 0.0, 0.0)
        self.CM1 = Marker2D("AreaScanMarker/area_marker", 0.0, 0.0, 1.0, 4)
        self.CM2 = Marker2D("AreaScanMarker/vec_marker", 0.0, 1.0, 0.0, 0)
        self.pub_msg = ["False", "True"]
        self.sc = sc

    @staticmethod
    def _vec_to_euler(x, y, scular):
        s = math.acos(x/scular)
        if y < 0:
            s = 2*np.pi - s
        return s
    
    @staticmethod
    def _quaternion_to_euler(x, y, z, w):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        """
        e = tf.transformations.euler_from_quaternion((x, y, z, w))
        return e[2]

    @staticmethod
    def euler_to_quaternion(x, y, z):
        """Convert Euler Angles to Quaternion
        """
        q = tf.transformations.quaternion_from_euler(x, y, z)
        return q


class WaitFunc(Function):
    def __init__(self, node_name, area, sc):
        super(WaitFunc, self).__init__(node_name, area, sc)

    def run(self, scan_msg):
        pass


class CheckObsFunc(Function):
    def __init__(self, node_name, area, sc):
        super(CheckObsFunc, self).__init__(node_name, area, sc)
        # pos = [position.x, position.y, angle]
        self.pos = np.array([0, 0, 0], dtype='float')
        self.angle_list = np.zeros(0)
        self.x_list = np.zeros(0)
        self.y_list = np.zeros(0)
        self.listener = tf.TransformListener()
        self.trans = 0
        self.rot = 0
        self.check = 0

    def run(self, scan_msg):
        execute = False
        if len(self.angle_list) == 0:
            self._init_list(scan_msg)
        try:
            (self.trans, self.rot) = self.listener.lookupTransform('/map', '/rplidar_frame', rospy.Time(0))
            execute = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        if execute:
            self.pos[0] = self.trans[0]
            self.pos[1] = self.trans[1]
            self.pos[2] = self._quaternion_to_euler(self.rot[0], self.rot[1], self.rot[2], self.rot[3]) + np.pi

            # センサー情報を極座標からxy(z)座標へ変換＋map座標とturtlebot位置の差分を加える　
            self.x_list = np.array(scan_msg.ranges) * np.cos((self.pos[2]-self.angle_list)) + self.pos[0]
            self.y_list = np.array(scan_msg.ranges) * np.sin((self.pos[2]-self.angle_list)) + self.pos[1]

            # 指定エリア内にあるスキャン点をTrueとする　
            x_flg_array = (self.x_list > self.area[0, 0]) * (self.x_list < self.area[1, 0])
            y_flg_array = (self.y_list > self.area[0, 1]) * (self.y_list < self.area[1, 1])
            area_flg_array = x_flg_array * y_flg_array

            # 指定エリア内にあるスキャン点に対しいろいろ処理　
            obs_num = np.sum(area_flg_array)
            check_msg = self._time_check(obs_num)
            rospy.loginfo(str(obs_num) + ", " + check_msg)
            indices = np.where(area_flg_array == 1)
            self.CM0.publish_marker(self.x_list[indices], self.y_list[indices])
            self.CM1.publish_marker([self.area[0, 0],
                                     self.area[1, 0], 
                                     self.area[1, 0],
                                     self.area[0, 0],
                                     self.area[0, 0]],
                                    [self.area[0, 1],
                                     self.area[0, 1],
                                     self.area[1, 1],
                                     self.area[1, 1],
                                     self.area[0, 1]])
            if check_msg == "True":
                self.pub.publish(check_msg)
                self.sc.stop_check = True

    # 障害物がないと判定する条件
    def _time_check(self, obs_num):
        r = self.pub_msg[0]
        if obs_num == 0:
            self.check += 1
        else:
            self.check = 0
        if self.check > 10:
            r = self.pub_msg[1]
        return r

    # /scanトピック情報を利用してリストを初期化
    def _init_list(self, msg):
        temp_angle = np.arange(msg.angle_min,
                               msg.angle_min + np.pi * 2,
                               msg.angle_increment, dtype='float') 
        self.angle_list = np.zeros(len(msg.ranges), dtype='float') +\
                          temp_angle[:len(msg.ranges)] - np.pi
        self.x_list = np.zeros(len(msg.ranges), dtype='float')
        self.y_list = np.zeros(len(msg.ranges), dtype='float')
        



# 特定領域のみの障害物検出クラス（非ナビゲーション状態で動く障害物を認識、行動判断することを目的としたクラス）
class ScanFunction(object):
    def __init__(self, key_list, area_dict, node_name):
        
        

        self.latest_map = np.zeros(0)
        self.latest_vec = np.zeros(0)
        self.units_len = 20
       

    def check_movement(self):
        pass

    # トピック内容からスキャン領域+タスクを更新
    def _update_task(self, new_status):
        if self.status != new_status:
            self.status = new_status
            rospy.loginfo("new status: " + self.status)
            if self.status in self.area_dict.keys():
                rospy.loginfo("update area")
                self.s_area = self.area_dict[self.status]
                if self.s_area[0, 0] > self.s_area[1, 0] or self.s_area[0, 1] > self.s_area[1, 1]:
                    print "error:スキャンエリア座標指定が間違っています"
                    rospy.loginfo("error:スキャンエリア座標指定が間違っています")
                    raise Exception
                # スキャン領域が更新された際、動体認識やりなおし
                x = int(math.ceil((self.s_area[1, 0] - self.s_area[0, 0]) * self.units_len))
                y = int(math.ceil((self.s_area[1, 1] - self.s_area[0, 1]) * self.units_len))
                self.latest_map = np.zeros((x, y), dtype='int')
                self.latest_vec = np.zeros((x*2-1, y*2-1), dtype='int')
            else:
                rospy.loginfo("keep current area")


    # ステータス更新コールバック
    def _status_cb(self, msg):
        self._update_area(msg.data)

    # /scanコールバック
    def _callback(self, scan_msg): 
        self.__scan_movement(self.x_list[indices], self.y_list[indices])
    
    def __scan_movement_task(self, x_list, y_list):
        # 2次元配列格子の数を設定
        x = int(math.ceil((self.s_area[1, 0] - self.s_area[0, 0]) * self.units_len))
        y = int(math.ceil((self.s_area[1, 1] - self.s_area[0, 1]) * self.units_len))
        obs_map = np.zeros((x, y), dtype='int')
        ind_list = np.zeros((2, 0), dtype='int')
        for n in range(len(x_list)):
            t_x = int(math.floor((x_list[n] - self.s_area[0, 0])*self.units_len))
            t_y = int(math.floor((y_list[n] - self.s_area[0, 1])*self.units_len))
            if obs_map[t_x, t_y] == 0:
                ind_list = np.hstack((ind_list, [[t_x], [t_y]]))
                obs_map[ind_list[0, len(ind_list[0])-1], ind_list[1, len(ind_list[0])-1]] = 1
        vec_map = np.zeros((x*2-1, y*2-1), dtype='int')
        for i in range(len(ind_list[0])):
            xi = (x-1)-ind_list[0, i]
            yi = (y-1)-ind_list[1, i]
            vec_map[xi:xi+x, yi:yi+y] -= self.latest_map[:, :]
        x_table = (np.zeros((2*x-1, 2*y-1)).T + np.arange(2*x-1) - (x - 1)).T
        y_table = np.zeros((2*x-1, 2*y-1)) + np.arange(2*y-1) - (y - 1)
        x_vec = np.sum((vec_map+self.latest_vec) * x_table)/self.units_len
        y_vec = np.sum((vec_map+self.latest_vec) * y_table)/self.units_len
        scular = math.sqrt(x_vec*x_vec + y_vec*y_vec)
        q = self.euler_to_quaternion(0.0, 0.0, self._vec_to_euler(x_vec, y_vec, scular))
        self.CM2.set_scale(scular/100, 0.05, 0.05)
        self.CM2.set_ori(q[0], q[1], q[2], q[3])
        self.CM2.publish_marker([(self.s_area[1, 0] + self.s_area[0, 0])/2], 
                                [(self.s_area[1, 1] + self.s_area[0, 1])/2])
        self.latest_map = obs_map
        self.latest_vec = vec_map


class StopChecker(object):
    def __init__(self):
        self._stop_check = False

    @property
    def stop_check(self):
        return self._stop_check

    @stop_check.setter
    def stop_check(self, value):
        self._stop_check = value


class AreaScanner(object):
    def __init__(self, key_list, area_dict, func_dict):
        # status = key
        self.node_name = "area_scanner" 
        rospy.init_node(self.node_name, anonymous=True)
        self.key_list = key_list
        self.area_dict = area_dict
        self.func_dict = func_dict
        self.func_cls = {key_list[0]:WaitFunc,
                         key_list[1]:CheckObsFunc}
        self.status = self.key_list[0]
        self.SC = StopChecker()
        self._set_func()
        
        
    def run(self):
        tt_status = rospy.Subscriber("/operator/call_area_scan", String, self._update_func)
        tt_scan = rospy.Subscriber("/scan", LaserScan, self._scan_cb)
        rospy.spin()
    
    def _scan_cb(self, msg):
        self.cb_func.run(msg)
        if self.SC.stop_check:
           rospy.loginfo("new status: wait")
           self.status = self.key_list[0]
           self._set_func()
           self.SC.stop_check = False

    def _update_func(self, new_status):
        if self.status != new_status.data:
            rospy.loginfo("get status: " + new_status.data)
            if new_status.data in self.func_dict.keys():
                self.status = new_status.data
                rospy.loginfo("update function: " + self.func_dict[self.status])
                self._set_func()
            else:
                rospy.loginfo("this status is invalid")
                self.status = self.key_list[0]
                self._set_func()

    def _set_func(self):
        self.cb_func = self.func_cls[self.status](self.node_name, self.area_dict[self.status], self.SC)



if __name__ == '__main__':
    AS = AreaScanner(KEY_LIST, AREA_DICT, FUNC_DICT)
    AS.run()

