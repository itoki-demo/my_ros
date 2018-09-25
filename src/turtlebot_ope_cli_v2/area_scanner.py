#!/usr/bin/env python
#coding:utf-8

import rospy
import tf
import message_filters
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32, String
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import math
import numpy as np


#scan_area = [[min_x, min_y], [max_x, max_y]]
EMPTY_AREA = np.array([[0.0, 0.0],
                         [0.0, 0.0]])
ROOM01 = np.array([[-9.5, -0.2],
                   [-8.2, 0.1]])
TEST = np.array([[-1.0, -1.0],
                 [0.0, 0.0]])
TEST2 = np.array([[-4.0, -1.0],
                  [-3.5, 0.0]])

# TASKNAMEは/call_scan_areaの内容のリスト 0番目は待機用の内容とする
TASKNAME_LIST = ["Wait", "Room01", "test", "test2"]
AREA_DICT = {TASKNAME_LIST[0]:[EMPTY_AREA],
             TASKNAME_LIST[1]:[ROOM01, ROOM01],
             TASKNAME_LIST[2]:[TEST2, TEST2],
             TASKNAME_LIST[3]:[TEST, TEST]}
FUNC_DICT = {TASKNAME_LIST[0]:["DGU"],
             TASKNAME_LIST[1]:["check_obs", "vision_check"],
             TASKNAME_LIST[2]:["check_obs", "vision_check"],
             TASKNAME_LIST[3]:["check_obs", "vision_check"]}


# マーカートピック配信用クラス
class Marker2D(object):
    def __init__(self, topic_name, r, g, b, marker_type=Marker.CUBE, task_id=0):
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
        self.task_id = task_id

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

    def set_rgb(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

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
            marker.id = i + self.task_id*1000
            marker.pose.position.x = x_list[i]
            marker.pose.position.y = y_list[i]
            marker.pose.position.z = 0.2
            self.marker_array.markers.append(marker)

    # LINE_STRIP用のパラメータ設定
    def _set_line_strip(self, x_list, y_list, length):
        marker = Marker()
        self.__set_marker_param(marker)
        marker.type = marker.LINE_STRIP
        marker.id = 1 + self.task_id
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
            marker.id = i + self.task_id * 1000
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


# funcクラスの親クラス
class Function(object):
    def __init__(self, area, sc, task_id):
        self.area = area
        self.sc = sc
        self.task_id = task_id
        self.CM0 = Marker2D("AreaScanMarker/obstacle_marker", 1.0, 0.0, 0.0, task_id=self.task_id)
        self.CM1 = Marker2D("AreaScanMarker/area_marker", 0.0, 0.0, 1.0, marker_type=4, task_id=self.task_id)
        self.CM2 = Marker2D("AreaScanMarker/vec_marker", 0.0, 1.0, 0.0, marker_type=0, task_id=self.task_id)
        self.x_list = np.zeros(0)
        self.y_list = np.zeros(0)
        # pos = [position.x, position.y, angle]
        self.pos = np.array([0, 0, 0], dtype='float')
        self.angle_list = np.zeros(0)
        self.listener = tf.TransformListener()
        self.trans = 0
        self.rot = 0
        self.status = GoalStatusArray()
        self.goal = MoveBaseActionGoal()

    # /scanトピックをmap座標に変換したリストを作成
    def _get_point_list(self, scan_msg):
        executable = False
        if len(self.angle_list) == 0:
            self._init_list(scan_msg)
        try:
            (self.trans, self.rot) = self.listener.lookupTransform('/map', '/rplidar_frame', rospy.Time(0))
            executable = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        if executable:
            self.pos[0] = self.trans[0]
            self.pos[1] = self.trans[1]
            self.pos[2] = self._quaternion_to_euler(self.rot[0], self.rot[1], self.rot[2], self.rot[3]) + np.pi
            # センサー情報を極座標からxy(z)座標へ変換＋map座標とturtlebot位置の差分を加える　
            self.x_list = np.array(scan_msg.ranges) * np.cos((self.pos[2]-self.angle_list)) + self.pos[0]
            self.y_list = np.array(scan_msg.ranges) * np.sin((self.pos[2]-self.angle_list)) + self.pos[1]
        return executable

    # /scanトピック情報を利用してリストを初期化
    def _init_list(self, msg):
        temp_angle = np.arange(msg.angle_min,
                               msg.angle_min + np.pi * 2,
                               msg.angle_increment, dtype='float') 
        self.angle_list = np.zeros(len(msg.ranges), dtype='float') +\
                          temp_angle[:len(msg.ranges)] - np.pi
        self.x_list = np.zeros(len(msg.ranges), dtype='float')
        self.y_list = np.zeros(len(msg.ranges), dtype='float')

    def set_status(self, status_msg):
        self.status = status_msg

    def set_goal(self, goal_msg):
        self.goal = goal_msg

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


# navigation時に移動目標地点の「ずらし」を行うクラス
class DynamicGoalUpdateFunc(Function):
    def __init__(self, area, sc, task_id):
        super(DynamicGoalUpdateFunc, self).__init__(area, sc, task_id)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.next_goal = MoveBaseGoal()
        self.previous_status = 0
        self.units_len = 10
        self.goal_radius = 1.0
        # 2次元ガウス関数マップ(g_widthは絶対に奇数)
        self.g_width = 9
        self.g_map = np.zeros((self.g_width, self.g_width), dtype='float')
        self._get_2d_gaussian(self.g_width)
        # 2次元配列格子の数を設定
        self.xy = int(math.ceil(self.goal_radius*2 * self.units_len)) + self.g_width
        self.obs_map = np.zeros((self.xy, self.xy), dtype='float')
        self.ind_list = np.zeros((2, 0), dtype='int')
        self.threshold = 1.0e-05
        
    def run(self, scan_msg):
        l = len(self.status.status_list)
        if l > 0:
            if self.status.status_list[l-1].status == 1:
                # ナビゲーション実行中処理
                if self.previous_status != 1:
                    rospy.loginfo("dynamic goal updating started")
                # レーザースキャン情報取得
                if self._get_point_list(scan_msg):
                    # ゴール周辺の障害物情報配列を取得
                    self._get_map_index()
                    # ゴールポイント上のコスト計算
                    if self._get_goal_cost():
                        self._copy_goal()
                        # ゴールポイント更新
                        self._update_goal()
            elif self.status.status_list[l-1].status == 3:
                if self.previous_status != 3:
                    # ゴールしたらnext_goalは初期化
                    self.next_goal = MoveBaseGoal()
                    rospy.loginfo("dynamic goal updating stoped")
            else:
                pass
            # 直前のgoal_statusを記録 goal_statusの変化を判断できるようにする
            self.previous_status = self.status.status_list[l-1].status

    def _update_goal(self):
        costmap = np.zeros((self.xy, self.xy), dtype='float') + 1.0
        for n in range(len(self.ind_list[0])):
            min_x = int(self.ind_list[0, n] - math.floor(self.g_width/2))
            min_y = int(self.ind_list[1, n] - math.floor(self.g_width/2))
            max_x = int(self.ind_list[0, n] + math.floor(self.g_width/2)) + 1
            max_y = int(self.ind_list[1, n] + math.floor(self.g_width/2)) + 1
            costmap[min_x:max_x, min_y:max_y] *= (1.0 - self.g_map)
            #costmap[min_x:max_x, min_y:max_y] -= self.g_map
        distance = np.zeros((self.xy, self.xy), dtype='float')
        xy0 = math.floor(self.xy/2)
        for x in range(self.xy):
            for y in range(self.xy):
                distance[x, y] = math.sqrt((x-xy0)**2+(y-xy0)**2)/(self.xy*10)
        s_map = self._get_slope_map(self.xy,
                                    self.pos[0] - self.next_goal.target_pose.pose.position.x,
                                    self.pos[1] - self.next_goal.target_pose.pose.position.y,
                                    p_flat=True)
        costmap *= s_map + 1.0
        costmap *= 1.0 - distance
        #costmap *= (costmap >= self.threshold).astype('int')
        max_ind = [np.where(costmap==np.max(costmap))[0][0], np.where(costmap==np.max(costmap))[1][0]]
        self.next_goal.target_pose.pose.position.x += ((max_ind[0]-self.xy/2)*1.0)/self.units_len
        self.next_goal.target_pose.pose.position.y += ((max_ind[1]-self.xy/2)*1.0)/self.units_len
        self.client.send_goal(self.next_goal)
        self.previous_status = 0
        rospy.loginfo("update goal")

    @staticmethod
    def _get_slope_map(size, vx=1, vy=0, n_flat=False, p_flat=False):
        n = 1
        p = 1
        if n_flat:
            n = 0
        if p_flat:
            p = 0
        s_map = np.zeros((size, size), dtype='float')
        theta1 = math.atan2(vy, vx)
        for x in range(int(math.ceil(-size*1.0/2)), int(math.ceil(size*1.0/2))):
            for y in range(int(math.ceil(-size*1.0/2)), int(math.ceil(size*1.0/2))):
                theta2 = math.atan2(y, x)
                distance = (math.sqrt(x**2 + y**2) * math.cos(theta1 - theta2)) / int(math.floor(size*1.0/2))
                if distance > 0:
                    distance *= p
                else:
                    distance *= n
                s_map[x+int(math.floor(size*1.0/2)), y+int(math.floor(size*1.0/2))] = round(distance, 10)
        return s_map
        
    def _copy_goal(self):
        self.next_goal = MoveBaseGoal()
        self.next_goal.target_pose.header.frame_id = 'map'
        self.next_goal.target_pose.pose.position.x = self.goal.goal.target_pose.pose.position.x
        self.next_goal.target_pose.pose.position.y = self.goal.goal.target_pose.pose.position.y
        self.next_goal.target_pose.pose.position.z = self.goal.goal.target_pose.pose.position.z
        self.next_goal.target_pose.pose.orientation.x = self.goal.goal.target_pose.pose.orientation.x
        self.next_goal.target_pose.pose.orientation.y = self.goal.goal.target_pose.pose.orientation.y
        self.next_goal.target_pose.pose.orientation.z = self.goal.goal.target_pose.pose.orientation.z
        self.next_goal.target_pose.pose.orientation.w = self.goal.goal.target_pose.pose.orientation.w

    def _get_goal_cost(self):
        r = False
        min_xy = int(math.floor(self.xy/2) - math.floor(self.g_width/2))
        max_xy = int(math.floor(self.xy/2) + math.floor(self.g_width/2)) + 1
        t_costmap = self.obs_map[min_xy:max_xy, min_xy:max_xy] * self.g_map
        if np.sum(t_costmap) > self.threshold:
            r = True
        return r

    def _get_map_index(self):
        self.obs_map = np.zeros((self.xy, self.xy), dtype='float')
        self.ind_list = np.zeros((2, 0), dtype='int')
        # obs_mapをscanポイントで塗りつぶす（obs_mapのある格子にscanポイントがある場合、1とする）、インデックスを取得
        min_x = self.goal.goal.target_pose.pose.position.x - self.goal_radius
        min_y = self.goal.goal.target_pose.pose.position.y - self.goal_radius
        max_x = self.goal.goal.target_pose.pose.position.x + self.goal_radius
        max_y = self.goal.goal.target_pose.pose.position.y + self.goal_radius
        for n in range(len(self.x_list)):
            if min_x < self.x_list[n] < max_x and min_y < self.y_list[n] < max_y:
                cell_x = int(math.floor((self.x_list[n] - min_x)*self.units_len) + math.floor(self.g_width/2))
                cell_y = int(math.floor((self.y_list[n] - min_y)*self.units_len) + math.floor(self.g_width/2))
                if self.obs_map[cell_x, cell_y] == 0:
                    self.ind_list = np.hstack((self.ind_list, [[cell_x], [cell_y]]))
                    self.obs_map[self.ind_list[0, len(self.ind_list[0])-1],
                                 self.ind_list[1, len(self.ind_list[0])-1]] = 1

    def _get_2d_gaussian(self, w, r=3.0):
        x0 = math.floor(w/2)
        y0 = x0
        for x in range(w):
            for y in range(w):
                self.g_map[x, y] = math.exp(-(((x-x0)/(r*2))**2+((y-y0)/(r*2))**2))


# 障害物有無判定funcクラス _check関数で判定の条件を記述
class CheckObsFunc(Function):
    def __init__(self, area, sc, task_id):
        super(CheckObsFunc, self).__init__(area, sc, task_id)
        self.check_count = 0

    def run(self, scan_msg):
        if self._get_point_list(scan_msg):
            # 指定エリア内にあるスキャン点をTrueとする　
            x_flg_array = (self.x_list > self.area[0, 0]) * (self.x_list < self.area[1, 0])
            y_flg_array = (self.y_list > self.area[0, 1]) * (self.y_list < self.area[1, 1])
            area_flg_array = x_flg_array * y_flg_array

            # 指定エリア内にあるスキャン点に対しいろいろ処理　
            obs_num = np.sum(area_flg_array)
            self.CM0.publish_marker(self.x_list[area_flg_array], self.y_list[area_flg_array])
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
            check_msg = self._check(obs_num)
            #rospy.loginfo(str(obs_num) + ", " + str(check_msg))
            self.sc.set_stop_check(self.task_id, check_msg)

    # 障害物がないと判定する条件
    def _check(self, obs_num):
        r = False
        if obs_num == 0:
            self.check_count += 1
        else:
            self.check_count = 0
        if self.check_count > 10:
            r = True
        return r


# 対象領域まで、視界が確保できているかの確認を行うクラス
class VisionCheckFunc(Function):
    def __init__(self, area, sc, task_id):
        super(VisionCheckFunc, self).__init__(area, sc, task_id)
        # スキャン領域の頂点座標4点分
        self.v_list = np.array([[self.area[0, 0], self.area[0, 1]],
                                [self.area[1, 0], self.area[0, 1]],
                                [self.area[0, 0], self.area[1, 1]],
                                [self.area[1, 0], self.area[1, 1]]], dtype='float')
        # 領域を求めるための頂点を指定するテーブル [[[（亀から見て）左の頂点, 右の頂点]]] v_set_table[2, 2]は計算しないこと
        self.v_set_table = np.array([[[1, 2], [1, 0], [3, 0]],
                                     [[0, 2], [9, 9], [3, 1]],
                                     [[0, 3], [2, 3], [2, 1]]], dtype='int')
        # 亀から対象領域頂点までの直線の、「視界の内側」を決めるテーブル 補足は上と同様
        self.flip_table = np.array([[[1, -1], [1, 1], [-1, 1]],
                                    [[1, -1], [0, 0], [-1, 1]],
                                    [[1, -1], [-1, -1], [-1, 1]]], dtype='int')
        self.CM0.set_rgb(1.0, 0.5, 0.5)
        self.CM1.set_rgb(0.5, 0.5, 1.0)
        self.check_count = 0

    def run(self, scan_msg):
        if self._get_point_list(scan_msg):
            table_x = self._assign_x(self.pos[0])
            table_y = self._assign_y(self.pos[1])
            if table_x != 1 or table_y != 1:
                l_vertex = self.v_list[self.v_set_table[table_y, table_x][0]]
                r_vertex = self.v_list[self.v_set_table[table_y, table_x][1]]
                l_flip = self.flip_table[table_y, table_x][0]
                r_flip = self.flip_table[table_y, table_x][1]
                flg_array_1 = self._view_check_1(l_vertex, l_flip, table_x) *\
                               self._view_check_1(r_vertex, r_flip, table_x, -1)
                flg_array_2 = self._view_check_2(table_x, table_y)
                area_flg_array = flg_array_1 * flg_array_2
                self.CM0.publish_marker(self.x_list[area_flg_array], self.y_list[area_flg_array])
                self.CM1.publish_marker([l_vertex[0],
                                         self.pos[0], 
                                         r_vertex[0]],
                                        [l_vertex[1],
                                         self.pos[1],
                                         r_vertex[1]])

                # 指定エリア内にあるスキャン点に対しいろいろ処理　
                obs_num = np.sum(area_flg_array)

            else:
                obs_num = 0

            check_msg = self._check(obs_num)
            #rospy.loginfo(str(obs_num) + ", " + str(check_msg))
            self.sc.set_stop_check(self.task_id, check_msg)
            
    # if x < min_x: x = 0, elif min_x <= x < max_x: x = 1, else: x = 2
    def _assign_x(self, x_list):
        return (x_list >= self.area[1, 0]).astype(np.int64) + ~(x_list < self.area[0, 0]).astype(np.int64) + 2
        
    def _assign_y(self, y_list):
        return (y_list >= self.area[1, 1]).astype(np.int64) + ~(y_list < self.area[0, 1]).astype(np.int64) + 2

    # スキャン領域までの視界の内側に入っている障害物判定
    def _view_check_1(self, vertex, flip, table_x, side=1):
        if vertex[0] - self.pos[0] != 0:
            a = (vertex[1] - self.pos[1])/(vertex[0] - self.pos[0])
            b = vertex[1] - a * vertex[0]
            r_list = (0 > ((a * self.x_list + b - self.y_list) * flip))
        else:
            # left: side = 1, right: side = -1
            if table_x == 1 or table_x == 3:
                r_list = 0 <= ((vertex[0] - self.x_list) * side)
            else:
                r_list = 0 <= ((vertex[0] - self.x_list) * -1 * side)
        return r_list

    # スキャン領域より手前に入っている障害物判定
    def _view_check_2(self, table_x, table_y):
        x_assign_list = self._assign_x(self.x_list)
        y_assign_list = self._assign_y(self.y_list)
        if table_x == 1 or table_y == 1:
            r_list = (table_x == x_assign_list) * (table_y == y_assign_list)
        else:
            r_list = ~(~(table_x == x_assign_list) * ~(table_y == y_assign_list))
        return r_list

    # 障害物がないと判定する条件
    def _check(self, obs_num):
        r = False
        if obs_num == 0:
            self.check_count += 1
        else:
            self.check_count = 0
        if self.check_count > 10:
            r = True
        return r


# task終了条件を満たしたことをAreaScanerに伝えるために使うクラス task終了時、/area_scanにて"True"を配信
class StopChecker(object):
    def __init__(self, node_name, max_num):
        self.node_name = node_name
        self.max_num = max_num
        self._stop_check = np.array([], dtype='bool')
        for n in range(self.max_num):
            self._stop_check = np.append(self._stop_check, False)
        self.pub = rospy.Publisher(self.node_name + "/area_scan", String, queue_size = 1)
        self.pub_msg = "True"

    def init_stop_check(self, task_num):
        # 同時task数0の場合は待機としてみなすので、_stop_checkはすべてFalseとしておく
        if task_num == 0:
            for n in range(self.max_num):
                self._stop_check[n] = False
        else:
            for n in range(task_num):
                self._stop_check[n] = False
            for n in range(task_num, self.max_num):
                self._stop_check[n] = True

    def stop_and_publish(self):
        r = False
        if np.sum(self._stop_check) == self.max_num:
            r = True
            self.pub.publish(self.pub_msg)
        return r

    def get_stop_check(self, task_id):
        return self._stop_check[task_id]

    def set_stop_check(self, task_id, value):
        self._stop_check[task_id] = value


# 特定領域に対して、障害物の有無や振る舞い等からフィードバックを返すクラス
# メインの処理は/scanを購読するごとに実行 /call_area_scanにより処理内容(task)を変更する
class AreaScaner(object):
    def __init__(self, taskname_list, area_dict, func_dict, multi_task_num=3):
        # taskname = /call_area_scanの内容
        # func: ある領域に対する判定関数（あるいはクラス）
        # task: 領域とfuncの組み合わせ または funcクラスにareaを与えたオブジェクト
        self.node_name = "area_scanner" 
        rospy.init_node(self.node_name, anonymous=True)
        self.taskname_list = taskname_list
        self.area_dict = area_dict
        self.func_dict = func_dict
        self.func_cls_dict = {"DGU":DynamicGoalUpdateFunc,
                              "check_obs":CheckObsFunc, 
                              "vision_check":VisionCheckFunc}
        self.func_len_dict = {}
        for tn in self.taskname_list:
            self.func_len_dict[tn] = len(self.func_dict[tn])
        # func_dict: /call_area_scanの内容をkeyとし、areaを格納する予定のfuncクラスのリストを値とする
        self.task_dict = {}
        self.multi_task_num = multi_task_num
        for tn in self.taskname_list:
            temp_list = []
            for f in self.func_dict[tn]:
                temp_list.append(self.func_cls_dict[f])
            self.task_dict[tn] = temp_list

        self.SC = StopChecker(self.node_name, self.multi_task_num)
        self.current_task = ""
        self.cb_task = []
        self._set_task(self.taskname_list[0])
        
    def run(self):
        rospy.Subscriber("/operator/call_area_scan", String, self._update_task_cb)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._update_status)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self._update_goal)
        rospy.spin()
    
    # /scanの内容をfuncに送り続け、都度終了条件を満たしたか確認する
    def _scan_cb(self, scan_msg):
        for task in self.cb_task:
            task.run(scan_msg)
        if self.SC.stop_and_publish():
            rospy.loginfo("task finished: " + self.current_task)
            self._set_task(self.taskname_list[0])

    # 新たな/call_area_scanの内容を反映する
    def _update_task_cb(self, new_task):
        rospy.loginfo("got new task")
        if self.current_task != new_task.data:
            if new_task.data in self.task_dict.keys():
                self._set_task(new_task.data)
            else:
                rospy.loginfo("this task is invalid: " + new_task.data)
                rospy.loginfo("keep current task: " + self.current_task)
                self._set_task(self.taskname_list[0])
        else:
            rospy.loginfo("keep current task: " + self.current_task)

    # _scan_cbで実行するtaskを変更する
    def _set_task(self, new_task):
        self.current_task = new_task
        rospy.loginfo("task updated: " + self.current_task)
        self.SC.init_stop_check(self.func_len_dict[self.current_task])
        self.cb_task = []
        for t in range(self.func_len_dict[self.current_task]):
            self.cb_task.append(self.task_dict[self.current_task][t](self.area_dict[self.current_task][t],
                                                                     self.SC, t))

    # navigationのstatusを更新
    def _update_status(self, status_msg):
        for task in self.cb_task:
            task.set_status(status_msg)

    def _update_goal(self, goal_msg):
        for task in self.cb_task:
            task.set_goal(goal_msg)


# 実行例
if __name__ == '__main__':
    AS = AreaScaner(TASKNAME_LIST, AREA_DICT, FUNC_DICT)
    AS.run()

