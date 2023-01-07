#!/usr/bin/env python2
# coding:utf-8
from enum import unique
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from course_agv_nav.srv import Plan,PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import time
# import matplotlib.pyplot as plt
from nav_msgs.msg import GridCells

## TODO import your own planner
from AStar import AStar

class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0  # 起点
        self.plan_sy = 0.0
        self.plan_gx = 8.0  # 目标
        self.plan_gy = -8.0
        self.plan_grid_size = 0.15  # 0.3
        self.plan_robot_radius = 0.2   # 0.6   # 仿真0.55，实物0.3
        self.plan_ox = []   # 障碍物
        self.plan_oy = []
        self.plan_rx = []   # 路径
        self.plan_ry = []

        # count to update map
        self.map_count = 0

        self.tf = tf.TransformListener() # 获取当前信息
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan) # 获取目标点
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid, self.mapCallback)
        self.expand_pub = rospy.Publisher('/course_agv/expand_map', GridCells, queue_size=10)
        self.search_pub = rospy.Publisher('/course_agv/search_map', GridCells, queue_size=10)
        # self.updateMap()
        # self.initPlanner()
        # 
        time.sleep(1.5)
        self.updateGlobalPose()
        self.initPlanner()
        pass

    def goalSet(self):
        self.plan_gx = self.plan_sx + 0 # 目标坐标
        self.plan_gy = self.plan_sy + 10
        print("=> Start:", self.plan_sx, self.plan_sy)
        print("=> Goal:",self.plan_gx,self.plan_gy)
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0) # 每次开始规划的起点

    def goalCallback(self,msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x # 目标坐标
        self.plan_gy = msg.pose.position.y
        print("=> Goal:",self.plan_gx,self.plan_gy)
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0) # 每次开始规划的起点
        pass

    def collisionCallback(self,msg):
        self.replan(0)
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/base_link',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0] # 起点坐标？
        self.plan_sy = self.trans[1]

    # 需要修改的部分！！！
    def replan(self,req):
        print('get request for replan!!!!!!!!')
        
        self.updateGlobalPose()
        ## TODO get planner result
        ## e.g. self.plan_rx,self.plan_ry = self.planner.planning(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        self.plan_rx,self.plan_ry = self.planner.planning(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        self.publishPath()
        map_mat, col_max, row_max = self.planner.save_map()   # 保存地图及规划结果
        self.publishMap(map_mat, col_max, row_max)
        res = True
        
        return PlanResponse(res)

    def initPlanner(self):
        map_data = np.array(self.map.data).reshape((self.map.info.height, -1)).transpose() # 实物
        # map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose() # 仿真
        # 0-100的数，实际机器人可能有-1，下面这条判断可能需要修改
        # unique,count = np.unique(map_data,return_counts = True)
        # data_count = dict(zip(unique,count))
        # print(data_count)
        # map_data[map_data == 100] = 1
        # 交换x和y。。。。？
        # print(map_data)
        # print(map_data.shape)
        # ox,oy = np.nonzero(map_data > 50)   # 仿真
        ox,oy = np.nonzero(map_data != 0) # 实物
        self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist() # 将下标转为真实坐标，先膨胀后栅格化？
        self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        ## TODO init your planner
        ## e.g. self.planner = Planner(...)
        # print(self.plan_oy)
        # print("==> Start Astar")
        # print(ox,oy)
        # print(self.map.info.resolution) # 0.15000000596
        # print(max(self.plan_ox), min(self.plan_ox))
        # print(max(self.plan_oy), min(self.plan_oy))
        x_max, x_min = max(self.plan_ox), min(self.plan_ox)

        self.planner = AStar(ox, oy, self.plan_grid_size, self.plan_robot_radius, x_max, x_min, 14, 0)
        # self.planner = AStar(self.plan_ox, self.plan_oy, self.plan_grid_size, self.plan_robot_radius,
        #                     max(self.plan_ox), min(self.plan_ox),
        #                     max(self.plan_oy), min(self.plan_oy))
        self.goalSet()

    def mapCallback(self,msg):
        self.map = msg
        pass

    def updateMap(self):
        rospy.wait_for_service('/map')  # /static_map
        try:
            getMap = rospy.ServiceProxy('/map',GetMap) # 获取地图，由于是静态地图，只需要拿一次
            msg = getMap().map
            # /jackal_helper/worlds/BARN/map_files/map_pgm_xxx.pgm

        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        print("=> Load static map")
        self.mapCallback(msg)

    def publishMap(self, map_mat, col_max, row_max):
        expand_map = GridCells() 
        expand_map.header.frame_id = "map"
        expand_map.cell_height = self.plan_grid_size
        expand_map.cell_width = self.plan_grid_size
        exp_id = 0
        # expand_map.cells.resize(3)
        search_map = GridCells() 
        search_map.header.frame_id = "map"
        search_map.cell_height = self.plan_grid_size
        search_map.cell_width = self.plan_grid_size
        # 发送膨胀区域和搜索区域（openlist+closelist-path）
        for i in range(row_max):
            for j in range(col_max):
                if map_mat[i][j] == 2:
                    exp_point = Point()
                    exp_point.x = self.planner.in_tf(i,'x')
                    exp_point.y = self.planner.in_tf(j,'y')
                    exp_point.z = 0
                    expand_map.cells.append(exp_point)
                elif map_mat[i][j] == 3 or map_mat[i][j] == 4:
                    sea_point = Point()
                    sea_point.x = self.planner.in_tf(i,'x')
                    sea_point.y = self.planner.in_tf(j,'y')
                    sea_point.z = 0
                    search_map.cells.append(sea_point)
        self.expand_pub.publish(expand_map)
        self.search_pub.publish(search_map)
    
    def publishPath(self):
        path = Path() # 查Ros path文档
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped() 
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i] # 逆序，方便找点
            pose.pose.position.y = self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
    # print("Hello World!")
    # /jackal_helper/worlds/BARN/map_files/map_pgm_xxx.pgm