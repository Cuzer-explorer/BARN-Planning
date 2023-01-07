#!/usr/bin/env python
# coding:utf-8
""" Use A* algorithm for path planning """
import heapq
from math import sqrt
import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
import time
from JPS_handler_6 import JPS_Handler, Node


class State(object):
    """ State of position in the map """
    def __init__(self, barrier):
        self.barrier = barrier
        self.pre_x = 0    # Previous point coordinate
        self.pre_y = 0
        self.g = 0   # Distance to the starting point
        self.in_openlist = False
        self.in_closelist = False

class AStar:
    def __init__(self,real_ox,real_oy,space,radius,x_max,x_min,y_max,y_min):
        self.space = space  # 间距
        self.radius = radius
        self.x_max = x_max
        self.x_min = x_min
        self.y_max = y_max
        self.y_min = y_min
        self.plan_x = []
        self.plan_y = []
        self.alpha = 1
        self.beta = 1
        self.ox = real_ox # self.tf(np.array(real_ox),'x')    # 栅格化的障碍物坐标，array
        self.oy = real_oy # self.tf(np.array(real_oy),'y')
        # create map
        start_time = time.time()
        map_x_max = self.tf(self.x_max,'x')+10
        map_y_max = self.tf(self.y_max,'y')+10
        print("[AStar] Map size: %d, %d" % (map_x_max,map_y_max))
        self.m = [[State(False) for j in range(map_y_max)] for i in range(map_x_max)]
        self.ob_map = [[0 for j in range(map_y_max)] for i in range(map_x_max)]
        for i in range(len(self.ox)):
            # self.m[self.ox[i]][self.oy[i]].barrier = True
            self.add_barrier(self.ox[i],self.oy[i])
        end_time = time.time()
        print("[AStar] Map create finish, use %f seconds" % (end_time - start_time))

    
    def add_barrier(self,ox,oy):
        # px, py = (self.tf(ox, 'x'), self.tf(oy, 'y'))
        for i in range(5):
            for j in range(5):
                px, py = ox+i-2, oy+j-2
                if self.in_map(px,py):
                    self.m[px][py].barrier = True
        """
        x_m = ox + self.radius
        y_m = oy + self.radius
        x = ox - self.radius/3.0*2
        y = oy - self.radius/3.0*2 
        while x <= x_m:
            while y <= y_m:
                px, py = (self.tf(x, 'x'), self.tf(y, 'y'))
                if self.in_map(px,py):
                    # print(px,py)
                    self.m[px][py].barrier = True
                    self.ob_map[px][py] = 1
                y += self.space
            x += self.space
            y = oy - self.radius
        """

    # Conversion between real coordinates and map coordinates
    def tf(self, real, ty):  # real to map
        if ty == 'x':
            map = ((real - self.x_min)/self.space)
        else:
            map = ((real - self.y_min)/self.space)
        if isinstance(real, np.ndarray):
            map = map.astype(int)
        else:
            map = int(map)
        return map

    def in_tf(self, map, ty):   # map to real
        if ty == 'x':
            return self.x_min + (map+0.5)*self.space
        else:
            return self.y_min + (map+0.5)*self.space

    # Calculate the geometric parameters of the point
    def cal_h(self,px1,py1,px2,py2):
        return abs(px1-px2) + abs(py1-py2)

    def cal_g(self,px1,py1,px2,py2):
        return sqrt((px1-px2)**2 + (py1-py2)**2)

    def equal(self,px1,py1,px2,py2):
        return (px1 == px2 and py1 == py2)
    
    def in_map(self,px,py):
        return (px >= 0 and px <= self.tf(self.x_max,'x') and py >= 0 and py <= self.tf(self.y_max,'y'))

    # 对外调用的路径规划接口
    def planning(self,real_sx,real_sy,real_gx,real_gy):
        method = 1  # 1使用A星，2使用Dj，3使用贪婪，4使用JPS
        alpha_list = [1, 1, 0]
        beta_list = [1, 0, 1]
        if method <= 3:
            self.alpha = alpha_list[method-1]
            self.beta = beta_list[method-1]
            return self.use_AStar(real_sx,real_sy,real_gx,real_gy)
        elif method == 4:   # 使用JPS
            return self.use_JPS(real_sx,real_sy,real_gx,real_gy)
    

    def use_JPS(self,real_sx,real_sy,real_gx,real_gy):
        JPS_handle = JPS_Handler(self.ob_map)
        sx,sy = self.tf(real_sx,'x'),self.tf(real_sy,'y')
        gx,gy = self.tf(real_gx,'x'),self.tf(real_gy,'y')
        startnode = Node(None, sx, sy, 0)
        finalnode = Node(None, gx, gy, 0)

        start_time = time.time()
        path = JPS_handle.Find_Path(startnode, finalnode)
        end_time = time.time()
        print("[AStar] Path planning finish, use %f seconds" % (end_time - start_time))
       
        plan_rx,plan_ry = [],[]
        for i in range(len(path)):
            px,py = path[i]
            real_x,real_y = (self.in_tf(px, 'x'), self.in_tf(py, 'y'))
            plan_rx.append(real_x)
            plan_ry.append(real_y)
            self.plan_x.append(px)
            self.plan_y.append(py)
        # open_list = JPS_handle.OpenList
        # close_list = JPS_handle.CloseList
        # for i in range(len(open_list)):
        #     px = open_list[i][1].x
        #     py = open_list[i][1].y
        #     self.m[px][py].in_openlist = True
        for i in range(len(JPS_handle.map)):
            for j in range(len(JPS_handle.map[0])):
                if JPS_handle.map[i][j] == 2:
                    self.m[i][j].in_openlist = True 
                elif JPS_handle.map[i][j] == 3:
                    self.m[i][j].in_closelist = True
        # for i in range(len(close_list)):
        #     px = close_list[i].x
        #     py = close_list[i].y
        #     self.m[px][py].in_closelist = True  
        return plan_rx,plan_ry 


    # Use A* algorithm to find the path
    def use_AStar(self,real_sx,real_sy,real_gx,real_gy):
        path_found = True
        open_list = []
        sx,sy = self.tf(real_sx,'x'),self.tf(real_sy,'y')
        gx,gy = self.tf(real_gx,'x'),self.tf(real_gy,'y')
        print("=> start:(%d,%d), goal:(%d,%d)" % (sx,sy,gx,gy))

        px,py = sx,sy  # temp position
        self.m[px][py].in_closelist = True
        start_time = time.time()
        if self.m[gx][gy].barrier:
            print("[AStar] Path error. goal is barrier.")
            path_found = False
        else:
            while not self.equal(px,py,gx,gy):
                # search p's all neighboring points
                for i in range(3):
                    for j in range(3):
                        nx,ny = px-1+i, py-1+j  # neighboring point
                        if not self.in_map(nx,ny):
                            continue
                        if self.m[nx][ny].barrier:
                            continue
                        pt_g_new = self.m[px][py].g + self.cal_g(nx,ny,px,py)
                        pt_g_old = self.m[nx][ny].g
                        if pt_g_old <= pt_g_new and (self.m[nx][ny].in_closelist or self.m[nx][ny].in_openlist):
                            continue
                        # update pt's state and add pt to openlist
                        self.m[nx][ny].g = pt_g_new
                        self.m[nx][ny].pre_x = px
                        self.m[nx][ny].pre_y = py
                        pt_f = self.alpha*self.m[px][py].g + self.beta*self.cal_h(nx,ny,gx,gy)
                        heapq.heappush(open_list, (pt_f, nx, ny))
                        self.m[nx][ny].in_openlist = True
                if not open_list:
                    path_found = False
                    break
                else:
                    # Pop up with the smallest f
                    (_, px, py) = heapq.heappop(open_list)
                    self.m[px][py].in_openlist = False
                    self.m[px][py].in_closelist = True
        # Backtracking
        plan_rx,plan_ry = [],[]
        if path_found:
            print("[AStar] Find path.")
            px,py = gx,gy
            while not self.equal(px,py,sx,sy):
                real_x,real_y = (self.in_tf(px, 'x'), self.in_tf(py, 'y'))
                self.plan_x.append(px)
                self.plan_y.append(py)
                plan_rx.append(real_x)
                plan_ry.append(real_y)
                px,py = self.m[px][py].pre_x, self.m[px][py].pre_y
            real_x,real_y = (self.in_tf(px, 'x'), self.in_tf(py, 'y'))
            plan_rx.append(real_x)
            plan_ry.append(real_y)
            self.plan_x.append(px)
            self.plan_y.append(py)
        else:
            plan_rx.append(real_sx)
            plan_ry.append(real_sy)
            print("[AStar] Path not found.")
        end_time = time.time()
        print("[AStar] Path planning finish, use %f seconds" % (end_time - start_time))
        return plan_rx,plan_ry 

    def save_map(self):
        col_max = self.tf(self.y_max,'y')+1
        row_max = self.tf(self.x_max,'x')+1
        map_mat = [[0 for j in range(col_max)] for i in range(row_max)]
        for i in range(row_max):
            for j in range(col_max):
                if self.m[i][j].barrier:
                    map_mat[i][j] = 2
                elif self.m[i][j].in_openlist:
                    map_mat[i][j] = 3
                elif self.m[i][j].in_closelist:
                    map_mat[i][j] = 4 
        for i in range(len(self.ox)):
            px, py = (self.tf(self.ox[i], 'x'), self.tf(self.oy[i], 'y'))
            for ii in [0,1]:
                for jj in [0,1]:
                    if self.in_map(px+ii,py+jj):
                        map_mat[px+ii][py+jj] = 1
        for i in range(len(self.plan_x)):
            map_mat[self.plan_x[i]][self.plan_y[i]] = 5
        """
        map_data = np.array(map_mat)
        map_writer = pd.DataFrame(map_mat)
        # plt.imshow(map_data)
        # plt.colorbar(shrink=0.9)
        # plt.show()
        map_id = 0
        f_name = "/home/zavier/robot/car_ws/src/c9-10/course_agv_nav/scripts/map5/map5_" + str(map_id) + ".csv"
        map_writer.to_csv(f_name, index=False)
        print("[AStar] Save map to " + f_name)
        """
        return map_mat, col_max, row_max


if __name__ == "__main__":
    print("A_star")