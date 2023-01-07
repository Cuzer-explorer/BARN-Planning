#!/usr/bin/env python2
# coding:utf-8
"""
Use feedback control to get vx & vw
Use PID to control vx
"""
# from threading import main_thread
import numpy as np
import math
from DWA import DWAPlanner

# Parameters
LK = 0.1  # look forward gain
LFC = 0.4  # [m] look-ahead distance
KP = 0.08  # speed proportional gain
DT = 0.02  # [s] time tick
WB = 0.4+0.03 #0.3  # [m] wheel base of vehicle

KVR = 6.5
KWA = 5
KWB = 0.15

# KVR = 3
# KWA = 8
# KWB = 1.5

TARGET_V = 1.5


class FeedBackPlanner:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w = 0.0, cx=[], cy=[], threshold=1.5):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w
        self.next_goal_ind = None
        self.cx = cx
        self.cy = cy
        self.threshold = threshold
        
    def updatePose(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        # self.v = v

    def updatePath(self, cx, cy):
        self.cx = cx
        self.cy = cy

    def cal_dist(self, px, py):
        dx = self.x - px
        dy = self.y - py
        return math.hypot(dx, dy)

    def ra_adjust(self,ra):
        while(ra > math.pi):
            ra -= 2.0*math.pi
        while(ra <= -math.pi):
            ra += 2.0*math.pi
        return ra

    # 对外调用的轨迹规划接口
    def get_track_plan(self,ob):
        method = 3  # 1用反馈控制，2用DWA，3用pure
        if method == 1: # 反馈控制
            return self.use_feedback()
        elif method == 2: # DWA
            return self.use_DWA(ob)
        elif method == 3: # pure_pursuit
            return self.use_pure()

    def use_feedback(self):
        mid_goal, Lf = self.get_mid_goal()
        tx = mid_goal[0] - self.x
        ty = mid_goal[1] - self.y
        r = math.sqrt(tx**2 + ty**2)
        b = self.ra_adjust(math.atan2(ty, tx))
        a = self.ra_adjust(b - self.yaw)
        v = KVR*r
        w = KWA*a + KWB*b
        return v, w
    
    def use_DWA(self,ob):
        mid_goal, Lf = self.get_mid_goal()
        dwa_planner = DWAPlanner()
        cur_p = [self.x, self.y, self.yaw]
        self.v, self.w = dwa_planner.dynamic_window_search(self.v,self.w,
                                                cur_p,mid_goal,ob)
        return self.v, self.w

    def use_pure(self):
        mid_goal, Lf = self.get_mid_goal()
        tx = mid_goal[0] - self.x
        ty = mid_goal[1] - self.y
        
        a = KP * (TARGET_V - self.v)
        self.v += a * DT
        alpha = math.atan2(ty, tx) - self.yaw
        w = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        
        return self.v, w

    def get_mid_goal(self):
        ind, Lf = self.search_target_index()
        if ind < len(self.cx):
            mid_goal = [self.cx[ind],self.cy[ind],0.0]
        else:  # toward goal
            mid_goal = [self.cx[-1],self.cy[-1],0.0]
        return mid_goal, Lf

    def search_target_index(self, restart = False):

        # To speed up nearest point search, doing it at only first time.
        if self.next_goal_ind is None or restart == True:
            # search nearest point index
            dx = [self.x - icx for icx in self.cx]
            dy = [self.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.next_goal_ind = ind
        else:
            ind = self.next_goal_ind
            distance_this_index = self.cal_dist(self.cx[ind],self.cy[ind])
            while True:
                if (ind+1) >= len(self.cx):
                    break
                distance_next_index = self.cal_dist(self.cx[ind + 1],self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.next_goal_ind = ind

        Lf = LK * self.v + LFC  # update look ahead distance

        # search look ahead target point index
        while Lf > self.cal_dist(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

    def reset_planner(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.next_goal_ind = None
        self.cx = []
        self.cy = []
        self.threshold = 1.5

