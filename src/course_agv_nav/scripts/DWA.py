#!/usr/bin/env python
# coding:utf-8
import numpy as np 
import math

inf = 1000000

class DWAPlanner:
    def __init__(self,map_grid = 0.1,vmin = 0,vmax = 0.2,wmin = -0.5,
                wmax = 0.5,va_max = 5000,wa_max = 5000,alpha = 1,
                beta = 0.001,gama = 0.001,dt = 0.01,dv = 0.01,dw = 0.01):
        self.vmin = vmin
        self.vmax = vmax
        self.wmin = wmin
        self.wmax = wmax
        self.va_max = va_max
        self.wa_max = wa_max

        self.alpha = alpha
        self.beta = beta
        self.gama = gama
        self.dt = dt
        self.dv = dv
        self.dw = dw
        self.map_grid = map_grid

    def velocity_window(self,v,w,position):
        vh_d = math.sqrt(2*self.dist(v,w)*self.va_max)
        wh_d = math.sqrt(2*self.dist(v,w)*self.wa_max)
        vh_a = v+self.va_max*self.dt
        vl_a = v-self.va_max*self.dt
        wh_a = w+self.wa_max*self.dt
        wl_a = w-self.wa_max*self.dt

        self.vh_s = min(self.vmax,vh_d,vh_a)
        self.vl_s = max(self.vmin,vl_a)
        self.wh_s = min(self.wmax,wh_d,wh_a)
        self.wl_s = max(self.wmin,wl_a)

        # print(self.vh_s,self.vl_s,self.wh_s,self.wl_s)
    
    def cal_R(self,v,w):
        max_radius = 1.5
        if abs(w) < 0.001:
            radius = max_radius
        else:
            radius = v / w
        return radius

    def get_predict(self,v,w,position):
        predict =[0,0,0]
        radius = self.cal_R(v,w)
        if abs(w) < 0.001:
            predict[0] = position[0]+v*self.dt*math.cos(position[2])
            predict[1] = position[1]+v*self.dt*math.sin(position[2])
        else:
            predict[0] = position[0]-radius*math.sin(position[2])+radius*math.sin(position[2]+w*self.dt)
            predict[1] = position[1]+radius*math.cos(position[2])-radius*math.cos(position[2]+w*self.dt)
        predict[2] = position[2]+w*self.dt
        return predict

    def dist(self,v,w):       
        # dist = 0
        # max_radius = 1.5
        # if abs(w) < 0.001:
        #     radius = max_radius
        # else:
        #     radius = abs(v / w)
        radius = self.cal_R(v,w)
        min_dist = min(self.ob)
        if min_dist < radius:
            return min_dist
        else:
            return radius
        # min_dist = inf
        # for x in range():
        #     for y in range():
        #         if(map[x][y]==1):
        #             dist = (x-postion[0])*(x-postion[0])+(y-postion[1])*(y-postion[1])
        #             if(dist<min_dist):
        #                 min_dist  = dist
    
    def head_cost(self,v,w,position,goal):
        goal_yaw = math.atan2(goal[1]-position[1],goal[0]-position[0])
        # yaw = position[2]+w*self.dt
        yaw = position[2]
        cost_angle = goal_yaw - yaw
        cost_w = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        # print("gy is %f,yaw is %f,cost_w is%f"%(goal_yaw,yaw,cost_w))
        # cost_w = min(abs(yaw - goal_yaw),2*math.pi)
        #dist = math.sqrt(math.pow(self.dt*math.sin(yaw),2)+math.pow(v*self.dt*math.cos(yaw),2))
        return (2*math.pi - cost_w) / (2*math.pi)

    def dist_cost(self,v,w,position):
        return self.dist(v,w)
        
    def velocity_cost(self,v):
        return v/self.vmax

    def total_cost(self,v,w,position,goal):
        h_cost = self.head_cost(v,w,position,goal)
        d_cost = self.dist_cost(v,w,position)
        v_cost = self.velocity_cost(v)
        total_cost = self.alpha*h_cost + self.beta*d_cost + self.gama*v_cost
        # print("h_cost is %f,d_cost is %f,v_cost is%f"%(h_cost,d_cost,v_cost))
        
        return total_cost,h_cost,d_cost,v_cost

    def dynamic_window_search(self,v,w,position,goal,ob):
        self.ob = ob
        max_cost = 0
        max_cost_v = 0
        max_cost_w = 0
        best_cost = [0,0,0]
        self.velocity_window(v,w,position)
        print("#######search")
        for v in np.arange(self.vl_s,self.vh_s,self.dv):
            for w in np.arange(self.wl_s,self.wh_s,self.dw):
                # print("v is %f,w is %f"%(v,w))
                predict = self.get_predict(v,w,position)
                cost,h_cost,d_cost,v_cost = self.total_cost(v,w,predict,goal)
                if(cost > max_cost):
                    max_cost = cost
                    max_cost_v = v
                    max_cost_w = w
                    best_cost = [h_cost,d_cost,v_cost]
        print(best_cost)
        print("goal: ")
        print(goal)
        print("position: ")
        print(position)
        v = max_cost_v
        w = max_cost_w
        predict = self.get_predict(v,w,position)
        goal_yaw = math.atan2(goal[1]-predict[1],goal[0]-predict[0])
        yaw = predict[2]
        cost_angle = goal_yaw - yaw
        cost_w = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        print("predict: ")
        print(predict)
        print("v is %f,w is %f"%(max_cost_v,max_cost_w))
        print("gy is %f,yaw is %f,cost_w is%f"%(goal_yaw,yaw,cost_w))


        return (max_cost_v,max_cost_w)

