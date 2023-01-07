# -*- coding: utf-8 -*-
"""
Created on Tue May 25 11:52:30 2021

@author: gaoyichun
"""
from math import sqrt
import heapq

class Dir:
    up = [-1, 0]
    right =[0, 1]
    down = [1, 0]
    left = [0, -1]
    leftup = [-1, -1]
    rightup = [-1, 1]
    rightdown = [1, 1]
    leftdown = [1, -1]
    
class State:
    In_OpenList = 0b10
    In_CloseList = 0b11

MovDir = [[1, 0], [0, 1], [0, -1], [-1, 0],
          [1, 1], [1, -1], [-1, 1], [-1, -1]]

class Node(object):
    def __init__(self, parent, x, y, g):
        self.parent = parent
        self.x = x
        self.y = y
        self.g = g
        self.h = 100000#h初始化为一个很大的数
        self.f = g + self.h
    def Get_Dir(self):
        if self.parent == None:
            return 0;
        else:
            if self.x - self.parent.x < 0 and self.y - self.parent.y == 0:
                return Dir.up
            elif self.x - self.parent.x > 0 and self.y - self.parent.y == 0:
                return Dir.down
            elif self.x - self.parent.x == 0 and self.y - self.parent.y < 0:
                return Dir.left
            elif self.x - self.parent.x == 0 and self.y - self.parent.y > 0:
                return Dir.right
            elif self.x - self.parent.x < 0 and self.y - self.parent.y > 0:
                return Dir.rightup
            elif self.x - self.parent.x < 0 and self.y - self.parent.y < 0:
                return Dir.leftup
            elif self.x - self.parent.x > 0 and self.y - self.parent.y > 0:
                return Dir.rightdown
            elif self.x - self.parent.x > 0 and self.y - self.parent.y < 0:
                return Dir.leftdown
        

class JPS_Handler(object):
    def __init__(self, gridmap):
        self.map = gridmap
        
        self.WIDTH = len(self.map)
        self.LENTH = len(self.map[0])
        self.OpenList = []
        self.CloseList = []
    
    def Is_Grid_Aviliable(self, x, y):
        if x >= 0 and x < self.WIDTH and y >= 0 and y < self.LENTH:
            '''
            if self.map[x][y] == 0 or self.map[x][y] == 2:
                return 1           
            '''
            return not self.map[x][y]
        return 0
    
    def Get_h(self, node, finalnode):
        return sqrt((node.x - finalnode.x)**2 + (node.y - finalnode.y)**2)
    def Get_g(self, pos1, pos2):
        if pos1[0] == pos2[0]:
            return abs(pos1[1] - pos2[1])
        elif pos1[1] == pos2[1]:
            return abs(pos1[0] - pos2[0])
        else:
            return abs(pos1[0] - pos2[0])*1.4
    '''
    改为位运算加快时间
    def Not_In_CloseList(self, node):
        for closenode in self.CloseList:
            if closenode.x == node.x and closenode.y == node.y:
                return 0
        return 1
    def Update_OpenList(self, node):
        for opennode in self.OpenList:
            if opennode.x == node.x and opennode.y == node.y:
                self.OpenList.remove(opennode)
        self.OpenList.append(node)
    '''
    #进入CloseList的方格会被置为State.In_CloseList
    def Not_In_CloseList(self, node):
        return self.map[node.x][node.y] ^ State.In_CloseList 
    
    def Update_OpenList(self, node):
        if not self.map[node.x][node.y] ^ State.In_OpenList:
            for opennode in self.OpenList:
                if opennode.x == node.x and opennode.y == node.y and opennode.f > node.f:
                    self.OpenList.remove(opennode)
        #self.OpenList.append(node)
        heapq.heappush(self.OpenList, (node.f, node))
        self.map[node.x][node.y] = State.In_OpenList
  
        
    def Jump(self, nodepos, Dir, depth, finalnodepos):
        
        nodepos = [nodepos[0] + Dir[0], nodepos[1] + Dir[1]]
        if depth == 0 or (nodepos[0] == finalnodepos[0] and nodepos[1] == finalnodepos[1]):
            return nodepos
        if not self.Is_Grid_Aviliable(nodepos[0], nodepos[1]):
            return None
        if Dir[0] != 0 and Dir[1] != 0:
            if (self.Is_Grid_Aviliable(nodepos[0] + Dir[0], nodepos[1] - Dir[1]) and (not self.Is_Grid_Aviliable(nodepos[0], nodepos[1] - Dir[1]))) or (self.Is_Grid_Aviliable(nodepos[0] - Dir[0], nodepos[1] + Dir[1]) and (not self.Is_Grid_Aviliable(nodepos[0] - Dir[0], nodepos[1]))):
                return nodepos
            if self.Jump([nodepos[0] + Dir[0], nodepos[1]], [Dir[0], 0], depth - 1, finalnodepos) != None:
                return nodepos
            if self.Jump([nodepos[0], nodepos[1] + Dir[1]], [0, Dir[1]], depth - 1, finalnodepos) != None:
                return nodepos
        #横向
        if Dir[1] != 0:
            if (self.Is_Grid_Aviliable(nodepos[0] + 1, nodepos[1] + Dir[1]) and (not self.Is_Grid_Aviliable(nodepos[0] + 1, nodepos[1]))) or (self.Is_Grid_Aviliable(nodepos[0] - 1, nodepos[1] + Dir[1]) and (not self.Is_Grid_Aviliable(nodepos[0] - 1, nodepos[1]))):
                return nodepos
        #纵向
        if Dir[0] != 0:          
            if (self.Is_Grid_Aviliable(nodepos[0] + Dir[0], nodepos[1] + 1) and (not self.Is_Grid_Aviliable(nodepos[0], nodepos[1] + 1))) or (self.Is_Grid_Aviliable(nodepos[0] + Dir[0], nodepos[1] - 1) and (not self.Is_Grid_Aviliable(nodepos[0], nodepos[1] - 1))):
                return nodepos
        #newnodepos = [nodepos[0] + Dir[0], nodepos[1] + Dir[1]]
        if self.Is_Grid_Aviliable(nodepos[0], nodepos[1]):
            return self.Jump(nodepos, Dir, depth - 1, finalnodepos)
        else:
            return None
        
        
    def Find_Neighber(self, node):
        neighbers = []
        #当前node不是起点
        if node.parent != None:
   
            nowdir = node.Get_Dir()
            #沿对角线拓展
            if nowdir[0] != 0 and nowdir[1] != 0:
                #当前方向是否可走
                if self.Is_Grid_Aviliable(node.x + nowdir[0], node.y + nowdir[1]):
                    neighbers.append([node.x + nowdir[0], node.y + nowdir[1], 1.4])
                #当前方向的水平分量是否可走
                if self.Is_Grid_Aviliable(node.x, node.y + nowdir[1]):
                    neighbers.append([node.x, node.y + nowdir[1], 1])
                #判断垂直分量是否可走
                if self.Is_Grid_Aviliable(node.x + nowdir[0], node.y):
                    neighbers.append([node.x + nowdir[0], node.y, 1])
                
                if (not self.Is_Grid_Aviliable(node.x - nowdir[0], node.y)) and self.Is_Grid_Aviliable(node.x - nowdir[0], node.y + nowdir[1]):
                    neighbers.append([node.x - nowdir[0], node.y + nowdir[1], 1.4])
                if (not self.Is_Grid_Aviliable(node.x, node.y - nowdir[1])) and self.Is_Grid_Aviliable(node.x + nowdir[0], node.y - nowdir[1]):
                    neighbers.append([node.x + nowdir[0], node.y - nowdir[1], 1.4])
            #沿水平方向直线拓展
            elif nowdir[0] == 0:
                #沿当前方向是否可走
                if self.Is_Grid_Aviliable(node.x + nowdir[0], node.y + nowdir[1]):
                    neighbers.append([node.x + nowdir[0], node.y + nowdir[1], 1])
                #左方不可走且左前方可走
                if (not self.Is_Grid_Aviliable(node.x - 1, node.y)) and self.Is_Grid_Aviliable(node.x - 1, node.y + nowdir[1]):
                    neighbers.append([node.x - 1, node.y + nowdir[1], 1.4])
                #右方不可走且右前方可走
                if (not self.Is_Grid_Aviliable(node.x + 1, node.y)) and self.Is_Grid_Aviliable(node.x + 1, node.y + nowdir[1]):
                    neighbers.append([node.x + 1, node.y + nowdir[1], 1.4])
            
            #沿竖直方向上拓展
            elif nowdir[1] == 0:
                #当前方向是否可走
                if self.Is_Grid_Aviliable(node.x + nowdir[0], node.y + nowdir[1]):
                    neighbers.append([node.x + nowdir[0], node.y + nowdir[1], 1])
                #左后方不可走且左方可走
                if (not self.Is_Grid_Aviliable(node.x, node.y - 1)) and self.Is_Grid_Aviliable(node.x + nowdir[0], node.y - 1):
                    neighbers.append([node.x + nowdir[0], node.y - 1, 1.4])
                #右后方不可走且右方可走
                if (not self.Is_Grid_Aviliable(node.x, node.y + 1)) and self.Is_Grid_Aviliable(node.x + nowdir[0], node.y + 1):
                    neighbers.append([node.x + nowdir[0], node.y + 1, 1.4])
        elif node.parent == None:
            for Dir in MovDir:
                if self.Is_Grid_Aviliable(node.x + Dir[0], node.y + Dir[1]):
                    neighbers.append([node.x + Dir[0], node.y + Dir[1], 1])
        
        return neighbers
    
    def Find_Path(self, startnode, finalnode):
        #self.OpenList.append(startnode)
        heapq.heappush(self.OpenList, (0, startnode))
        self.map[startnode.x][startnode.y] = State.In_OpenList
        current = startnode
        #count = 1
        while current.x != finalnode.x or current.y != finalnode.y:  
            '''
            测试用
            count = count + 1
            print("========")
            print(count)
            print(current.x)
            print(current.y)
            '''
            
            #self.OpenList.remove(current)
            #self.CloseList.append(current)
            
            neighbers = self.Find_Neighber(current)
            currentpos = [current.x, current.y]
            for neighber in neighbers:
                tempnode = Node(current, neighber[0], neighber[1], current.g + neighber[2])
                NowDir = tempnode.Get_Dir();
                newnodepos = self.Jump(currentpos, NowDir, 20, [finalnode.x, finalnode.y])
                if newnodepos:
                    newnode = Node(current, newnodepos[0], newnodepos[1], self.Get_g([current.x, current.y], newnodepos))
                    if self.Not_In_CloseList(newnode):
                        newnode.h = self.Get_h(newnode, finalnode)
                        newnode.f = newnode.h + newnode.g
                        self.Update_OpenList(newnode)
                '''
                newnode = Node(current, neighber[0], neighber[1], current.g + neighber[2])
                if self.Not_In_CloseList(newnode):
                    newnode.h = self.Get_h(newnode, finalnode)
                    newnode.f = newnode.h + newnode.g
                    self.Update_OpenList(newnode)
                '''
            
            #print(len(self.OpenList))
            #print(current.Get_Dir())
            
            '''
            改为最小堆实现减小时间
            mincost = self.OpenList[0].f
            minindex = 0
            for node in self.OpenList:
                if node.f < mincost:
                    mincost = node.f
                    minindex = self.OpenList.index(node)
            current = self.OpenList[minindex]
            '''
            #从OpenList中取出
            (_, current) = heapq.heappop(self.OpenList)
            self.map[current.x][current.y] = State.In_CloseList
        
        #回溯路径
        path = []
        node = current
        while node.parent != None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])    
        #path.reverse()
        return path       


# test_map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#             [0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
#             [0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0],
#             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
#             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]]
# JPS_handle = JPS_Handler(test_map)
# startnode = Node(None, 0, 0, 0)
# finalnode = Node(None, 0, 11, 9)
# #nodepos = JPS_handle.Jump([0, 0], [1, 1], 15, [4, 9])
# path = JPS_handle.Find_Path(startnode, finalnode)


