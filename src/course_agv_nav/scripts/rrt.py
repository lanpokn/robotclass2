import os
import math
import sys
import random
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import numpy as np



path_step = 800



class Node():
    def __init__(self,x,y,length = path_step,parent = None) :
        self.x = x
        self.y = y
        self.length = length
        self.parent = parent

        pass

    def return_path_to_root(self):
        #once near the end,back the path to theroot
        temp = Node(self.x,self.y,parent = self.parent)
        #numpy
        path_x,path_y = [],[]
        while(temp.parent!=None):
            path_x.append(temp.x)
            path_y.append(temp.y)
            temp = temp.parent
        path_x.append(temp.x)
        path_y.append(temp.y)

        path_x = list(reversed(path_x))
        path_y = list(reversed(path_y))
        return path_x,path_y
    
pass


class RRT():
    #try to use rrt*,*can't be a name of class
    def __init__(self,):

        self.minx = -4500
        self.maxx = 4500
        self.maxy = 3000
        self.miny = -3000
        self.node_list = []
        self.robot_size = 200

        self.avoid_dist = 300
        self.stop_distance = 900
        self.path_step = path_step
        self.maxN = 1000
        pass

    def plan(self,vision,start_x,start_y,goal_x,goal_y):
        obstacle_x,obstacle_y = [-999999],[-999999]
        road_map = []

        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id>0:
                obstacle_x.append(robot_blue.x)
                obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)
        pass

        obstree = KDTree(np.vstack((obstacle_x,obstacle_y)).T)


        #generate tree

        node_final,sample_x,sample_y= self.generate_tree(start_x,start_y,goal_x,goal_y,obstree)
        path_x,path_y = node_final.return_path_to_root()

        path_x_new,path_y_new = self.delete_node(path_x,path_y,obstree)
        return path_x,path_y,road_map,sample_x,sample_y

    def delete_node(self,path_x,path_y,obstree):
        i = 0
        while(i<len(path_x)-2):
            node_temp_i = Node(path_x[i],path_y[i])
            for j in range(i+2,len(path_x)):
                node_temp_j = Node(path_x[j],path_y[j],parent=node_temp_i)
                if(j==len(path_x)-1):
                        i+=1
                if(self.check_obs(node_temp_j,obstree)):
                    continue
                else:
                    for k in range(i+1,j):
                        path_x.remove(path_x[k])
                        path_y.remove(path_y[k])
                    break
        
        return path_x,path_y
                



    def generate_tree(self,start_x,start_y,goal_x,goal_y,obstree):

        node_final = Node(-999999,-999999)
        sample = []
        sample_x,sample_y = [],[]

        sample.append(Node(start_x,start_y))
        sample_x.append(start_x)
        sample_y.append(start_y)
        sampletree = KDTree(np.vstack((sample_x,sample_y)).T)


        i = 0
        while(self.exit_status(node_final,goal_x,goal_y) and i<self.maxN):
            i = i+1
            node_temp  =self.add_one_node(obstree) 
            if(node_temp!=None):
                node_final_temp = self.search_qnew(node_temp,sample,sampletree,obstree)
                if(node_final_temp!=None):
                    node_final = node_final_temp

                    sample_x.append(node_final.x)
                    sample_y.append(node_final.y)
                    sample.append(node_final)
                    sampletree = KDTree(np.vstack((sample_x,sample_y)).T)
                    pass
            pass
    
        return node_final,sample_x,sample_y

    def exit_status(self,node_final,goal_x,goal_y):

        if( (abs(node_final.x-goal_x) < self.stop_distance) and 
            (abs(node_final.y-goal_y) < self.stop_distance)):
            return False
        else:
            return True

    def add_one_node(self,obstree):

        tx = random.random()*(self.maxx-self.minx)+self.minx
        ty = random.random()*(self.maxy-self.miny)+self.miny
        distance, index = obstree.query(np.array([tx,ty]))

        if distance >= self.robot_size+self.avoid_dist:
            return Node(tx,ty)
        else:
            return None
    
    #
    def check_obs(self,node,obstree):

        #judge whether the node and its line to the parent can be add

        x = node.parent.x
        y = node.parent.y
        dx = node.x - x
        dy = node.y - y
        angle = math.atan2(dy,dx)
        dis = math.hypot(dx,dy)
        step_size = self.robot_size+self.avoid_dist
        steps = max(round(dis/step_size),20)
        for i in range(steps):
            distance,index = obstree.query(np.array([x,y]))
            if distance <= step_size:
                return True

            x+= dis/steps*math.cos(angle)
            y+= dis/steps*math.sin(angle)
            
        pass

    def search_qnew(self,node,sample,sampletree,obstree):

        #find qnear, this may cost a lot time ,so try to optimize

        distance, index = sampletree.query(np.array([node.x,node.y]))
        node_temp = node
        node_near = sample[index]

        theta = math.atan2(node_temp.y - node_near.y,node_temp.x - node_near.x)

        delta_x = math.cos(theta)*self.path_step
        delta_y = math.sin(theta)*self.path_step
        node_final = Node(node_near.x+delta_x, node_near.y+delta_y,length=self.path_step,parent=node_near)


        if(abs(node_final.x)>self.maxx or abs(node_final.y)>self.maxy):
            return None
        else:
            if(self.check_obs(node_final,obstree)):
                return None
            else:
                return node_final

    pass

class test():
    def __init__(self):
        # self.plan_sx = 0
        # self.plan_sy =0
        # self.plan_gx =0
        # self.plan_gy=0
        self.length = 20
        pass
         
    def planning(self,plan_sx,plan_sy,plan_gx,plan_gy):
        path_x,path_y = [],[]
        xdiff = plan_gx-plan_sx
        ydiff = plan_gy-plan_sy
        
        for i in range(1,self.length+1):
            path_x.append(plan_sx+i/self.length*xdiff)
            path_y.append(plan_sy+i/self.length*ydiff)
        path_x = list(reversed(path_x))
        path_y = list(reversed(path_y))
        print(path_x)
        #goal shoule be in front
        return path_x,path_y
            
            