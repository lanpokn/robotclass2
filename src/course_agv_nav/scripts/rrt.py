import os
import math
from pickle import FALSE
import sys
import random
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import numpy as np


# y and v are just inverse!,in data
path_step = 7/6.3
data_high_secure = np.ones([129,129])
data_low_secure = np.ones([129,129])
try:
    # with np.load("/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map_ob.npy") as data2:
    #     data = data2.copy()
    data_high_secure = np.load("/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map_ob.npy") 
    data_low_secure = np.load("/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map.npy")
    data = data_high_secure
except:
    print("load error!\n\n\n\n\n\n\n\n\n\n\n")
np.savetxt("/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map_ob.txt",data)
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
    #no need for KDtree
    def __init__(self,):

        self.minx = -64/6.3
        self.maxx = 64/6.3
        self.maxy = 64/6.3
        self.miny = -64/6.3
        self.node_list = []
        #no use
        self.robot_size = 0.1
        self.avoid_dist = 0.2
        
        #no use too, try join the goal whenver possible
        #self.stop_distance = 900
        self.path_step = path_step
        self.maxN = 2000
        pass

    def planning(self,start_x,start_y,goal_x,goal_y,plan_ox,plan_oy,vision = None):

        obstree = KDTree(np.vstack((plan_ox,plan_oy)).T)


        #generate tree
        # data = data_high_secure
        # print(data)
        # start_u,start_v = self.real_to_map(start_x,start_y)
        # goal_u,goal_v = self.real_to_map(goal_x,goal_y)
        
        # if(data[goal_u,goal_v] == 100):
        #     data = data_low_secure
        #     if(data[goal_u,goal_v] == 100):
        #         print("can't reach!!")
        #         return [start_x],[start_y]
            
        node_final,sample_u,sample_v= self.generate_tree(start_x,start_y,goal_x,goal_y,obstree=obstree)
        path_u,path_v = node_final.return_path_to_root()

        path_u_new,path_v_new = self.delete_node(path_u,path_v,obstree)
        
        #convert back
        path_x,path_y = [],[]
        for i in range(0,len(path_u_new)):
            #temp_x,temp_y = self.map_to_real(path_u_new[i],path_v_new[i])
            temp_x,temp_y = path_u_new[i],path_v_new[i]
            path_x.append(temp_x)
            path_y.append(temp_y)
        path_x = list(reversed(path_x))
        path_y = list(reversed(path_y))
        # path_x.pop()
        # path_y.pop()
        return path_x,path_y

    def delete_node(self,path_x,path_y,obstree = None):
        i = 0
        flag = 0
        while(True):
            flag = 0
            print(len(path_x))
            if(i>len(path_x)-3):
                break
            node_temp_i = Node(path_x[i],path_y[i])
            print("i=",i)
            while(True):
                if(flag==1):
                    break
                for j in range(i+2,len(path_x)):
                    print("j=",j)
                    if(j>len(path_x)-2):
                        flag = 1
                    node_temp_j = Node(path_x[j],path_y[j],parent=node_temp_i)
                    # if(j==len(path_x)-1):
                    #         i+=1
                    if(self.check_obs(node_temp_j,obstree) == True):
                        continue
                    else:
                        for k in range(i+1,j):
                            del path_x[i+1]
                            del path_y[i+1]
                        break
            i = i+1
        
        return path_x,path_y
                



    def generate_tree(self,start_x,start_y,goal_x,goal_y,obstree = None):
        #no start point, goal in first
        node_final = Node(start_x,start_y)
        sample = []
        sample_x,sample_y = [],[]

        sample.append(Node(start_x,start_y))
        sample_x.append(start_x)
        sample_y.append(start_y)
        sampletree = KDTree(np.vstack((sample_x,sample_y)).T)


        i = 0
        while(self.exit_status(node_final,goal_x,goal_y,obstree)==False and i<self.maxN):
            i = i+1
            node_temp  =self.add_one_node(obstree=obstree) 
            if(node_temp!=None):
                node_final_temp = self.search_qnew(node_temp,sample,sampletree,obstree=obstree)
                if(node_final_temp!=None):
                    node_final = node_final_temp

                    sample_x.append(node_final.x)
                    sample_y.append(node_final.y)
                    sample.append(node_final)
                    sampletree = KDTree(np.vstack((sample_x,sample_y)).T)
                    pass
            pass
        #add goal to the sample
        node_final_temp = Node(goal_x,goal_y,parent= node_final)
        node_final = node_final_temp
        sample_x.append(node_final.x)
        sample_y.append(node_final.y)
        sample.append(node_final)
        # sampletree = KDTree(np.vstack((sample_x,sample_y)).T)
    
        return node_final,sample_x,sample_y

    def exit_status(self,node_final,goal_x,goal_y,obstree):
        # need to rewrite completely
        
        # if( (abs(node_final.x-goal_x) < self.stop_distance) and 
        #     (abs(node_final.y-goal_y) < self.stop_distance)):
        #     return False
        # else:
        #     return True
        temp  = Node(goal_x,goal_y,parent=node_final)
        if(self.check_obs(temp,obstree=obstree) == False):
            return True
        else:
            return False
        pass

    def add_one_node(self,obstree = None):

        tx = random.random()*(self.maxx-self.minx)+self.minx
        ty = random.random()*(self.maxy-self.miny)+self.miny
        # distance, index = obstree.query(np.array([tx,ty]))

        #-1 is secure
        #TODO
        #tx,ty = int(tx+0.5),int(ty+0.5)
        if data[tx,ty] == -1:
            return Node(tx,ty)
        else:
            return None
    
    #
    def check_obs(self,node,obstree=None):

        #judge whether the node and its line to the parent can be add

        x = node.parent.x
        y = node.parent.y
        dx = node.x - x
        dy = node.y - y
        angle = math.atan2(dy,dx)
        dis = math.hypot(dx,dy)
        # in map system ,this should be 0.5
        step_size = 0.5
        steps = max([int(dis/step_size+0.5),100])
        for i in range(steps):
            #TODO
            if (data[int(x+0.5),int(y+0.5)] == 100):
                return True

            x+= dis/steps*math.cos(angle)
            y+= dis/steps*math.sin(angle)
        
        return False
        pass

    def search_qnew(self,node,sample,sampletree,obstree = None):

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

    #added for robot2
    def real_to_map(self,x,y):
        v = int((x+10)/0.155+0.5)
        u = int((y+10)/0.155+0.5)
        return u,v

    def map_to_real(self,u,v):
        y=u*0.155-10
        x=v*0.155-10
        return x,y
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
        #need to convert to map matrix
        path_x,path_y = [],[]
        xdiff = plan_gx-plan_sx
        ydiff = plan_gy-plan_sy
        
        for i in range(1,self.length+1):
            path_x.append(plan_sx+i/self.length*xdiff)
            path_y.append(plan_sy+i/self.length*ydiff)
        path_x = list(reversed(path_x))
        path_y = list(reversed(path_y))
        print(plan_gx,plan_gy)
        #goal shoule be in front
        print(data)
        return path_x,path_y
            
            