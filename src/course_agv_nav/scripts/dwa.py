#u = self.dwa.plan(self.plan_x, self.plan_goal, self.plan_ob)
from math import *
import numpy as np

class DWAPlanner():
    def __init__(self):
        #start and end
        self.sx = 0
        self.sy = 0
        self.stheta = 0
        self.sv =0
        self.sw =0
        self.gx =0
        self.gy =0
        #parameter of dwa
        self.vmin = -0.2
        self.vmax = 0.2
        self.wmin = -3
        self.wmax = 3
        self.vstep = 0.01
        self.wstep = 0.1
        self.vdotup = 0.5
        self.vdotdown = 1
        self.wdot = 0.2
        self.dt = 0.1
        #G(v,w) = a*heanding + b.dist + c*velocity
        self.a = 0.25
        self.b = 0.5
        self.c = 0.25
        pass
    
    def plan(self,plan_x,plan_goal,plan_ob):
        self.sx,self.sy,self.stheta,self.sv,self.sw = plan_x[0],plan_x[1],plan_x[2],plan_x[3],plan_x[4]
        self.gx,self.gy = plan_goal[0],plan_goal[1]
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        #self.plan_x = [self.x,self.y,self.yaw, self.vx, self.vw]
        #TODO,what is obstcle,waht is map
        vsteps = int((self.vmax-self.vmin)/self.vstep)
        wsteps = int((self.wmax-self.wmin)/self.wstep)
        u = []
        for i in range(0,vsteps):
            vtemp = self.vmin+i*self.vstep
            for j in range(0,wsteps):
                wtemp = self.wmin+j*self.wstep
                #TODO
                if(check_velocity(vtemp,wtemp)):
                    pass
                pass
        return u
    
    def G(self,u,plan_ob):
        #u is a list,u = [v,w]
        #get the best [v,w]
        heading,dist,velocity = [],[],[]
        h_sum,d_sum,v_sum = 0,0,0
        for i in range(len(u)):
            heading.append(self.heading(u[0],u[1]))
            dist.append(self.dist(u[0],u[1],plan_ob))
            velocity.append(self.velocity(u[0],u[1]))
            h_sum =h_sum+heading[i]
            d_sum =d_sum+dist[i]
            v_sum=v_sum+velocity[i]
        pass
        G = []
        max_index = 0
        for i in range(len(u)):
            temp = heading[i]/h_sum + dist[i]/d_sum + velocity[i]/v_sum
            G.append(temp)
            if(temp>G(max_index)):
                max_index = i
        return u[max_index]
            
        
    def heading(self,v,w):
        theta2 = self.stheta+w*self.dt
        x2 = self.sx-v/w*sin(self.stheta)+v/w*sin(theta2)
        y2 = self.sx+v/w*cos(self.stheta)-v/w*cos(theta2)
        thetag = atan2(self.gx-x2,self.gy-y2)
        delta_theta = abs(theta2-thetag)
        #score high:close to the oritention of goal
        return pi-delta_theta
    def dist(self,v,w,obstree):
        #choose the minimum distance the obscal?
        #TODO
        return 0
        pass
    def velocity(self,v,w):
        #velocity need to be continuous
        # TODO change weight
        vdiff = v-self.sv
        wdiff = w-self.sw
        return vdiff+wdiff
        pass
    def real_to_map(self,x,y):
        v = int((x+10)/0.155+0.5)
        u = int((y+10)/0.155+0.5)
        return u,v

    def map_to_real(self,u,v):
        y=u*0.155-10
        x=v*0.155-10
        return x,y
    pass