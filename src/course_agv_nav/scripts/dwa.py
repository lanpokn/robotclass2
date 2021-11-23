#u = self.dwa.plan(self.plan_x, self.plan_goal, self.plan_ob)

class DWAPlanner():
    def __init__(self):
        #parameter of dwa
        self.vmin = -0.2
        self.vmax = 0.2
        self.wmin = -0.5
        self.wmax = 0.5
        self.vdot = 0.2
        self.wdot = 0.2
        #G(v,w) = a*heanding + b.dist + c*velocity
        self.a = 0.33
        self.b = 0.33
        self.c = 0.33
        pass
    
    def plan(plan_x,plan_goal,plan_ob):
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        #self.plan_x = [self.x,self.y,self.yaw, self.vx, self.vw]
        u = [0,0]
        return u
    
    def heading(v,w,theta1):
        pass
    def dist(v,w):
        pass
    def velocity(v,w):
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