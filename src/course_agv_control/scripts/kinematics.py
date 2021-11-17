#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import Float64
from math import *

class robot:
    # a1=0
    # a2=0
    # b1=0
    # b2=0
    # l1=0
    # l2=0
    # r=0
    def __init__(self):
        self.a1 = pi/2
        self.a2 = 3*pi/2
        self.b1 = 0
        self.b2 = pi
        self.l1 = 0.113332
        self.l2 = 0.113332
        self.r = 0.08
    pass

class Kinematics:
    left_wheel_vel = 0.0
    right_wheel_vel = 0.0
    def __init__(self):
        self.receiver = rospy.Subscriber(
            "/course_agv/velocity", geometry_msgs.msg.Twist,self.callback)
        self.vel_left_pub = rospy.Publisher(
            '/course_agv/left_wheel_velocity_controller/command',Float64, queue_size=10)
        self.vel_right_pub = rospy.Publisher(
            '/course_agv/right_wheel_velocity_controller/command',Float64, queue_size=10)

    def callback(self, data):
        vx = data.linear.x
        vw = data.angular.z
        self.left_wheel_vel,self.right_wheel_vel = self.kinematics(vx,vw)

    def kinematics(self,vx,vw):
        # too simple method , TODO
        r = robot()

        # cmd_np = np.array([vx,0,vw])
        # transform_matrix = np.array([
        #     [sin(r.a1+r.b1),-1*cos(r.a1+r.b1),-r.l1*cos(r.b1)],
        #     [sin(r.a2+r.b2),-1*cos(r.a2+r.b2),-r.l2*cos(r.b2)],
        # ])
        left = sin(r.a1+r.b1)*vx-r.l1*cos(r.b1)*vw
        right = sin(r.a2+r.b2)*vx-r.l2*cos(r.b2)*vw
        # print("left = ",left/r.r,"right = " ,right/r.r)
        return left/r.r,right/r.r
        #return 1,1

    def publish(self):
        self.vel_left_pub.publish(self.left_wheel_vel)
        self.vel_right_pub.publish(self.right_wheel_vel)

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    try:
        
        rospy.init_node(node_name)
        k = Kinematics()
        rate = rospy.Rate(rospy.get_param('~publish_rate',200))
        while not rospy.is_shutdown():
            k.publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
