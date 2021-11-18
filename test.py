import rospy
import tf
from nav_msgs.srv import GetMap
import numpy as np
#resolution: 0.155
if __name__ == "__main__":
    # getMap = rospy.ServiceProxy('/static_map',GetMap)
    # msg = getMap().map
    # data = np.array(msg.data)
    # data.resize(129,129)
    # np.save('/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map',data)
    # print(data[:,4])
    
    
    # data1 = np.load("/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map.npy")
    # data2 = np.zeros(shape = (129,129))
    # avoid_dist = 2
    # for i in range(0,129):
    #     for j in range(0,129):
    #             temp = data1[i][j]
    #             flag = 0#no obscal
    #             for k in range(i-avoid_dist,i+avoid_dist):
    #                 for l in range(j-avoid_dist,j+avoid_dist):
    #                     if(k<0 or k>=129 or l<0 or l>=129):
    #                         continue
    #                     else:
    #                         if(data1[k][l]== 100):
    #                             flag  =1#have obscal
                
    #             if(flag == 0):
    #                 data2[i][j] = -1
    #             else:
    #                 data2[i][j] = 100
    # np.save('/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map_ob',data2) 
    # print(data2[:,65])
    data1 = np.array([[1,2],[3,4]])
    data2 = data1.copy()
    data1[0,0] = 0
    print(data2)
    