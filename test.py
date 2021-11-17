import rospy
import tf
from nav_msgs.srv import GetMap
import numpy as np

if __name__ == "__main__":
    # getMap = rospy.ServiceProxy('/static_map',GetMap)
    # msg = getMap().map
    # data = np.array(msg.data)
    # data.resize(129,129)
    # np.savetxt(fname='/home/lanpokn/Documents/2021/robot2/course2021-Robotics2-master/map.txt',X = data)
    # print(data[:,4])
    data = np.