#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import random
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from utils import pos_to_gridcell
from utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from scipy import spatial
import time
from sensormodel import *
from motionmodel import *
import copy
from mapping import *

ocmap = OccupancyGrid()

init_map = False
init_odom = False

observations = []
odom = Odometry()

def listener():

    global ocmap
    global odom
    


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/map2/', OccupancyGrid, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(1) # 10hz

    # map_pose = Pose()
    # map_pose.position.x = 9.0
    # map_pose.position.y = -2.15
    # map_pose.position.z = 0.0
    # map_pose.orientation.x = 0.0
    # map_pose.orientation.y = 0.0
    # map_pose.orientation.z = 0.7071066498756409
    # map_pose.orientation.w = 0.70710688829422

    # #occu_map = ocmap
    # occu_map = OccupancyGrid();
    # occu_map.info.width = 860
    # occu_map.info.height = 1200
    # occu_map.info.resolution = 0.01
    # occu_map.info.origin = map_pose

    occu_map = init_MapMsg()


    #wait for map

    time.sleep(10)


    grid = np.zeros((occu_map.info.height*occu_map.info.width), dtype=np.int8)
    prob_grid = np.zeros((occu_map.info.height*occu_map.info.width), dtype=np.float32)
    grid.fill(50)
    prob_grid.fill(0.5)


    
    file2 = open('data', 'r')

    odom_list,point_list = read_record(file2)

    file2.close()


    for i in range(0,len(odom_list)):

        print("Processing record number: "+str(i))


        m_odom = odom_list[i]
        observations = point_list[i]

        prob_grid, grid = OccupancyGridMapping(prob_grid, grid, occu_map.info ,observations, m_odom)

        
        print("done")

        occu_map.data = grid
        mg_pub.publish(occu_map)



    while not rospy.is_shutdown():

        mg_pub.publish(occu_map)
        rate.sleep()


if __name__ == '__main__':
    listener()
