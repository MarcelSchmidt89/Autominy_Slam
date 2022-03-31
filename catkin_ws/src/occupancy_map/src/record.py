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

observations = []
odom = Odometry()
synched_odom = np.array([0.0,0.0,0.0])



def callback(data):
    global observations
    global odom
    global synched_odom

    synched_odom = get_odom_vector(odom)

    tmp_observations = []
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    rows = np.random.choice(points.shape[0] , 2000)
    points_m = points[rows, :]
  
    for point in points_m:
        if not ((0.41 < point[0] < 0.43) and (-0.12 < point[1] < 0.12)):
            tmp_observations.append(np.array([point[0],point[1]]))

    observations = tmp_observations
    


def odom_callback(data):
    global odom
    global init_odom
    
    odom = data
    

    
def listener():

    global odom
    global synched_odom
    global observations
    


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensors/slam/cloud", PointCloud2, callback)
    rospy.Subscriber("/sensors/odometry/odom", Odometry, odom_callback)



    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz

    time.sleep(1)


    #wait for map



    #m_odom = get_odom_vector(synched_odom)


    file = open('data', 'w')
    file.write(str(synched_odom)+";"+str(observations)+"\n")
    file.close()

    
    file2 = open('data', 'r')


    while not rospy.is_shutdown():


        #m_odom = get_odom_vector(synched_odom)

        file = open('data', 'a')
        file.write(str(synched_odom)+";"+str(observations)+"\n")
        file.close()



        # line = file2.readline()

        # #print(line)
        
        # line2 = line.split(';')

        # #print(line2)

        # #str_odom.replace('[','')

        # str_odom = line2[0]
        # str_odom = str_odom.replace('[','')
        # str_odom = str_odom.replace(']','')
        # #print(str_odom)
        # str_odom = str_odom.split()
        # print(str_odom)
        # np_odom = np.array(str_odom).astype(np.float)


        # str_points = line2[1]

        # #print(str_points)
        # str_points = str_points.replace('[','')
        # str_points = str_points.replace(']','')
        # str_points = str_points.split("), array(")
        # str_points[0] = str_points[0].replace('array(','')
        # str_points[len(str_points)-1] = str_points[len(str_points)-1].replace(')\n','')
        # print(str_points[1])
        # print("....")

        # rec_points = []

        # for pointstr in str_points:
        #     point_pair = pointstr.split(", ")
        #     rec_points.append(np.array(point_pair).astype(np.float))

        # print(rec_points[1])


        #print(np_odom[0])




        
        #print("done")

        rate.sleep()


if __name__ == '__main__':
    listener()
