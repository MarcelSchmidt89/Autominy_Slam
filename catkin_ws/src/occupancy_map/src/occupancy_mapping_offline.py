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

ocmap = OccupancyGrid()

init_map = False
init_odom = False

observations = []
odom = Odometry()

def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

def map_callback(data):
    global ocmap
    global init

    ocmap = data

    #print(ocmap.info.height*ocmap.info.width)

    init_map = True


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

    #ospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback)
    rospy.Subscriber("/sensors/slam/likelyhoodmap/", OccupancyGrid, map_callback)



    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(1) # 10hz


    #wait for map

    time.sleep(10)


    map_angle = get_map_orientation(ocmap)

    likelyhood_grid = ocmap.data

    grid = np.zeros((ocmap.info.height*ocmap.info.width), dtype=np.int8)
    prob_grid = np.zeros((ocmap.info.height*ocmap.info.width), dtype=np.float32)
    grid.fill(50)

    prob_grid.fill(0.5)

    likelymap_msg = ocmap

    gausscale = gaussian(0.0,0.0,0.01)


    prior = 0.5

    file2 = open('data', 'r')

    odom_list,point_list = read_record(file2)

    print(len(odom_list))
    print("...")
    print(len(point_list))

    file2.close()


    for i in range(0,len(odom_list)):

        #print(observations[0])
        

        #grid.fill(100)

        print("Processing record number: "+str(i))


        m_odom = odom_list[i]
        observations = point_list[i]

        position = np.array([m_odom[0],m_odom[1]])


        obs=[]

        obs.append([-1000,-1000])

        for ob in observations:
            obs.append(transform_point(ob,m_odom[2],position))


        kdtree = spatial.cKDTree(obs) 

        pos_int_x = int(np.rint(position[0]*100))
        pos_int_y = int(np.rint(position[1]*100))
        


        for x in range(pos_int_x-100, pos_int_x+100):
            for y in range(pos_int_y-100, pos_int_y+100):

                point = np.array([x,y])                

                car_point = transform_point(point/100, -0, -position)

                car_point = transform_point(car_point, -m_odom[2], np.array([0,0]))

                if(0.2 < np.linalg.norm(car_point) < 0.8) and (angle_between_vectors(car_point,np.array([1,0])) < 0.69) :

                #if dist < 0.5:

                    dist, index = kdtree.query(np.array([x,y])/100)   

                    point = np.rint(transform_point(point,-map_angle,np.array([0.0,0.0])))
                    gridtarget = int((point[1]+ocmap.info.height*0.75)*(ocmap.info.width))+int(point[0]+ocmap.info.width/4)

                    new_prob = np.clip(((gaussian(dist,0.0,0.01))/gausscale),0.4,0.6) 
                    #new_prob = (gaussian(dist,0.0,0.02))/gausscale


                    old_prob = prob_grid[gridtarget] 


                    new_odds = (new_prob/(1-new_prob)) * (old_prob/(1-old_prob)) * ((1-prior)/prior)

                    new_prob = 1 / (1+  ((1-new_prob)/new_prob) * ((1-old_prob)/old_prob) * (prior/(1-prior)) )

                    prob_grid[gridtarget] = new_prob

                    grid[gridtarget] = int(new_prob*100)




        
        print("done")

        likelymap_msg.data = grid
        mg_pub.publish(likelymap_msg)



    while not rospy.is_shutdown():

        mg_pub.publish(likelymap_msg)
        rate.sleep()


if __name__ == '__main__':
    listener()
