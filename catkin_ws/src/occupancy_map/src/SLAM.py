#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import random
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
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

from threading import Thread


particles_amount = 30

occupancy_map = init_MapMsg()
likelyhood_map = init_MapMsg()

particle_odoms = []
particle_weights = []

occupancy_grids = []
likelyhood_grids = []


old_odom = np.array([0,0,0])
new_odom = np.array([0,0,0])
observations = []

odom_records = []
observation_records = []



def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

def thread_handler(thread_id):

    for j in range(0,particles_per_thread):
        Slam((particles_per_thread*thread_id)+j)



def Slam(particle_index):

    global occupancy_map
    global likelyhood_map

    global particle_odoms
    global particle_weights

    global occupancy_grids
    global likelyhood_grids
    
    global observations
    global new_odom
    global old_odom

    print("starte particle " + str(particle_index))


    particle_odoms[particle_index] = sample_motion_model(get_motion_delta(old_odom,new_odom),particle_odoms[particle_index])
    particle_odom = particle_odoms[particle_index]

    particle_pose = np.array([particle_odom[0],particle_odom[1]])

    #transform point from world to grid_index            

    #grid_point = np.rint(transform_point(np.array(particle_pose*100),-map_angle,np.array([0.0,0.0])))

    likelyhood_grids[particle_index] = update_likelyhoodmap(occupancy_grids[particle_index], likelyhood_grids[particle_index] ,likelyhood_map.info, particle_odom)

    #likelyhood_map.data = np.rint(likelyhood_grids[0] * 100).astype(dtype=np.int8)
    #map2_pub.publish(likelyhood_map)


    map_angle = get_map_orientation(occupancy_map.info)

    prob = end_point_model(particle_odom, observations, likelyhood_grids[particle_index], map_angle, likelyhood_map.info.height,likelyhood_map.info.width)

    particle_weights[particle_index] = prob


    occupancy_grids[particle_index], grid = OccupancyGridMapping(occupancy_grids[particle_index], [], occupancy_map.info ,observations, particle_odom)


    return





def listener():

    global occupancy_map
    global particle_odoms
    global particle_weights
    global occupancy_grids
    global likelyhood_grids
    global observations
    global new_odom
    global old_odom 
    


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    map_pub = rospy.Publisher('/sensors/slam_map/', OccupancyGrid, queue_size=10)
    map2_pub = rospy.Publisher('/sensors/slam_likelyhood/', OccupancyGrid, queue_size=10)
    cloud_pub = rospy.Publisher('sensors/slam_particles', PointCloud2, queue_size=10)
    odom_pub = rospy.Publisher('sensors/slam_odom',Odometry, queue_size=10)



    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz

    
    #init ROS Messages


    pc_msg = PointCloud2()
    pc_msg.header.frame_id = 'map'

    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"




    #wait for map



    #read records

    file2 = open('data', 'r')

    odom_records,observation_records = read_record(file2)

    file2.close()


    #initialize data

    new_odom = odom_records[0]
    old_odom = odom_records[0]

    particle_odoms = [None]*particles_amount
    particle_weights = [None]*particles_amount

    occupancy_grids = [None]*particles_amount
    likelyhood_grids = [None]*particles_amount


    for i in range(0,particles_amount):

        likelyhood_grids[i] = np.full((likelyhood_map.info.height*likelyhood_map.info.width),0.5, dtype=np.float32)
        occupancy_grids[i] = np.full((occupancy_map.info.height*occupancy_map.info.width),0.5, dtype=np.float32)
        
        
        particle_weights[i] = 0
        particle_odoms[i] = odom_records[0]

    likelyhood_map.data = np.rint(likelyhood_grids[0] * 100).astype(dtype=np.int8)
    occupancy_map.data = np.rint(occupancy_grids[0] * 100).astype(dtype=np.int8)


    map_pub.publish(occupancy_map)
    map2_pub.publish(likelyhood_map)

    for i in range(0,particles_amount):

        particle_odom = odom_records[0]

        occupancy_grids[i], grid = OccupancyGridMapping(occupancy_grids[i], [], occupancy_map.info ,observation_records[0], particle_odom)
        occupancy_grids[i], grid = OccupancyGridMapping(occupancy_grids[i], [], occupancy_map.info ,observation_records[0], particle_odom)
        occupancy_grids[i], grid = OccupancyGridMapping(occupancy_grids[i], [], occupancy_map.info ,observation_records[0], particle_odom)
        occupancy_grids[i], grid = OccupancyGridMapping(occupancy_grids[i], [], occupancy_map.info ,observation_records[0], particle_odom)
        occupancy_grids[i], grid = OccupancyGridMapping(occupancy_grids[i], [], occupancy_map.info ,observation_records[0], particle_odom)

    occupancy_map.data = np.rint(occupancy_grids[0] * 100).astype(dtype=np.int8)
    map_pub.publish(occupancy_map)



    for i in range(0,len(odom_records)):

        print("Processing record number: "+str(i))
        new_odom = odom_records[i]
        observations = observation_records[i]

        threads = []

        for j in range(0,particles_amount):
            Slam(j)

        old_odom = new_odom

        # for j in range(0,thread_amount):   

        #     #Slam(j)
        #     threads.append(Thread(target=thread_handler, args=[j]))

        # for thread in threads:
        #     thread.start()

        # for thread in threads:
        #     thread.join()

        print(particle_weights)

        weight_sum = sum(particle_weights)

        if weight_sum > 0:

            particle_weights = particle_weights / weight_sum

            n_eff = 1 / np.sum(np.square(particle_weights))

            if n_eff < (particles_amount/2):

                rng = np.random.default_rng()      

                particle_odoms_indexes = np.random.choice(len(particle_odoms),particles_amount,True,particle_weights)

                particle_odoms = np.take(particle_odoms,particle_odoms_indexes,0)                    
                particle_weights = np.take(particle_weights,particle_odoms_indexes,0)
                occupancy_grids = np.take(occupancy_grids,particle_odoms_indexes,0)
                likelyhood_grids = np.take(likelyhood_grids,particle_odoms_indexes,0)


        print(particle_weights)

        particles = []

        for particle in particle_odoms:
            particles.append(np.array([particle[0],particle[1],0.0]))


        pc_msg = create_cloud_xyz32(pc_msg.header, particles)

        index = np.argmax(particle_weights)
        best_odom = particle_odoms[index]

        odom_quat = tf.transformations.quaternion_from_euler(0,0,best_odom[2])
        odom_msg.pose.pose = Pose(Point(best_odom[0],best_odom[1],0),Quaternion(*odom_quat))


        likelyhood_map.data = np.rint(likelyhood_grids[index] * 100).astype(dtype=np.int8)
        occupancy_map.data = np.rint(occupancy_grids[index] * 100).astype(dtype=np.int8)


        map_pub.publish(occupancy_map)
        map2_pub.publish(likelyhood_map)
        cloud_pub.publish(pc_msg)
        odom_pub.publish(odom_msg)


        





    while not rospy.is_shutdown():



        rate.sleep()


if __name__ == '__main__':
    listener()
