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
initial_odom = Odometry()

def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

def map_callback(data):
    global ocmap
    global init

    ocmap = data

    #print(ocmap.info.height*ocmap.info.width)

    init_map = True



def callback(data):
    global observations
    global odom
    observations = []
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    rows = np.random.choice(points.shape[0] , 50)
    points_m = points[rows, :]

    orientation = odom.pose.pose.orientation
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
    pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) 
    angle = yaw

    for point in points_m:
        observations.append(point)


def odom_callback(data):
    global odom
    global init_odom
    odom = data
    init_odom = True

def init_odom_callback(data):
    global initial_odom
    initial_odom = data
    init_odom = True


    
def listener():

    global ocmap
    


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/map2/', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/sensors/slam/likelyhoodmap/", OccupancyGrid, map_callback)
    rospy.Subscriber("/sensors/slam/cloud", PointCloud2, callback)
    rospy.Subscriber("/sensors/odometry/odom", Odometry, odom_callback)
    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, init_odom_callback)



    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz


    #wait for map

    time.sleep(1)


    map_angle = get_map_orientation(ocmap.info)

    likelyhood_grid = ocmap.data

    grid = np.zeros((ocmap.info.height*ocmap.info.width), dtype=np.int8)
    grid.fill(100)

    likelymap_msg = ocmap


    particle_odoms = []

    num_particles = 100

    odom_old = get_odom_vector(odom)

    init_odom =get_odom_vector(initial_odom)

    print(init_odom)



    for i in range(0,num_particles):
        p_pos = np.array([6.0*np.random.random_sample(),4.3*np.random.random_sample()])
        p_angle = np.random.random_sample() * (2 * np.pi)

        p_odom = np.array([p_pos[0],p_pos[1],p_angle])
        #particle_odoms.append(p_odom)
        particle_odoms.append(init_odom)


    particle_weights = np.zeros(num_particles)


   #particle_odom = get_odom_vector(odom)




    #particle_odoms[0] = particle_odom
    


    while not rospy.is_shutdown():



        observations_copy = np.copy(observations)    

        last_prob = 0
        grid.fill(100)


        odom_new = get_odom_vector(odom)

        if np.array_equal(odom_new, odom_old):
            print("whoops!")

        for i in range(0,num_particles):

            particle_odoms[i] = sample_motion_model(get_motion_delta(odom_old,odom_new),particle_odoms[i])
            particle_odom = particle_odoms[i]

            particle_point = np.array([particle_odom[0],particle_odom[1]])

        #transform point from world to map
        

            point = np.rint(transform_point(np.array(particle_point*100),-map_angle,np.array([0.0,0.0])))

            gridtarget = int((point[1]+ocmap.info.height*0.75)*(ocmap.info.width))+int(point[0]+ocmap.info.width/4)
            grid[gridtarget] = 0

            prob = end_point_model(particle_odom, observations_copy, likelyhood_grid, map_angle, ocmap.info.height,ocmap.info.width)

            particle_weights[i] = prob

        
        odom_old = odom_new

        weight_sum = sum(particle_weights)

        if weight_sum > 0:

            particle_weights = particle_weights / weight_sum

            n_eff = 1 / np.sum(np.square(particle_weights))

            if n_eff < (num_particles/4):

                rng = np.random.default_rng()      

                particle_odoms_indexes = np.random.choice(len(particle_odoms),num_particles,True,particle_weights)

                particle_odoms = np.take(particle_odoms,particle_odoms_indexes,0)






        likelymap_msg.data = grid
        mg_pub.publish(likelymap_msg)
        rate.sleep()


if __name__ == '__main__':
    listener()
