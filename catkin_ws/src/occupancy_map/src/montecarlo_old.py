#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from utils import pos_to_gridcell
from utils import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from scipy import spatial

ocmap = OccupancyGrid()

init_map = False
init_odom = False

num_particles = 100

observations = []
odom = Odometry()

def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

def map_callback(data):
    global ocmap
    global init
    global odom

    ocmap = data
    init_map = True

def callback(data):
    global observations
    global odom
    observations = []
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    rows = np.random.choice(points.shape[0] , 200)
    points_m = points[rows, :]

    orientation = odom.pose.pose.orientation
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
    pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) 
    angle = yaw

    for point in points_m:
        #print(np.rint(point*100))
        #print("Take point: "+str(point)+" with "+str(angle)+" "+str(pos)+" result: "+str(transform_point(point,-angle,-pos)))
        #observations.append(transform_point(point,-1*angle,-1*pos))
        observations.append(point)


def odom_callback(data):
    global odom
    global init_odom
    odom = data
    init_odom = True


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/map2/', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/sensors/map/", OccupancyGrid, map_callback)
    rospy.Subscriber("/sensors/slam/cloud", PointCloud2, callback)
    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz


    #wait for map

    print("Waiting for map....")

    while not init_map and not init_odom:
        print("waiting...")
        rate.sleep()

    print(ocmap.info.width)


    print("Init likelyhood map")

    points = []

    for x in range(0,ocmap.info.width):
        for y in range(0,ocmap.info.height):
            if ocmap.data[y*ocmap.info.width+x] <= 5:
                #print(str(x)+","+str(y))
                points.append([x,y])


    kdtree = spatial.cKDTree(points)

    grid = np.zeros((ocmap.info.height*ocmap.info.width), dtype=np.int8)
    grid.fill(100)
    likelyhood_grid = np.zeros((ocmap.info.height*ocmap.info.width), dtype=np.int8)
    likelyhood_map = np.zeros(shape=(ocmap.info.width,ocmap.info.height))

    gausscale = gaussian(0.0,0.0,5.0)

    for x in range(0,ocmap.info.width):
        for y in range(0,ocmap.info.height):
            dist, index = kdtree.query(np.array([x,y]))            
            #print(dist)
            likelyhood = ((gaussian(dist,0.0,5.0)/gausscale)) 
            likelyhood_map[x][y] = likelyhood
            #print(str(dist)+" : "+str(occ_value))
            likelyhood_grid[y*ocmap.info.width+x] = int(np.maximum(likelyhood * 100,1))

    likelymap_msg = ocmap   
    #likelymap_msg.data = grid

    print("Finished Likelyhoodmap initialization")

    map_orientation = ocmap.info.origin.orientation
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w]) 
    map_angle = yaw



    #print(np.rint(a))

    prob = 1.0

    while not rospy.is_shutdown():

        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
        particle_pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) 
        particle_angle = yaw

        

        
        #print(num_particles)

        


        for i in range(0,1000):

            prob = 1.0

            for observation in observations:


               
                #transform point from [0,0] to particle position
    
                particle_point = transform_point(observation, particle_angle, particle_pos)

                #transform point from world to map

                point = np.rint(transform_point(np.array(particle_point*100),-map_angle,np.array([-0.0,0.0])))
                point2 = np.rint(np.array(particle_point*100))
                if ((0 < point2[0] < ocmap.info.height-1) and (0 < point2[1] < ocmap.info.width-1)): 

                #print(str(int(point[0]))+";"+str(int(point[1])))
                    gridtarget = ((int(point[1])*ocmap.info.width)+int(point[0]))
                #print(likelyhood_map[int(point[0]-1),int(point[1]-1)])
                #    grid[gridtarget] = 100 - int(likelyhood_map[int(point[0]),int(point[1])]*100)
                    if(i==1):
                        grid[gridtarget] = 100-likelyhood_grid[gridtarget]
                #print(likelyhood_map[int(point[0]-1),int(point[1]-1)])
                    #prob *= likelyhood_map[int(point[0]),int(point[1])]   
                        prob *= likelyhood_grid[gridtarget] / 100             
        

        print(prob)  

        likelymap_msg.data = grid
        mg_pub.publish(likelymap_msg)
        rate.sleep()


if __name__ == '__main__':
    listener()
