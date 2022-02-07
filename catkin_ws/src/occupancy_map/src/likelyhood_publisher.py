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

init = False


def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

def map_callback(data):
    global ocmap
    global init
    ocmap = data
    init = True
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/slam/likelyhoodmap', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/sensors/map/", OccupancyGrid, map_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(1) # 10hz


    #wait for map

    while not init:
        print("waiting...")
        rate.sleep()

    points = []

    for x in range(0,ocmap.info.width):
        for y in range(0,ocmap.info.height):
            if ocmap.data[y*ocmap.info.width+x] <= 5:
                #print(str(x)+","+str(y))
                points.append([x,y])


    kdtree = spatial.cKDTree(points)

    last_dist = 10000.0
    grid = np.zeros((ocmap.info.height*2*ocmap.info.width*2), dtype=np.int8)
    grid.fill(100)


    gausscale = gaussian(0.0,0.0,10.0)

    #print(int(np.rint((gausscale / gausscale) * 100)))


    x_min = int(-ocmap.info.width + (ocmap.info.width/4))
    x_max = int(ocmap.info.width + (ocmap.info.width/4))

    y_min = int(-ocmap.info.height + (ocmap.info.height/4))
    y_max = int(ocmap.info.height + (ocmap.info.height/4))

    for x in range(x_min,x_max):
        for y in range(y_min,y_max):

            dist, index = kdtree.query(np.array([x,y]))            
            #print(dist)
            occ_value = int(((gaussian(dist,0.0,10.0)/gausscale) * 100)) 
            #print(str(dist)+" : "+str(occ_value))

            grid[int(y+ocmap.info.height/2)*(ocmap.info.width*2)+int(x+ocmap.info.width/2)] = occ_value

    likelymap = ocmap
    likelymap.info.width= ocmap.info.width*2
    likelymap.info.height = ocmap.info.height*2
    likelymap.info.origin.position.x = likelymap.info.origin.position.x + (ocmap.info.height/100/4)
    likelymap.info.origin.position.y = likelymap.info.origin.position.y - (ocmap.info.width/100/4)
    likelymap.data = grid

    print("done")



    #print(np.rint(a))

    while not rospy.is_shutdown():

        mg_pub.publish(likelymap)
        rate.sleep()


if __name__ == '__main__':
    listener()
