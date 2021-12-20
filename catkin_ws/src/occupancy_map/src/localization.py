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

odom = Odometry()
observations = []


def callback(data):
    global observations
    observations = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    #rows = np.random.choice(observations.shape[0] , 100)
    #observations = observations[rows, :]





def odom_callback(data):
    global odom
    odom = data
    
def listener():

    points = []
    grid = []

    
    mapwidth = 600
    mapheight = 430
    mapres = 0.01


    grid = np.zeros((mapheight*mapwidth), dtype=np.int8)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    mg_pub = rospy.Publisher('/sensors/map2/', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/sensors/road_marking_localization/random_sampled_pcl", PointCloud2, callback)
    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(5) # 10hz
    

    for point in np.nditer(grid, op_flags=['readwrite']):
        point += 100

    #for x in range(0,300):
    #    for y in range(0,430):
    #        grid[y*600 + x] = 0

    


    meta = MapMetaData()

    meta.resolution = mapres
    meta.width = mapwidth
    meta.height = mapheight
    meta.origin = Pose()


    a = np.array([18.0, 0.0])



    #print(np.rint(a))

    while not rospy.is_shutdown():
        img_msg = OccupancyGrid()
        img_msg.info = meta        
        img_msg.data = grid

        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
        pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) * (1/mapres)
        angle = yaw


       
        target = np.array([0,0])

        offset = 100


        for x in range(0,1):
            for y in range(-80,80):               
                    target = np.array([x+offset,y]) 
                    ##print(str(target)+"; "+str(pos))
                    target = np.rint(transform_point(target, angle, pos))
                    if ((0 <= target[0] < mapwidth) and (0 <= target[1] < mapheight)):
                        for point in observations:
                            grid[int(target[1])*mapwidth+int(target[0])] = 0

                        # for point in observations:
                        #     p1 = np.array([point[0],point[1]])*(1/mapres)
                        #     p2 = target                                     
                        #     dist= np.linalg.norm(p1-p2)
                        #     if dist <=1.0:                                
                        #         grid[int(target[1])*mapwidth+int(target[0])] = 0



        #print(str(pos) +" ; "+ str(np.radians(angle)) + " ; " + str(target))





        # for i in range(0,100):  
        #     for x in range(minx,maxx):
        #         for y in range(miny,maxy):
        #             if ((0 <= x < mapwidth) and (0 <= y < mapheight)):
        #                  for point in observations:
        #                     p1 = np.array([point[0],point[1]])*(1/mapres)
        #                     p2 = np.array([float(x),float(y)])                                        
        #                     dist= np.linalg.norm(p1-p2)
        #                     if dist <=0.5:                                
        #                         grid[y*mapwidth+x] = 0

        # #print("done")



        #for i in range(0,100):
        #    for point in observations:
        #        grid[pos_to_gridcell(point, 0.01, 600)] = 0

        #print("done")
#        img_msg.header = bild.header
#       img_msg.height = bild.height
#        img_msg.width = bild.width
#        img_msg.encoding = bild.encoding
#        img_msg.is_bigendian = bild.is_bigendian
#        img_msg.step = bild.step
#        img_msg.data = bild.data

        mg_pub.publish(img_msg)
        rate.sleep()


if __name__ == '__main__':
    listener()
