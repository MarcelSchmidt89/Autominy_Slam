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

points = []
grid = []
grid = np.zeros((430*600), dtype=np.int8)


def callback(data):
    global observations
    observations = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    rows = np.random.choice(observations.shape[0] , 10)
    observations = observations[rows, :]





def odom_callback(data):
    global odom
    odom = data
    
def listener():

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
    rate = rospy.Rate(1) # 10hz

    


    

    for point in np.nditer(grid, op_flags=['readwrite']):
        point += 100

    #for x in range(0,300):
    #    for y in range(0,430):
    #        grid[y*600 + x] = 0

    


    meta = MapMetaData()

    meta.resolution = 0.01
    meta.width = 600
    meta.height = 430
    meta.origin = Pose()

    a = np.array([-25.0, 0.0]) 
    b = np.array([25.0, 0.0]) 
    c = np.array([25.0, 50.0])  
    d = np.array([-25.0, 50.0])

    a = np.array([90.0, 0.0])



    #print(np.rint(a))

    while not rospy.is_shutdown():
        img_msg = OccupancyGrid()
        img_msg.info = meta        
        img_msg.data = grid

        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
        pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) * 100
        angle = yaw


        transform_mat = np.array([[np.cos(angle), np.sin(angle),0],[-np.sin(angle),np.cos(angle),0],[pos[0],pos[1],1]])


         
        target = np.rint(transform_point(a, angle, pos))




        minx = int(target[0]-30)
        maxx = int(target[0]+30)

        miny = int(target[1]-30)
        maxy = int(target[1]+30)



        #print(str(pos) +" ; "+ str(np.radians(angle)) + " ; " + str(target))


        for i in range(0,1):  
            for x in range(minx,maxx):
                for y in range(miny,maxy):
                    if ((0 <= x < 600) and (0 <= y < 430)):
                         for point in observations:
                            p1 = np.array([point[0],point[1]])*100
                            p2 = np.array([float(x),float(y)])                                        
                            dist= np.linalg.norm(p1-p2)
                            if dist <=0.5:                                
                                grid[y*600+x] = 0

        #print("done")



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
