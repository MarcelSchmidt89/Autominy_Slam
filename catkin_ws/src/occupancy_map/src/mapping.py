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
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from scipy import spatial
import time
from sensormodel import *
from motionmodel import *
import copy

def gaussian(x, mu, sig):
    return 1./(np.sqrt(2.*np.pi)*sig)*np.exp(-np.power((x - mu)/sig, 2.)/2)

gausscale = gaussian(0.0,0.0,0.01)

prior = 0.5


def OccupancyGridMapping(Occupancy_Grid,Occupancy_Map, MapInfo, Observations, Odometry):

	position = np.array([Odometry[0],Odometry[1]])

	obs = []

	map_angle = get_map_orientation(MapInfo)

	for ob in Observations:
		obs.append(transform_point(ob,Odometry[2],position))

	kdtree = spatial.cKDTree(obs) 

	pos_int_x = int(np.rint(position[0]*100))
	pos_int_y = int(np.rint(position[1]*100))

	for x in range(pos_int_x-100, pos_int_x+100):
		for y in range(pos_int_y-100, pos_int_y+100):

			point = np.array([x,y])                

        
			car_point = transform_point(point/100, -0, -position)
			car_point = transform_point(car_point, -Odometry[2], np.array([0,0]))


			if(0.2 < np.linalg.norm(car_point) < 0.8) and (angle_between_vectors(car_point,np.array([1,0])) < 0.69) :

				dist, index = kdtree.query(np.array([x,y])/100)   

				point = np.rint(transform_point(point,-map_angle,np.array([0.0,0.0])))
				gridtarget = int((point[1]+MapInfo.height*0.75)*(MapInfo.width))+int(point[0]+MapInfo.width/4)

				new_prob = np.clip(((gaussian(dist,0.0,0.01))/gausscale),0.4,0.6) 

				old_prob = Occupancy_Grid[gridtarget] 


				new_odds = (new_prob/(1-new_prob)) * (old_prob/(1-old_prob)) * ((1-prior)/prior)

				new_prob = 1 / (1+  ((1-new_prob)/new_prob) * ((1-old_prob)/old_prob) * (prior/(1-prior)) )

				Occupancy_Grid[gridtarget] = new_prob

				Occupancy_Map[gridtarget] = int(new_prob*100)

	return Occupancy_Grid, Occupancy_Map