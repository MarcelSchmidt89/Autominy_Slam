#!/usr/bin/env python
import numpy as np
import tf


def pos_to_gridcell(point, resolution, map_width):

	point = np.rint(np.array(point) * (100))
	res = 0
	
	if ((point[0] < 600) and (point[1] < 430)):
		res = int((point[1]*map_width + point[0]))
	

	return res


def rectangle_check(a, b, c, d, pos):
	am = pos - a 
	ab = b - a
	ad = d - a
	return (0 < am @ ab < ab @ ab) and (0 < am @ ad < ad @ ad)


def transform_point(point, angle, transform):
	#angle = np.radians(angle)
	x = np.cos(angle)*point[0] - point[1]*np.sin(angle) + transform[0]
	y = np.sin(angle)*point[0] + point[1]*np.cos(angle) + transform[1] 
	return np.array([x,y])


def get_odom_vector(odom):
	orientation = odom.pose.pose.orientation
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) 
	pos = np.array([odom.pose.pose.position.x , odom.pose.pose.position.y]) 
	return(np.array([pos[0],pos[1],yaw]))


def get_map_orientation(ocmap):
	map_orientation = ocmap.info.origin.orientation
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w]) 
	return yaw

