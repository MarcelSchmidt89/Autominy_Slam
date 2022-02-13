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


def get_map_orientation(ocmap_info):
	map_orientation = ocmap_info.origin.orientation
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([map_orientation.x, map_orientation.y, map_orientation.z, map_orientation.w]) 
	return yaw

def angle_between_vectors(vector_1, vector_2):
	unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
	unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
	dot_product = np.dot(unit_vector_1, unit_vector_2)

	angle = np.arccos(dot_product)
	return(angle)


def read_record(file):

	lines = file.readlines()


    #print(line)

	odoms = []
	points = []

	for line in lines:
        
		line2 = line.split(';')

	    #print(line2)

	    #str_odom.replace('[','')

		str_odom = line2[0]
		str_odom = str_odom.replace('[','')
		str_odom = str_odom.replace(']','')
	    #print(str_odom)
		str_odom = str_odom.split()
	    #print(str_odom)
		np_odom = np.array(str_odom).astype(np.float)

		odoms.append(np_odom)


		str_points = line2[1]

		#print(str_points)
		str_points = str_points.replace('[','')
		str_points = str_points.replace(']','')
		str_points = str_points.split("), array(")
		str_points[0] = str_points[0].replace('array(','')
		str_points[len(str_points)-1] = str_points[len(str_points)-1].replace(')\n','')
		#print(str_points[1])
		#print("....")

		rec_points = []

		for pointstr in str_points:
		    point_pair = pointstr.split(", ")
		    rec_points.append(np.array(point_pair).astype(np.float))

		#print(rec_points[1])

		points.append(rec_points)

	return(odoms,points)

