#!/usr/bin/env python
import numpy as np
from utils import *

def end_point_model(odometry, observations, likelyhoodmap, map_angle, map_height, map_width):
	
	prob = 1.0


	for observation in observations:

		if(np.linalg.norm(observation)<0.8):

			particle_point = transform_point(observation, odometry[2], np.array([odometry[0],odometry[1]]))

		#transform point from world to map
	        

			point = np.rint(transform_point(np.array(particle_point*100),-map_angle,np.array([-0.0,0.0])))
			#point2 = np.rint(np.array(particle_point*100))




			gridtarget = int((point[1]+map_height*0.75)*(map_width))+int(point[0]+map_width/4)
			#gridtarget = ((int(point[1])*map_width)+int(point[0]))
			#gridtarget = 1000
			#if ((0 <= point2[0] < map_height-1) and (0 <= point2[1] < map_width-1)):
			
			#prob *= likelyhoodmap[gridtarget] / 100

			prob *= likelyhoodmap[gridtarget]			
			
				#grid[gridtarget] = 100-likelyhoodmap[gridtarget]
			#else:
			#	prob *= 0.5

	return prob


def end_point_model_debug(odometry, observations, likelyhoodmap, map_angle, map_height, map_width, debug_grid):
	prob = 1.0

	grid = debug_grid


	for observation in observations:

		particle_point = transform_point(observation, odometry[2], np.array([odometry[0],odometry[1]]))

	#transform point from world to map
        

		point = np.rint(transform_point(np.array(particle_point*100),-map_angle,np.array([-0.0,0.0])))
		point2 = np.rint(np.array(particle_point*100))


		gridtarget = int((point[1]+map_height*0.75)*(map_width))+int(point[0]+map_width/4)

		#gridtarget = ((int(point[1])*map_width)+int(point[0]))

		if ((0 <= point2[0] < map_height-1) and (0 <= point2[1] < map_width-1)):
			prob *= likelyhoodmap[gridtarget] / 100
			grid[gridtarget] = 100-likelyhoodmap[gridtarget]
			#grid[gridtarget] = 0

		else:
			prob *= 0.5

	return prob, grid

