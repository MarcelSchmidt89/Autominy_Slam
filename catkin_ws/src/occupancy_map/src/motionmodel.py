#!/usr/bin/env python
import numpy as np
from utils import *


a1 = 0.08
a2 = 0.08
a3 = 0.05
a4 = 0.16

def odometry_model_simpel(odometry):


	new_x = np.random.normal(odometry[0],0.05)
	new_y = np.random.normal(odometry[1],0.05)
	new_angle = np.random.normal(odometry[2],0.17)

	new_angle = odometry[2]
	
	return np.array([new_x,new_y,new_angle])


def get_motion_delta(odom_old, odom_new):

	delta_trans = np.sqrt(np.square(odom_new[0]-odom_old[0]) + np.square(odom_new[1]-odom_old[1]))

	delta_rot1 = np.arctan2(odom_new[1]-odom_old[1],odom_new[0]-odom_old[0]) - odom_old[2]

	delta_rot2 = odom_new[2] - odom_old[2] - delta_rot1

	#print(str(odom_old)+" "+str(odom_new)+" "+str(delta_rot2))

	return np.array([delta_trans, delta_rot1, delta_rot2])


def sample_motion_model(deltas, position):

	delta_trans = deltas[0] + np.random.normal(0, a3*deltas[0] + a4 * (np.abs(deltas[1]) + np.abs(deltas[2])) )
	delta_rot1 = deltas[1] + np.random.normal(0, a1*np.abs(deltas[1]) + a2 * deltas[0])
	delta_rot2 = deltas[2] + np.random.normal(0, a1*np.abs(deltas[2]) + a2 * deltas[0])

	x = position[0] + delta_trans * np.cos(position[2] + delta_rot1)
	y = position[1] + delta_trans * np.sin(position[2] + delta_rot1)
	rotation = position[2] + delta_rot1 + delta_rot2

	return np.array([x,y,rotation])




