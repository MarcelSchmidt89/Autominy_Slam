from multiprocessing import Pool
import numpy as np


def worker(index):
	global poses
	global maps
	global pointlist
	#print('hello?')
	pose = poses[index] * index
	mymap = maps[index] * index
	points = pointlist[index] * index

	return pose, mymap, pointlist


t_pose = np.array([1,1])
t_map = np.array([1.0,2.0,3.0])
t_points = np.array([[1,1],[2,2],[3,3]])

index = np.array([0,1,2])

poses = []
maps = []
pointlist = []


for i in index:
	poses.append(t_pose)
	maps.append(t_map)
	pointlist.append(t_points)


if __name__ == '__main__':

	#pool = Pool()


	for i in range(10):
		pool = Pool(8)
		results = []
		results = pool.map(worker,index)
		for idx,b in enumerate(results):
			poses[idx] = b[0]
			maps[idx] = b[1]
			pointlist[idx] = b[2]
			#print(idx)
			#print(b)
		print(results[2][0])

		pool.close()
		pool.join()






