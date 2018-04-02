from __future__ import print_function
import gtsam
import math
import numpy as np

# arr: string array containing pose values [x, y, theta]
# returns x, y, and theta as floats
def make_pose(arr):
	x = float(arr[0])
	y = float(arr[1])
	theta = float(arr[2])
	return x, y, theta


# arr: string array to convert to floats
# converts values in arr to floats and returns as np.array
def str_to_float(arr):
	arr1 = [0] * len(arr)
	for i in range(0,len(arr)):
		arr1[i] = float(arr[i])
	return np.array(arr1)


def main():
	graph = gtsam.NonlinearFactorGraph()
	initialEstimate = gtsam.Values()

	# read data from .g2o file
	# and initialize nodes/edges
	data_file = 'input_INTEL_g2o.g2o'
	with open(data_file,'r') as f:
		for line in f:
			line_split = line.split()
			if line_split[0] == 'VERTEX_SE2':
				node = int(line_split[1])
				x, y, theta = make_pose(line_split[2:])
				initialEstimate.insert(node, gtsam.Pose2(x, y, theta))
			elif line_split[0] == 'EDGE_SE2':
				node1 = int(line_split[1])
				node2 = int(line_split[2])
				x, y, theta = make_pose(line_split[3:6])
				noise = gtsam.noiseModel.Diagonal.Sigmas(str_to_float(line_split[6:]))
				graph.add(gtsam.BetweenFactorPose2(node1, node2, gtsam.Pose2(x, y, theta), noise))
	f.close()



if __name__ == "__main__":
    main()