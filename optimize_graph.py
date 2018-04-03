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


# arr: array with 6 elements (noise matrix)
# converts arr to a 2x3 matrix
def matrix_2x3(arr):
	arr1 = [0] * 6
	for i in range(0,6):
		arr1[i] = float(arr[i])
	return np.reshape(arr1,(2,3))

# arr: array with 6 elements (noise matrix)
# converts arr to 3x3 information matrix
def create_information_matrix(arr):
	arr1 = [0] * 9
	arr1[0] = float(arr[0])
	arr1[1] = float(arr[1])
	arr1[2] = float(arr[2])
	arr1[3] = float(arr[1])
	arr1[4] = float(arr[3])
	arr1[5] = float(arr[4])
	arr1[6] = float(arr[2])
	arr1[7] = float(arr[4])
	arr1[8] = float(arr[5])
	arr1 = np.reshape(arr1,(3,3))
	return np.matrix(arr1)


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
				x, y, th = make_pose(line_split[2:])
				initialEstimate.insert(node, gtsam.Pose2(x, y, th))
			elif line_split[0] == 'EDGE_SE2':
				node1 = int(line_split[1])
				node2 = int(line_split[2])
				dx, dy, dth = make_pose(line_split[3:6])
				noise = gtsam.noiseModel.Gaussian.Information(create_information_matrix(line_split[6:]))
				graph.add(gtsam.BetweenFactorPose2(node1, node2, gtsam.Pose2(dx, dy, dth), noise))
	f.close()

	# initialEstimate.print("\nInitial Estimate:\n")

	parameters = gtsam.GaussNewtonParams()

	# Stop iterating once the change in error between steps is less than this value
	parameters.relativeErrorTol = 1e-5
	# Do not perform more than N iteration steps
	parameters.maxIterations = 100
	# Create the optimizer ...
	optimizer = gtsam.GaussNewtonOptimizer(graph, initialEstimate, parameters)
	# ... and optimize
	result = optimizer.optimize()
	result.print("Final Result:\n")



if __name__ == "__main__":
    main()