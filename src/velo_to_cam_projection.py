import numpy as np
import cv2
from open3d import *
import numpy as np 
import matplotlib.pyplot as plt
from plyfile import PlyData, PlyElement
#from cameramodels import PinholeCameraModel
"""
Project Velodyne generated point cloud into camera image, do segmentation on camera image and project back as colored point cloud
"""

##how the scaling be calculated
##filter!!
class Project_Velodyne_to_Image:
	def __init__(self, img_path=None, velPC_path=None, velo2cam_path=None, camIntrin_path=None,velo_to_world=None):
		self.__img_path  = img_path             #img file path
		self.__velPC_path = velPC_path          #pcd file path
		self.__velo2cam_path = velo2cam_path    #velodyne to camera transformation file
		self.__camIntrin_path = camIntrin_path  #camera intrinsic information file
		self.__velo_to_world = velo_to_world
		self.RT_concat = self.__load_velo2cam()
		## currently is loading the projection matrix
		self.K_intrinsic, self.P_intrinsic  = self.__load_camIntrinsic() 
		self.homo_points = self.__process_veloPC()
		self.camCor_points = self.__veloCoor_to_camCoor()
		self.uv_coor = self.__camCor_to_pixel()
	""" 
	RT_concat shape:  [R|T](3, 4)
	[[ 0.   0.   0.   0. ]
	 [ 0.   0.   0.   0. ]
	 [ 0.   0.   0.  -0.1]]
	"""
	def __load_velo2cam(self):
		with open(self.__velo2cam_path, "r") as file:
			velo2cam_file = file.readlines()
		for line in velo2cam_file:
			(key,val) = line.split(":", 1)
			if key == 'R':
				R = np.fromstring(val, sep=' ').reshape(3,3)
				#print("R from velo to cam: reshape(3,3)", R)
			elif key == 'T':
				T = np.fromstring(val, sep=' ').reshape(3,1)
	                             ############**********trying -RC
				#print("T from velo to cam: reshape(3,1)", T)
		RT_concat = np.concatenate((R,T), axis=1)
		#print("RT_concat shape: ", RT_concat.shape)
		print(RT_concat)
		return RT_concat
	"""
	Question is use which one: (distortion/rectification)
	P for camIntrin:  [[610.17994701   0.         512.5        -42.71259629]
					 [  0.         610.17994701 512.5          0.        ]
					 [  0.           0.           1.           0.        ]]
	K_intrinsic:  [[610.17994701   0.         512.5       ]
				 [  0.         610.17994701 512.5       ]
				 [  0.           0.           1.        ]]
	"""
	def __load_camIntrinsic(self):
		with open(self.__camIntrin_path, "r") as file:
			camIntrinsic_file = file.readlines()
		for line in camIntrinsic_file:
			(key, val) = line.split(":", 1)
			if key == 'K':
				K_intrinsic = np.fromstring(val, sep=',').reshape(3,3)
				#print("K_intrinsic: ",K_intrinsic)
			elif key == 'P':
				P_intrinsic = np.fromstring(val, sep=',').reshape(3,4)
				print("P_intrinsic: ", P_intrinsic)
		return K_intrinsic, P_intrinsic


	def __process_veloPC(self):
		##In gazebo world coordinate is z vertical, in camera is y vertical
		##rotate for point coordinate 
		###read in pcd file and process the data to be [X Y Z 1].T 
		#transform from euclidean to homogeneous coordinate 
		pcd_file = read_point_cloud(self.__velPC_path)
		print ("pcd_file: ", pcd_file)
		pcd_points = np.asarray(pcd_file.points)
		#draw_geometries([pcd_file])
		#print("pcd_points shape: ",pcd_points.shape) ### (3789, 3)
		


		# points_a = []
		# points_b = []
		# points_c = []
		# for point in pcd_points:
		# 	if point[0] < 0 :
		# 		continue;
		# 	else:
		# 		points_a.append(-point[1])
		# 		points_b.append(-point[2])
		# 		points_c.append(point[0])
		# points_x = np.asarray(points_a)
		# points_y = np.asarray(points_b)
		# points_z = np.asarray(points_c)

		for point in pcd_points:
			if point[0] < 0:
				continue;
			else:
				uv_coor = get_UV_coor(-point[1], -point[2], point[0])
				if (uv_coor[0] <= 0 || uv_coor[0] > self.image_width || uv_coor[1] <= 0 || uv_coor[1] > self.image_height):
					continue
				else:





		####cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);

		points_x = -pcd_points[:, 1]
		points_y = -pcd_points[:, 2]
		points_z = pcd_points[:, 0]
		points = np.hstack((points_x[:, None], points_y[:, None], points_z[:, None]))

		import pdb;pdb.set_trace()

		points = points.T
		#print (points)
		# in order to get [X Y Z 1] 
		one_stack = np.full((1, points.shape[1]), 1)
		homo_points = np.concatenate((points, one_stack))
		print("points shape after stack, transpose, and concatenate: ", homo_points.shape)
		return homo_points
	def __veloCoor_to_camCoor(self):
		camCor_points = []
		camCor_points = np.matmul(self.RT_concat, self.homo_points)
		print(camCor_points.shape)
		print ("camCor_points: -----------")
		#print(camCor_points)
		return camCor_points
	def __camCor_to_pixel(self): ##multiply intrinsic matrix with camCor points

		imageCor_points = []
		#************************************************************************		#************************************************************************
				#************************************************************************
		####append 1 
		one_stack = np.full((1, self.camCor_points.shape[1]), 1)
		self.camCor_points = np.concatenate((self.camCor_points, one_stack))
		imageCor_points = np.matmul(self.P_intrinsic, self.camCor_points)
		print("imageCor_points.shape after matmul with P matrix: ", imageCor_points.shape)
		
		imageCor_points = imageCor_points[::] / imageCor_points[::][2]
		imageCor_points = np.delete(imageCor_points, 2, axis=0)
		print("imageCor_points.shape after transform from image coor to pixel coor: ", imageCor_points.shape)
		

		imageCor_points = imageCor_points.T  ##should be 3789,2
		print ("imageCor_points: -----------")#(2, 3789)
		print(imageCor_points.shape)
		print(imageCor_points)
		# plt.imshow(uv_coor)
		# plt.show()
		m1 = imageCor_points[:, 0] > 0
		m1_2 = imageCor_points[:, 0] < 800
		m2 = imageCor_points[:, 1] > 0
		m2_2 = imageCor_points[:, 1] < 800
		mask = m1 & m1_2 & m2 & m2_2
		imageCor_points = imageCor_points[mask]

		print ("imageCor_points after filter: -----------")#(2, 3789)
		print(imageCor_points.shape)
		print(imageCor_points)  ##### no points after filtering
		uv_coor = imageCor_points
		return uv_coor
	def print_projection_cv2(self):
		frame = cv2.imread(self.__img_path)
		g,b,r = cv2.split(img)
		import pdb; pdb.set_trace();

		print("frame shape: ",frame.shape)
		hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		#import pdb; pdb.set_trace()
		for i in range(self.uv_coor.shape[0]):
			cv2.circle(hsv_image, (np.int32(self.uv_coor[i][0]),\
			 np.int32(self.uv_coor[i][1])),\
			  2, \
			  (200, 200, 10), -1)

		return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
	def projection_from_vel_to_img(self):
		img = cv2.imread(self.__img_path)
		b,g,r = cv2.split(img)
		hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		for i in range(self.uv_coor.shape[0]):
			cv2.circle(hsv_image, (np.int32(self.uv_coor[i][0]), np.int32(self.uv_coor[i][1])), 2, (50, 205, 50), -1)
		return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
	def show_projection(self):
		res = self.projection_from_vel_to_img()
		#res = print_projection_cv2()
		cv2.imshow('projection result', res)
		cv2.waitKey(0)	
	def filter_uvCor(self):
		img = cv2.imread(self.__img_path)
		row, col, channel = img.shape
		print ("row: ", row)
		print ("col: ", col)
		print ("channel: ", channel)
		uv_coor = self.uv_coor.T  ###(3000,2)
		print ("uv_coor: ", uv_coor)
		for point in uv_coor:
			if ((point[0] >0 and point[1] > 0)):
				print (point)

