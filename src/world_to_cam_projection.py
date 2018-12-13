import numpy as np
import cv2
from open3d import *
import matplotlib.pyplot as plt
from plyfile import PlyData, PlyElement
from pyquaternion import Quaternion
import PIL.Image

import pandas as pd
from pyntcloud import PyntCloud
import glob
import os
import sys 
import collections
#from PIL import Image


Camera = collections.namedtuple(
    "Camera", ["id", "model", "width", "height", "params"])

Image = collections.namedtuple(
    "Image", ["id", "qtvec", "camera_id", "name"])



def read_model(path, ext):
	if ext == ".txt":
		cameras = read_cameras_text(os.path.join(path, "cameras" + ext))
		images = read_images_text(os.path.join(path, "images" + ext))
		print ("len of intrinsic camera: ", len(cameras))
		print ("len of extrinsice camera: ", len(images))
	else:
		cameras = None
		images = None
	return cameras, images
def read_cameras_text(path):
	cameras = {}
	with open(path, "r") as fid:
		while True:
			line = fid.readline()
			if not line:
				break
			line = line.strip()
			if len(line) > 0 and line[0] != "#":
				elems = line.split()
				camera_id = int(elems[0])
				model = elems[1]
				width = int(elems[2])
				height = int(elems[3])
				params = np.array(tuple(map(float, elems[4:])))
				cameras[camera_id] = Camera(id=camera_id, model=model,\
					width=width, height=height,params=params)
	return cameras

def read_images_text(path):
	images = {}
	with open(path, "r") as fid:
		while True:
			line = fid.readline()
			if not line:
				break
			line = line.strip()
			if len(line) > 0 and line[0] != "#":
				elems = line.split()
				image_id = int(elems[0])
				qvec = np.array(tuple(map(float, elems[1:5])))
				tvec = np.array(tuple(map(float, elems[5:8])))
				camera_id = int(elems[8])
				image_name = elems[9]
				#import pdb; pdb.set_trace()
				my_quaternion = Quaternion(qvec)
				R = my_quaternion.rotation_matrix         # 3x3 rotation matrix
				T = my_quaternion.transformation_matrix   # 4x4 transformation matrix
				### replace the translation part in T above and generate the real transform matrix
				T[:3,3] = tvec
				#print("world_to_camera_transformation: ", T)  
				elems = fid.readline().split()
				xys = np.column_stack([tuple(map(float, elems[0::3])),tuple(map(float, elems[1::3]))])
				point3D_ids = np.array(tuple(map(int, elems[2::3])))
				images[image_id] = Image(id=image_id, qtvec=T, camera_id=camera_id, name=image_name)
	return images



def load_pcd(velPC_path):
	#visulize the ply and read in the 3d coordinate information
	pcd_file = read_point_cloud(velPC_path)
	#draw_geometries([pcd_file])
	pcd_points = np.asarray(pcd_file.points) ## PointCloud with 113101 points.
	points_x = pcd_points[:, 0]
	points_y = pcd_points[:, 1]
	points_z = pcd_points[:, 2]
	points = np.hstack((points_x[:, None], points_y[:, None], points_z[:, None]))
	points = points.T
	# in order to get [X Y Z 1].T
	one_stack = np.full((1, points.shape[1]), 1)
	homo_points = np.concatenate((points, one_stack))   ## (4, 109502)
	return pcd_points, homo_points  



def convtMat(arr):
	mat = np.zeros((3,4), dtype=float)
	mat[0, 0] = arr[0]
	mat[1, 1] = arr[0]
	mat[0, 2] = arr[1]
	mat[1, 2] = arr[2]
	mat[2, 2] = 1
	return mat
def cam_to_pixel(P_intrinsic,camera_points):
	imageCor_points = []
	imageCor_points = np.matmul(P_intrinsic, camera_points)
	imageCor_points = imageCor_points[::] / imageCor_points[::][2]
	imageCor_points = np.delete(imageCor_points, 2, axis=0)
	imageCor_points = imageCor_points.T  ##should be 3789,2
	return imageCor_points

def get_image(img_path):
	print (img_path)
	#import pdb;pdb.set_trace()

	#image = Image.open(img_path, 'r')
	image = PIL.Image.open(img_path)

	width, height = image.size
	rgb_values = list(image.getdata())
	if image.mode == 'RGB':
		channels = 3
		print("image mode: %s" % image.mode)
	elif image.mode == 'L':
		channels = 1
	else:
		print("Unknown mode: %s" % image.mode)
		return None
	rgb_values = np.array(rgb_values).reshape((height, width, channels)) 
	#print ("pixel_values:   ", pixel_values)
	return width, height, rgb_values
def draw_pcd_geometries(pcd_path):
	pcd_file = read_point_cloud(pcd_path)
	print ("pcd_file: ", pcd_file)
	draw_geometries([pcd_file])


def project_on_image(pcd_points, uv_coor,width, height, rgb_, text, byte_order, filename):
	selected_uv_coor = uv_coor[(uv_coor[:, 0]>=0) & (uv_coor[:, 1]>=0) & (uv_coor[:, 0]<width) & (uv_coor[:, 1]<height)]
	selected_pcd_points = pcd_points[(uv_coor[:, 0]>=0) & (uv_coor[:, 1]>=0) & (uv_coor[:, 0]<width) & (uv_coor[:, 1]<height)]
	u = selected_uv_coor[:, 0].astype('int')  ### col, width
	v = selected_uv_coor[:, 1].astype('int')  ### row, height
	combined_array = np.concatenate([selected_pcd_points, rgb_[v, u, :]], 1)
	vertex =  np.array([tuple(ele) for ele in combined_array], dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4'),('red', 'u1'), ('green', 'u1'),('blue', 'u1')])
	ply = PlyData(
		[
			PlyElement.describe(
				vertex, 'vertex',
				comments=['vertices']
			)
		],
		text=text, byte_order=byte_order,
		comments=['single tetrahedron with colored faces']
	)
	filename = filename + ".ply"
	ply.write(str(filename))
	#draw_pcd_geometries(filename)
	print("finished: ", filename)



def main():
	""" read camera.txt and images.txt for intrinsic and extrisic(world to camera)"""
	path = '/Users/txl0524/Desktop/notebooks/projection_pipeline/data/'
	cameras, images = read_model(path, ".txt")

	velPC_path = '/Users/txl0524/Desktop/notebooks/projection_pipeline/data/fused.ply'
	pcd_points, homo_points = load_pcd(velPC_path)

	img_folder = '/Users/txl0524/Desktop/notebooks/projection_pipeline/image/'

	for key in images.keys():
		print ("1. image key: ", key)
		print ("2. camera_id: ", images[key].camera_id)
		camera_id = images[key].camera_id
		intrinsic_arr = cameras[camera_id].params
		intrinsic_mat = convtMat(intrinsic_arr)
		extrinsic_mat = images[key].qtvec
		#### world to camera coordiante system
		camera_points = np.matmul(extrinsic_mat, homo_points)
		uv_coor = cam_to_pixel(intrinsic_mat,camera_points)
		img_path = os.path.join(img_folder, images[key].name)
		filename = images[key].name.rsplit('.')[0]
		width, height, rgb_ = get_image(img_path)
		text = False
		byte_order = '<'
		project_on_image(pcd_points, uv_coor,width, height, rgb_, text, byte_order, filename)


"""
	camera:  Camera(id=1, model='SIMPLE_RADIAL', width=1280, height=720, params=array([ 1.36780e+03,  6.40000e+02,  3.60000e+02, -1.07011e-02]))
image:  Image(id=1, qtvec=array([[ 0.99995208, -0.00644838, -0.00736601, -0.762336  ],
       [ 0.00616236,  0.9992506 , -0.03821337,  1.03323   ],
       [ 0.00760691,  0.03816615,  0.99924245,  2.10508   ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]), camera_id=1, name='10000_render_lynda.png')
"""

main()

