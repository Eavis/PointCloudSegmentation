import os
import glob
import argparse
import numpy as np
import plyfile
from open3d import *




pcd_path = '/Users/txl0524/Desktop/notebooks/projection_pipeline/output_ply/merge_res.ply'
def draw_pcd_geometries(pcd_path):
	pcd_file = read_point_cloud(pcd_path)
	print ("pcd_file: ", pcd_file)
	draw_geometries([pcd_file])
	
draw_pcd_geometries(pcd_path)
# import pdb;pdb.set_trace()