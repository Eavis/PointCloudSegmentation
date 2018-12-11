from Projection import Project_Velodyne_to_Image

velo2cam_path = '/Users/txl0524/Desktop/notebooks/velo_to_cam/calib_velo_to_cam.txt'
camIntrin_path = '/Users/txl0524/Desktop/notebooks/velo_to_cam/data/intrinsic.txt'

velPC_path = '/Users/txl0524/Desktop/notebooks/velo_to_cam/data/001.pcd'
#img_path= '/Users/txl0524/Desktop/notebooks/velo_to_cam/data/001.png'
img_path= '/Users/txl0524/Desktop/notebooks/velo_to_cam/data/001.jpg'
velo_to_world = '/Users/txl0524/Desktop/notebooks/velo_to_cam/data/calib_velo_to_world.txt'


res = Project_Velodyne_to_Image(velo2cam_path=velo2cam_path, camIntrin_path=camIntrin_path, velPC_path=velPC_path, img_path=img_path,velo_to_world=velo_to_world)


res.show_projection()
