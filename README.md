# Point Cloud Segmentation
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
![](https://github.com/Eavis/PointCloudSegmentation/blob/master/origin_images/segmentedlivingarea.gif)
In the project, I am trying to solve the 3D point cloud segmentation based on indoor scene. The point clouds that I am using come from two resources, one part come from simulated environment created from Gazebo(robot simulation system), the other part generate from photogrammetry pipeline implemented using COLMAP. The demonstration will include two parts.
 - Use Velodyne HDL-32E to generate point cloud and pinhole model to generate RGB images(set up process is demonstrated in another repository). Display the projection and back-projection process here.
  - Show the images rendered using Blender Cycle and run semantic segmentation network on realistic images pretrained by "Pytorch implementation for Semantic Segmentation/Scene Parsing on MIT ADE20K dataset
  - Merge the point clouds projected from different poses and postions of the camera to reconstruct the indoor segmented point cloud.
## Processing Pipeline
<p align="center">
  <img src="https://github.com/Eavis/PointCloudSegmentation/blob/master/origin_images/pipeline.png"  title="pipeline">
</p>
  - Run pretrained semantic segmentation model on realistic synthetic images to get pixel-level annotations(indicated by color)
  - Project 3D point cloud points to Images(Either World-Camera-Pixel coordinate OR Velodyne-Camera-Pixel)
  - Extrace color information(label information) from Image
  - Save a new point cloud(ply file) with 3D coordinates and RGB value

  ## New Features!
  - Set up Gazebo environment to generate high quality images to make up the shortage of 3D data.
  - Render realistic images to supplement  deep learning training process.
  - Use 2D methods to solve 3D problems
