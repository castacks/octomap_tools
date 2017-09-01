#!/bin/bash

echo "Begin to convert PCD to Octomap"

#Step 0:
#   You may want to downsample the pointcloud first. To do this simply

echo "Step 0: Cloud downsample"
rosrun octomap_tools cloud_downsample in_cloud.pcd 0.05 new_cloud.pcd
pcl_viewer -ax 1 new_cloud.pcd

#Step 1: 
#   The coordinate of Ji's PCD file is: Y upward, Z forward, X left
#   Our global map coordinate is: X forward, Z downward, Y right
#   So we need to transform the point cloud: roll, pitch, yaw
#   It should be: -1.57 0.0 -1.57 x, y, z
echo "Step 1: Coordinate transform:"
rosrun octomap_tools cloud_transform in_cloud.pcd -1.57   0.0   0.0   0.0   0.0   -1.3 new_cloud.pcd
pcl_viewer -ax 1 new_cloud.pcd

#Step 2: 
#In order to get a better visualization, we can crop the ceiling points by specifying the Z value:
#Parameter: axis min max 
echo "Step 2: Crop the point cloud:"
rosrun octomap_tools cloud_passthrough new_cloud.pcd z -3.0 0.1 crop_cloud.pcd
pcl_viewer -ax 1 crop_cloud.pcd


#Step 3:
#Convert the point cloud file into octomap using cloud2octomap package:
#Note: Since we convert a very big PCD file into octomap, the resolution could not be very high, otherwise there is no enough memory. 
#Parameter: resolution
echo "Step 3: Convert to octomap:"
rosrun octomap_tools cloud2octomap crop_cloud.pcd 0.05 crop_cloud.bt
octovis crop_cloud.bt


#Step 4:
# Merge multiple pointclouds into one octomap. 
# rosrun octomap_tools clouds2octomap [resolution] [path of generated bt file] [path of transformation information csv file] [path to all point clouds] [true = pcd in body frame, false = pcd in global frame]
# transforms.csv should follow the format:
# [pcd file name], t(0), t(1), t(2), t(3), t(4), t(5), t(6), t(7), t(8), t(9), t(10), t(11), t(12), t(13), t(14), t(15), 
echo "Step 4: Merge multiple pointclouds into octomap:"
rosrun octomap_tools clouds2octomap 0.05 ./octomap.bt ./transforms.csv ./ true

echo "Convertion is completed!"
