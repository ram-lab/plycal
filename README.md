# Extrinsic Calibration of the camera and LiDAR via polygon plane
This project includes the extrinsic calibration library and gui tool for calibrating the extrinsic parameter(6-DoF rigid-body transformation) between the camera and LiDAR. This method needs convex polygon plane as calibration object and the camera intrinsic parameter(K, D) are needed (if images are not undistorted).  

*note: current implemention only support rectangular plane*

## Dependency list
- [pcl](http://pointclouds.org/) 
- [ceres](http://ceres-solver.org/)
- Eigen 3
- VTK
- Qt 5 (system)

## Build
we test on `Ubuntu 16.04`, if you already install ros kinect and then you only need to compile and install ceres-solver. 
```sh
$ mkdir build& cd build
$ cmake .. 							# build library and QT GUI
$ cmake -DBUILD_PlyCal_TEST=True ..	#build test tools(only for debug usage)
$ make
```


## Test
We have tested the current qt-based tool with RSLidar-16, RSlidar-32, Rslidar-mems by rectangular plane. And we provide test data and config file under `./data`directory which was collected with RSLidar-16 and usb webcam.

## Usage  
[中文使用说明](./doc/README.md)  

Before using the gui tool:
1. Calibrate camera and put intrinsic parameter `D, K` at config file or set by gui.  
2. Collect synced image(png,jpg,jpeg) and pointcloud (pcd), put them into `some_place/image_orig` and  `some_place/pointcloud` directory, respectively. Name the file like `0.jpg, 1.jpg, ..., n.jpg`.  
![Demo](./doc/demo.gif) 


## Reference  
For the method you can read the paper:  
```
@INPROCEEDINGS{8665256, 
author={Q. {Liao} and Z. {Chen} and Y. {Liu} and Z. {Wang} and M. {Liu}}, 
booktitle={2018 IEEE International Conference on Robotics and Biomimetics (ROBIO)}, 
title={Extrinsic Calibration of Lidar and Camera with Polygon}, 
year={2018}, 
volume={}, 
number={}, 
pages={200-205}, 
keywords={calibration;cameras;feature extraction;image fusion;image sensors;micromechanical devices;optical radar;sensor fusion;stereo image processing;heterogeneous exteroceptive sensors;heterogeneous sensory systems;multisensor information;polygon board;t6/32-beam Lidar;extrinsic calibration;MEMS-Lidar;point-cloud;laser range finder;2D feature space;3D feature space;Calibration;Cameras;Laser radar;Three-dimensional displays;Sensors;Image edge detection;Two dimensional displays}, 
doi={10.1109/ROBIO.2018.8665256}, 
ISSN={}, 
month={Dec},}
```