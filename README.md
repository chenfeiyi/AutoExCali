# Automatic calibration among multiple cameras and LiDARs using a checkerboard



## Description

This repo is the source code for preparing paper on ITSC. It's for the extrinsic calibration between cameras and 3D LiDARs containing camera-to-camera, lidar-to-camera and lidar-to-lidar calibration



## Structure

```
├── 20210125_IRLS_ICP :implementation of icp algorithm
│   ├── data
│   └── kernel
├── CamLiDAR  :main file of camera-lidar extrinsic calibration
├── DualCam   :main file of camera-camera extrinsic calibration 
├── DualLiDAR :main file of lidar-lidar extrinsic calibration
├── solver    :solver to solve the extrinsic calibration problem
└── tools     :tools for all other files
    ├── board_extraction : extract board points in lidar and camera frame
    ├── noise_generator
    └── plane_ransac     :   plane-ransac fitting algorithm

```

## How to use

### Camera-LiDAR calibration

- AutoCaliCamLiDAR.m is the executable file

- modify the parameters and data path 

  ```
  K = [897.4566,0,635.4040;
      0,896.7992,375.3149;
      0,0,1];
  D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
  borW=0.767; % the geometric size of the checkerboard, unit meter
  borH=0.626; % the geometric size of the checkerboard, unit meter
  pcd_path = "/home/cfy/Documents/unifiedCali/data/real-world/cam-lidar/pcd2";
  img_path = "/home/cfy/Documents/unifiedCali/data/real-world/cam-lidar/img2";
  ```

- run this file, the result will print is command window and show the projection result 



### Camera-Camera calibration

- AutoCaliDualCam.m is the executable file

- modify the parameters and data path 

  ```
  K1= [ 825.6334,0,639.9610;0,824.9260,384.6734;0,0,1.0000];
  D1=[-0.3371,0.1315,-6.3185e-06,-3.6323e-04,0];
  
  K2 = [897.4566,0,635.4040;0,896.7992,375.3149;0,0,1];
  D2 = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
  borW=0.767;
  borH=0.626;
  img_path1 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-camera/img1";
  img_path2 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-camera/img2";
  ```

- run this file, the result will print is command window and show the projection result 

### LiDAR-LiDAR calibration

- AutoCaliDualLiDAR.m is the executable file

- modify the parameters and data path 

  ```
  borW=0.77;
  borH=0.63;
  pcd_path1 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-lidar/pcd1";
  pcd_path2 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-lidar/pcd2";
  ```

- run this file, the result will print is command window and show the fusion result. To better view the fusion result, we can save a pcd file and the intensity channel denotes different point cloud.





