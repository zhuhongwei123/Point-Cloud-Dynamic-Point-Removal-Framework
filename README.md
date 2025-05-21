# Point-Cloud-Dynamic-Point-Removal-Framework
The framework of our system is composed of two modules, this code is the dynamic point recognition module. The manuscript is currently under review. At present, due to a large number of experiments, the code is relatively messy. After the manuscript is accepted, it will be continuously updated in the future.

![OurRobot](https://github.com/zhuhongwei123/An-Online-dynamic-point-removal-SLAM-Framework-for-Dynamic-Environments/blob/main/Robot_hardware.png)

## Our parking lot point cloud data, and pose estimated by Fast-lio2
https://drive.google.com/drive/folders/1d7xF9cENaoifsUSjsupPBWco56bnRkK1?usp=drive_link 

## This is a demonstration video of our robot, including robot self-exploration and self-service 3D mapping, which has been realized in a simple environment. In complex environments, key technologies are still being explored.
https://youtu.be/1xsb6KZ6HNI

## Requirements
Based on C++17 <br>
ROS (and Eigen, PCL, OpenMP): the all examples in this readme are tested under Ubuntu 18.04 and ROS Melodic.

## How to use
$ mkdir -p ~/catkin/workspace_ws/src <br>
$ cd ~/catkin/workspace_ws/src <br>
$ git clone https://github.com/zhuhongwei123/An-Online-dynamic-point-removal-SLAM-Framework-for-Dynamic-Environments.git <br>
$ cd ..  <br>
$ catkin_make <br>
$ source devel/setup.bash <br>
Download parking lot point cloud data and pose, and modify save_pcd_directory, sequence_scan_dir, sequence_pose_path in params_mulran_scliosam.yaml in config, and replace them with the downloaded location <br>
$ roslaunch removert run_scliosam.launch
## if you use KITTI dataset <br>
Download KITTY point cloud data and pose, and modify save_pcd_directory, sequence_scan_dir, sequence_pose_path in params_kitti.yaml in config, and replace them with the downloaded location <br>
$ roslaunch removert run_kitti.launch 
