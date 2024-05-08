## FAST_LIO_SAM with Dynamic Removal, Multi-session mapping and Online Relocalization, developed by Yixin-F, SEU

## 0 Introdunction
- Front_end: fastlio2 + SC + GPS(Optional) + Dynamic Removal + Yolo(Optional)
- Back_end: GTSAM
- Application: Joint Pose-graph Optimization using iSAM2, Fast and Robust ICP

## 1 Prerequisites

- Ubuntu 20.04 and ROS Noetic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- GTSAM >= 4.0.2(tested on 4.0.0-alpha2)

## 3 Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/Yixin-F/yolo_dr_sc_fastlio2.git
cd ..
catkin_make
```

## 4 How to Use
### 4.1 LIO Mapping using Livox, Velodyne, Ouster or Robosense
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam mapping_*.launch
```
<!-- <p align='center'>
    <img src="/pic/1.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/2.png" alt="drawing" width="400" height =200/>
</p> -->


### 4.2 Multi-session Mapping
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam multi_session.launch
```

### 4.3 Object-level Updating
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam object_update.launch
```

### 4.4 Online Relocalization
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam online_relocalization.launch
```

## 5 More
### 5.1 Utilize the iVox to achieve more efficient dynamic removal
### 5.2 Develop a Multi-agent Exploration system
### 5.3 Improve the STD or Scan Context ++ to replace the Scan Context

