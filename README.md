## FAST_LIO_SAM with Dynamic Removal, Multi-session mapping and Online Relocalization, developed by Yixin-F, SEU

## 0 Introdunction
- Front_end: fastlio2 + SC + GPS(Optional) + Dynamic Removal + Yolo(Optional)
- Back_end: GTSAM
- Application: Joint Pose-graph Optimization using iSAM2, Fast and Robust ICP

## 1 Prerequisites

- Ubuntu 20.04 and ROS Noetic
- PCL >= 1.10 (default for Ubuntu 20.04)
- Eigen >= 3.3.4 (default for Ubuntu 20.04)
- GTSAM >= 4.0.3 (test on 4.2(a))
- Livox Driver
- Darknet-ros

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
You need to first check the Config/*.yaml about the settings for different LiDAR types, we list parameters here. "-" means that it depends on your own project.

| Parameters | 中文解释 | default(默认值) |
| --- | --- | --- |
| lid_topic | 雷达话题名称 | - |
| imu_topic | IMU话题名称 | - |
| time_sync_en | 是否进行时间软同步 | false |
| rootDir | 结果保存根路径 | - |
| savePCD | 是否保存点云帧PCD | true |
| savePCDDirectory | 点云帧PCD保存路径 | PCDs/ |
| saveSCD | 是否保存点云帧SCD | true |
| saveSCDDirectory | 点云帧SCD保存路径 | SCDs/ |
| saveLOG | 是否保存LOG文件 | true |
| saveLOGDirectory | LOG文件保存路径 | LOG/ |
| map_save_en | 是否保存地图 | true |
| pcd_save_interval | -（未使用） | - |
| lidar_type | 雷达类型 | - |
| livox_type | Livox类型 | - |
| scan_line | 雷达线数 | - |
| blind | 无效距离范围(m) | 1.0 |
| feature_enabled | 是否进行特征提取 | false |
| point_filter_num | 有效采样点步长 | 1 |
| scan_rate | 扫描频率 | - |
| time_unit | 时间单位 | - |
| camera_en | 是否使用相机 | false |
| camera_external | 相机到IMU外参 | - |
| camera_internal | 相机内参 | - |
| acc_cov | 线加速度协方差 | 0.1 |
| gyr_cov | 角速度协方差 | 0.1 |
| b_acc_cov | 线加速度偏置协方差 | 0.001 |
| b_gyr_cov | 角速度偏置协方差 | 0.001 |
| fov_degree | 视角范围(deg) | 180.0 |
| det_range | 最远探测距离(m) | 100.0 |
| cube_len | i-kdtree维护立方体边长(m) | 500 |
| extrinsic_est_en | 是否在线标定雷达到IMU外参 | false |
| mappingSurfLeafSize | 点云地图下采样分辨率(m) | 0.2 |
| keyframeAddingDistThreshold | 关键帧平移阈值(m) | 1.0 |
| keyframeAddingAngleThreshold | 关键帧旋转阈值(rad) | 0.2 |
| extrinsic_T | 雷达到IMU平移外参 | - |
| extrinsic_R | 雷达到IMU旋转外参 | - |
| max_iteration | ESKF最大迭代次数 | 3 |
| recontructKdTree | 是否重建i-kdtree | false |
| kd_step | i-kdtree重建步长 | 50 |
| filter_size_map_min | i-kdtree下采样分辨率(m) | 0.2 |
| loopClosureEnableFlag | 是否开启回环检测 | false |
| loopClosureFrequency | 回环检测频率(hz) | 1.0 |
| historyKeyframeSearchRadius | 有效回环检测搜索半径(m) | 10.0 |
| historyKeyframeSearchTimeDiff | 有效回环检搜索时差(s) | 30.0 |
| historyKeyframeSearchNum | 历史帧搜索个数 | 1 |
| historyKeyframeFitnessScore | icp检验阈值 | 0.2 |
| ground_en | 是否进行地面约束 | false |
| tollerance_en | 是否使用自由度阈值约束 | false |
| sensor_height | 传感器高度(m) | - |
| z_tollerance | z轴约束阈值(m) | 2.0 |
| rotation_tollerance | pitch和roll约束阈值(rad) | 0.2 |
| path_en | 是否发布位姿轨迹 | true |
| scan_publish_en | 是否发布点云 | true |
| dense_publish_en | 是否发布稠密点云 | true |
| scan_bodyframe_pub_en | 是否发布IMU坐标系下点云 | false |
| globalMapVisualizationSearchRadius | i-kdtree搜索距离(m) | 10.0 |
| globalMapVisualizationPoseDensity | i-kdtree位姿采样步长 | 1 |
| globalMapVisualizationLeafSize | i-kdtree可视化下采样分辨率(m) | 0.2 |
| visulize_IkdtreeMap | 是否发布i-kdtree | true |
|  |  |  |

After you have run the command, there are several files being generated in the filefold "rootDir/*" as follows:

<img src="./pic/lio_file.png" alt="Files generated after running LIO" width="600">

| File Name | 中文解释 |
| --- | --- |
| LOG | 日志文件 |
| PCDs | PCD格式 关键帧点云 |
| SCDs | SCD格式 关键帧Scan Context描述子 |
| globalMap.pcd | PCD格式 全局地图 |
| singlesession_posegraph.g2o | g2o格式 全局位姿图 |
| trajectory.pcd | PCD格式 xyz位姿轨迹|
| transformations.pcd | PCD格式 xyz+rpy位姿轨迹 |
|  |  |

### 4.2 Multi-session Mapping
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam multi_session.launch
```

You also need to first check the Config/multi_session.yaml, we list parameters here. "-" means that it depends on your own project.

| Parameters | 中文解释 | default(默认值) |
| --- | --- | --- |
| sessions_dir | 存储多个lio结果的根路径 | - |
| central_sess_name | 中心阶段lio文件名称 | - |
| query_sess_name | 子阶段lio文件名称 | - |
| save_directory | 多阶段结果保存路径 | - |
| iteration | isam2迭代优化次数 | 3 |
|  |  |  |

If you wanna run this multi-session module, you should have two-stage results from the LIO mapping module, more details can be found in the last section. We give examples on Parkinglot Dataset here.

<img src="./pic/multi-session.png" alt="Files generated after running multi-session" width="800">

| File Name | 中文解释 |
| --- | --- |
| 01 ~ 02 | 01 ~ 02 的单阶段 lio mapping 结果，文件格式同 4.1 |
| 01** | ** 对 01 的 multi-session 结果 |
|  |  |

We show the details in file "0102" as follows:

<img src="./pic/multi_session_details.png" alt="Files generated after running multi-session" width="800">

| File Name | 中文解释 |
| --- | --- |
| 01_central_aft_intersession_loops.txt | TXT格式 multi-session后 中心坐标系下01位姿轨迹 |
| 01_central_bfr_intersession_loops.txt | TXT格式 multi-session前 中心坐标系下01位姿轨迹 |
| 01_local_aft_intersession_loops.txt | TXT格式 multi-session后 子坐标系下01位姿轨迹  |
| 01_local_bfr_intersession_loops.txt | TXT格式 multi-session前 子坐标系下01位姿轨迹  |
| 02_central_aft_intersession_loops.txt | TXT格式 multi-session后 中心坐标系下02位姿轨迹 |
| 02_central_bfr_intersession_loops.txt | TXT格式 multi-session前 中心坐标系下02位姿轨迹 |
| 02_local_aft_intersession_loops.txt | TXT格式 multi-session后 子坐标系下02位姿轨迹 |
| 02_local_bfr_intersession_loops.txt | TXT格式 multi-session前 子坐标系下02位姿轨迹 |
| aft_map2.pcd | PCD格式  multi-session后 中心坐标系下0102拼接地图|
| aft_transformation1.pcd | PCD格式  multi-session后 中心坐标系下01地图 |
| aft_transformation2.pcd | PCD格式  multi-session后 中心坐标系下02地图 |
|  |  |

### 4.3 Object-level Updating
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam object_update.launch
```

Note that we just update the local map in similar area, so if you wanna test object-level updating, you should manually select these areas like that we show you in src/object_update.cpp line 235~321.

<img src="./pic/update.png" alt="Files generated after running object-level updating" width="350">

The upper part means that we choose the 0~50 frames with skip as 5 in 01 to update the 0~30 frames with skip as 3 in 02. Remember that you can change the skip by rewritting the "i" in for-loop. We finally get the updated map of 01.

<img src="./pic/update_details.png" alt="Files generated after running object-level updating" width="400">

| File Name | 中文解释 |
| --- | --- |
| prior_map_select.pcd | PCD格式 先验被更新地图的区域 |
| cur_map_select.pcd | PCD格式 当前更新地图的区域 |
| result.pcd | PCD格式 更新地图结果 |
|  |  |

### 4.4 Online Relocalization
```shell
source ./devel/setup.bash
roslaunch fast_lio_sam online_relocalization.launch
roslaunch fast_lio_sam mapping_*.launch
rosbag play -r * *.bag
```
You also need to first check the Config/online_relocalization.yaml, we list parameters here. "-" means that it depends on your own project.

| Parameters | 中文解释 | default(默认值) |
| --- | --- | --- |
| priorDir | 先验知识根路径 | - |
| cloudTopic | lio点云发布话题 | - |
| poseTopic | lio位姿发布话题 | - |
| searchDis | 近邻先验关键帧搜索半径(m) | 5.0 |
| searchNum | 近邻先验关键帧搜索个数 | 3 |
| trustDis | 区域覆盖搜索半径(m) | 5.0 |
| regMode | 配准方法 | 4 |
| extrinsic_T | 雷达到IMU平移外参 | - |
| extrinsic_R | 雷达到IMU旋转外参 | - |
|  |  |  |

The data structure in "priorDir" is similar to the result of lio mapping. Please do not open i-kdtree recontruction, loop closure detection or dynamic removal during online relocalization. You can set the mannual pose in rviz by button "2D Pose Estimation".

## 5 References
- Baseline: https://github.com/JS-622/YOLO-fast-lio-sam
- PGO(GPS): https://github.com/gisbi-kim/FAST_LIO_SLAM
- TGRS: https://github.com/Yixin-F/DR-Using-SCV-OD
- Multi-session: https://github.com/Yixin-F/LT-mapper_fyx


