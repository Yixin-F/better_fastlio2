# FAST_LIO_SAM

## Object detect : YOLO 
## Front_end : fastlio2
## Back_end : lio_sam

<p align='center'>
    <img src="/pic/1.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/2.png" alt="drawing" width="400" height =200/>
</p>
<p align='center'>
    <img src="/pic/3.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/4.png" alt="drawing" width="400" height =200/>
</p>

## Related worked 

1.[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)为紧耦合的lio slam系统，因其缺乏前端，所以缺少全局一致性，参考lio_sam的后端部分，接入GTSAM进行后端优化。

2.[FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)的作者kim在FAST-LIO2的基础上，添加SC-PGO模块，通过加入ScanContext全局描述子，进行回环修正,SC-PGO模块与FAST-LIO2解耦，非常方便，很优秀的工作。

3.[darknet_ros](https://github.com/leggedrobotics/darknet_ros)为YOLO系列的部分网络提供了便捷的ROS接口。

## Prerequisites

- Ubuntu 20.04 and ROS Noetic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- GTSAM >= 4.0.0(tested on 4.0.0-alpha2)

## Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/JS-622/YOLO-fast-lio-sam.git
cd ..
catkin_make
```

## Results show

#### For outdoor dataset

场景图片
![my](/pic/pic1.png)

未进行检测去除的运行效果
<p align='center'>
    <img src="/pic/no1.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/no2.png" alt="drawing" width="400" height =200/>
</p>

使用单目相机进行赋色后的运行效果
<p align='center'>
    <img src="/pic/color2.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/color3.png" alt="drawing" width="400" height =200/>
</p>
<p align='center'>
    <img src="/pic/color4.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/color5.png" alt="drawing" width="400" height =200/>
</p>

使用YOLO进行目标检测的过程
![my](/pic/det1.png)

检测结果对点云的投影
<p align='center'>
    <img src="/pic/p1.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/p2.png" alt="drawing" width="400" height =200/>
</p>
<p align='center'>
    <img src="/pic/p3.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/p4.png" alt="drawing" width="400" height =200/>
</p>

最终的建图效果(此处展示内容为去除了车辆)
<p align='center'>
    <img src="/pic/re1.png" alt="drawing" width="400" height ="200"/>
    <img src="/pic/re2.png" alt="drawing" width="400" height =200/>
</p>


## More

