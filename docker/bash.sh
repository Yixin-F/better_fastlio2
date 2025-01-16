#!/bin/bash

# 允许 Docker 访问 X11 显示
xhost +local:docker

# 运行 Docker 容器
docker run -it \
    --gpus all \
    --privileged \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /home/ian/catkin_ws:/workspace/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/video0:/dev/video0 \
    -v /dev/video1:/dev/video1 \
    -v /dev/snd:/dev/snd \
    --device=/dev/dri:/dev/dri \
    --name="better-fast-lio2"
    jonathonyin/better-fast-lio2:0.1.0
