# 使用 NVIDIA 提供的 CUDA 基础镜像，Ubuntu 20.04
FROM nvidia/cuda:11.7.1-runtime-ubuntu20.04

# 设置非交互模式以避免安装过程中的交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装基础工具和 ROS Noetic 桌面版
RUN apt-get update && \
    apt-get install -y \
        curl \
        lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get update && \
    apt-get install -y \
        python3 \
        python3-pip \
        python3-rosdep \
        ros-noetic-desktop-full \
        libgl1-mesa-glx \
        libegl1-mesa \
        mesa-utils \
        build-essential \
        cmake \
        git \
        wget && \
    rm -rf /var/lib/apt/lists/*

# 初始化 ROS 和相关依赖
RUN rosdep init && rosdep update

# 设置 ROS 环境变量
RUN echo "source /opt/ros/noetic/setup.bash" > /etc/profile.d/ros_noetic.sh && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 安装 RViz 和其他工具依赖
RUN apt-get update && \
    apt-get install -y \
        ros-noetic-rviz && \
    rm -rf /var/lib/apt/lists/*

# 安装 Python 的 ROS 工具
RUN pip3 install --no-cache-dir \
    catkin_tools \
    rospkg \
    empy

# 设置工作目录为 /workspace/catkin_ws
WORKDIR /workspace/catkin_ws

# 添加 entrypoint 脚本，用于加载环境
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 使用 entrypoint 脚本启动容器
ENTRYPOINT ["/entrypoint.sh"]

# 默认命令进入 bash
CMD ["/bin/bash"]

