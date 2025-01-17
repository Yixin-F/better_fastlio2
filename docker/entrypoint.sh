#!/bin/bash

# 加载 ROS 环境
source /opt/ros/noetic/setup.bash

# 如果存在 catkin_ws 工作空间，加载其环境
if [ -f "/workspace/catkin_ws/devel/setup.bash" ]; then
    source /workspace/catkin_ws/devel/setup.bash
fi

# 导出额外的环境变量
export LIBGL_ALWAYS_INDIRECT=1
export MESA_GL_VERSION_OVERRIDE=4.5
export MESA_GLSL_VERSION_OVERRIDE=450
export LIBGL_DEBUG=verbose
export QT_X11_NO_MITSHM=1
export DISPLAY=${DISPLAY:-:0}

# 打印当前环境变量（可选，用于调试）
echo "ROS environment sourced. Starting with arguments: $@"
echo "LIBGL_ALWAYS_INDIRECT=$LIBGL_ALWAYS_INDIRECT"
echo "MESA_GL_VERSION_OVERRIDE=$MESA_GL_VERSION_OVERRIDE"
echo "MESA_GLSL_VERSION_OVERRIDE=$MESA_GLSL_VERSION_OVERRIDE"
echo "LIBGL_DEBUG=$LIBGL_DEBUG"
echo "QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM"
echo "DISPLAY=$DISPLAY"

# 执行传入的命令
exec "$@"
