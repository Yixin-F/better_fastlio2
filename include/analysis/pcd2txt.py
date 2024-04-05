import open3d as o3d
import numpy as np

# 读取 PCD 文件
pcd = o3d.io.read_point_cloud("/media/yixin-f/YixinF/Dataset/Mulran/DCC01/data_lio/transformations.pcd")

# 获取点云数据
points = np.asarray(pcd.points)

# 计算yaw角逆时针旋转90度的旋转矩阵
yaw_angle = np.radians(95)
rotation_matrix = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle), 0],
                            [np.sin(yaw_angle), np.cos(yaw_angle), 0],
                            [0, 0, 1]])

# 对点云进行旋转操作
rotated_points = np.dot(rotation_matrix, points.T).T

# 将旋转后的点云数据写入文本文件
with open("/home/yixin-f/fast-lio2/src/Mulran/DCC01/lio.txt", "w") as txt_file:
    for point in rotated_points:
        txt_file.write(f"{point[0]} {point[1]} {point[2]}\n")

