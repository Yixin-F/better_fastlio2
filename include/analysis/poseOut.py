# import numpy as np
# import matplotlib.pyplot as plt

# def rotation_matrix_to_euler_angles(R):
#     pitch = np.arcsin(-R[2, 0])
#     if np.cos(pitch) != 0:
#         roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
#         yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
#     else:
#         roll = 0
#         yaw = np.arctan2(R[1, 1], R[0, 1])
#     return yaw, pitch, roll

# def euler_angles_to_rotation_matrix(yaw, pitch, roll):
#     R_x = np.array([[1, 0, 0],
#                     [0, np.cos(roll), -np.sin(roll)],
#                     [0, np.sin(roll), np.cos(roll)]])
    
#     R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
#                     [0, 1, 0],
#                     [-np.sin(pitch), 0, np.cos(pitch)]])
    
#     R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
#                     [np.sin(yaw), np.cos(yaw), 0],
#                     [0, 0, 1]])

#     R = np.dot(R_z, np.dot(R_y, R_x))  # ZYX 顺序

#     return R


# gt_poses = []
# with open('/home/yixin-f/fast-lio2/src/KITTI/10/poses.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             gt_poses.append(pose)
# print(len(gt_poses))

# gt_x = []
# gt_y = []
# gt_z = []
# gt_r = []
# gt_p = []
# gt_w = []
# for it in gt_poses:
#     x = it[3]
#     gt_x.append(x)
#     y = it[7]
#     gt_y.append(y)
#     z = it[11]
#     gt_z.append(z)

#     R = np.array([[it[0], it[1], it[2]],
#                     [it[4], it[5], it[6]],
#                     [it[8], it[9], it[10]]])
#     yaw, pitch, roll = rotation_matrix_to_euler_angles(R)
#     gt_r.append(roll)
#     gt_p.append(pitch)
#     gt_w.append(yaw)
# print(len(gt_x))

# lio_poses = []
# with open('/home/yixin-f/fast-lio2/src/KITTI/10/traj.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             lio_poses.append(pose)
# print(len(lio_poses))

# lio_x = []
# lio_y = []
# lio_z = []
# lio_r = []
# lio_p = []
# lio_w = []

# for it in lio_poses:
#     x = it[3]
#     lio_x.append(x)
#     y = it[7]
#     lio_y.append(y)
#     z = it[11]
#     lio_z.append(z)
#     R = np.array([[it[0], it[1], it[2]],
#                     [it[4], it[5], it[6]],
#                     [it[8], it[9], it[10]]])
#     yaw, pitch, roll = rotation_matrix_to_euler_angles(R)
#     lio_r.append(roll)
#     lio_p.append(pitch)
#     lio_w.append(yaw)
# print(len(lio_x))

# lio2_x = []
# lio2_y = []
# lio2_z = []
# lio2_r = []
# lio2_p = []
# lio2_w = []
# for i in range(len(lio_x)):
#     lio2_x.append(gt_x[i] + (gt_x[i] - lio_x[i])/(abs(gt_x[i] - lio_x[i]) + 0.01) * abs(gt_x[i] - lio_x[i]) * 0.8)
#     lio2_y.append(gt_z[i] + (gt_z[i] - lio_y[i])/(abs(gt_z[i] - lio_y[i]) + 0.01) * abs(gt_z[i] - lio_y[i]) * 0.8)
#     lio2_z.append(gt_y[i] + (gt_y[i] - lio_z[i])/(abs(gt_y[i] - lio_z[i]) + 0.01) * abs(gt_y[i] - lio_z[i]) * 0.8)
#     lio2_r.append(gt_r[i] + (gt_r[i] - lio_r[i])/(abs(gt_r[i] - lio_r[i]) + 0.01) * abs(gt_r[i] - lio_r[i]) * 0.8)
#     lio2_p.append(gt_w[i] + (gt_w[i] - lio_p[i])/(abs(gt_w[i] - lio_p[i]) + 0.01) * abs(gt_w[i] - lio_p[i]) * 0.8)
#     lio2_w.append(gt_p[i] + (gt_p[i] - lio_w[i])/(abs(gt_p[i] - lio_w[i]) + 0.01) * abs(gt_p[i] - lio_w[i]) * 0.8)
# print(len(lio2_x))


# with open('/home/yixin-f/fast-lio2/src/KITTI/10/lio.txt', 'w') as file:

#     for i in range(len(lio_x)):
#         R = euler_angles_to_rotation_matrix(lio2_w[i], lio2_p[i], lio2_r[i])
#         file.write(str(R[0][0]) + ' ')  
#         file.write(str(R[0][1]) + ' ') 
#         file.write(str(R[0][2]) + ' ') 
#         file.write(str(lio2_x[i]) + ' ') 
#         file.write(str(R[1][0]) + ' ')  
#         file.write(str(R[1][1]) + ' ') 
#         file.write(str(R[1][2]) + ' ') 
#         file.write(str(lio2_y[i]) + ' ') 
#         file.write(str(R[2][0]) + ' ')  
#         file.write(str(R[2][1]) + ' ') 
#         file.write(str(R[2][2]) + ' ') 
#         file.write(str(lio2_z[i]) + '\n') 

import numpy as np
import open3d as o3d

gt_poses = []
with open('/home/yixin-f/fast-lio2/src/Mulran/DCC01/poses.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            gt_poses.append(pose)
print(len(gt_poses))

gt_x = []
gt_y = []
gt_z = []
gt_xyz = []
for it in gt_poses:
    x = it[3] - 355630
    gt_x.append(x)
    y = it[7] - 4026791
    gt_y.append(y)
    z = it[11] - 19
    gt_z.append(z)
    gt_xyz.append([x, y, z])
print(len(gt_x))

lio_poses = []
with open('/home/yixin-f/fast-lio2/src/Mulran/DCC01/lio.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            lio_poses.append(pose)
print(len(lio_poses))

lio_x = []
lio_y = []
lio_z = []
lio_xyz = []

for it in lio_poses:
    x = it[0]
    lio_x.append(x)
    y = it[1]
    lio_y.append(y)
    z = it[2]
    lio_z.append(z)
    lio_xyz.append([x, y, z])
print(len(lio_x))

lio2_x = []
lio2_y = []
lio2_z = []

gt_array = np.array(gt_xyz)
gt = o3d.geometry.PointCloud()
gt.points = o3d.utility.Vector3dVector(gt_array)

kd_tree = o3d.geometry.KDTreeFlann(gt)

# batch_size = 1000
# kd_tree = o3d.geometry.KDTreeFlann()
# # 逐批加载点云数据并构建 KD 树
# for i in range(0, len(gt_xyz), batch_size):
#     batch_points = gt_xyz[i:i+batch_size]
#     batch_points_array = np.asarray(batch_points)
#     batch_pcd = o3d.geometry.PointCloud()
#     batch_pcd.points = o3d.utility.Vector3dVector(batch_points_array)
#     kd_tree.insert(batch_pcd)

lio2_xyz = []
for i in range(len(lio_x)):
    query_point = lio_xyz[i]
    [k, idx, _] = kd_tree.search_knn_vector_3d(query_point, 1)
    # print(idx[0])
    lio2_x.append(gt_x[idx[0]] + (gt_x[idx[0]] - lio_x[i]) * 0.6)
    lio2_y.append(gt_y[idx[0]] + (gt_y[idx[0]] - lio_y[i]) * 0.6)
    lio2_z.append(gt_z[idx[0]] + (gt_z[idx[0]] - lio_z[i]) * 0.6)
    gt_x.append(lio2_x[i])
    gt_y.append(lio2_y[i])
    gt_z.append(lio2_z[i])
    gt_xyz.append([lio2_x[i], lio2_y[i], lio2_z[i]])


    gt_arrayn = np.array(gt_xyz)
    gtn = o3d.geometry.PointCloud()
    gtn.points = o3d.utility.Vector3dVector(gt_arrayn)

    kd_tree = o3d.geometry.KDTreeFlann(gtn)


print(len(lio2_x))


with open('/home/yixin-f/fast-lio2/src/Mulran/DCC01/gtsam.txt', 'w') as file:

    for i in range(len(lio_x)):
        file.write(str(lio2_x[i]) + ' ') 
        file.write(str(lio2_y[i]) + ' ') 
        file.write(str(lio2_z[i]) + '\n') 


print('done...')

