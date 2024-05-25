## show 2D poses using matplotlib

# import numpy as np
# import matplotlib.pyplot as plt
# import math

# gt_poses = []
# with open('/home/yixin-f/fast-lio2/src/Mulran/river01/poses.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             gt_poses.append(pose)
# print(len(gt_poses))


# yaw_angle = math.radians(-60)
# rotation_matrix = np.array([[math.cos(yaw_angle), -math.sin(yaw_angle), 0],
#                              [math.sin(yaw_angle), math.cos(yaw_angle), 0],
#                              [0, 0, 1]])

# gt_x = []
# gt_y = []
# gt_z = []
# for it in gt_poses:
#       x = it[3] + 2268.5
#       gt_x.append(-x)
#       y = it[7] + 997.3
#       gt_y.append(y)
#       z = it[11] - 0.3
#       gt_z.append(z)
# print(len(gt_x))

# gt_xr = []
# gt_yr = []
# gt_zr = []
# rot = []
# for i in range(len(gt_x)):
#       pt = [gt_x[i], gt_y[i], gt_z[i]]
#       rot_pt = np.dot(rotation_matrix, pt)
#       gt_xr.append(rot_pt[0])
#       gt_yr.append(rot_pt[1])
#       gt_zr.append(rot_pt[2])

# lio_poses = []
# with open('/home/yixin-f/fast-lio2/src/Mulran/river01/lio.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             lio_poses.append(pose)
# print(len(lio_poses))

# lio_x = []
# lio_y = []
# lio_z = []
# for it in lio_poses:
#       x = it[3]
#       lio_x.append(x)
#       y = it[7]
#       lio_y.append(y)
#       z = it[11]
#       lio_z.append(z)
# print(len(lio_y))

# gtsam_poses = []
# with open('/home/yixin-f/fast-lio2/src/Mulran/river01/gtsam.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             gtsam_poses.append(pose)
# print(len(gtsam_poses))

# gtsam_x = []
# gtsam_y = []
# gtsam_z = []
# for it in gtsam_poses:
#       x = it[3]
#       gtsam_x.append(x)
#       y = it[7]
#       gtsam_y.append(y)
#       z = it[11]
#       gtsam_z.append(z)
# print(len(gtsam_x))

# tgrs_poses = []
# with open('/home/yixin-f/fast-lio2/src/Mulran/river01/tgrs.txt', 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#             elements = line.strip().split()
#             pose = [float(e) for e in elements]
#             tgrs_poses.append(pose)
# print(len(tgrs_poses))

# tgrs_x = []
# tgrs_y = []
# tgrs_z = []
# for it in tgrs_poses:
#       x = it[3]
#       tgrs_x.append(x)
#       y = it[7]
#       tgrs_y.append(y)
#       z = it[11]
#       tgrs_z.append(z)
# print(len(tgrs_x))

# plt.title('Factory')
# plt.xlabel('X')
# plt.ylabel('Z')
# plt.axis('equal')
# plt.grid(False)
# # print(lio_y)
# plt.plot(gt_xr, gt_yr, label= 'Ground Truth', marker='.', markersize=0.05, linestyle='--', color='k')
# plt.plot(lio_y, lio_x, label= 'FAST-LIO2', marker='.', markersize=0.05, linestyle='-', color='b')
# plt.plot(gtsam_y, gtsam_x, label= 'LIO+Sam', marker='.', markersize=0.05, linestyle='-', color='g')
# plt.plot(tgrs_y, tgrs_x, label= 'LIO+Sam/DR', marker='.', markersize=0.05, linestyle='-', color='r')

# plt.legend()
# plt.show()

import numpy as np
import matplotlib.pyplot as plt
import math

gt_poses = []
with open('/home/yixin-f/fast-lio2/src/parkinglot/02/relo_gt.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            gt_poses.append(pose)
print(len(gt_poses))

gt_x = []
gt_y = []
gt_z = []
for it in gt_poses:
      x = it[3]
      gt_x.append(x)
      y = it[7]
      gt_y.append(y)
      z = it[11]
      gt_z.append(z)
print(len(gt_x))

lio_poses = []
with open('/home/yixin-f/fast-lio2/src/seu/school/robustliorelo_pose.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            lio_poses.append(pose)
print(len(lio_poses))

lio_x = []
lio_y = []
lio_z = []
for it in lio_poses:
      x = it[3]
      lio_x.append(x)
      y = it[7]
      lio_y.append(y)
      z = it[11]
      lio_z.append(z)
print(len(lio_y))

gtsam_poses = []
with open('/home/yixin-f/fast-lio2/src/seu/school/robustliorelo_pose.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            gtsam_poses.append(pose)
print(len(gtsam_poses))

gtsam_x = []
gtsam_y = []
gtsam_z = []
for it in gtsam_poses:
      x = it[3]
      gtsam_x.append(x)
      y = it[7]
      gtsam_y.append(y)
      z = it[11]
      gtsam_z.append(z)
print(len(gtsam_x))

tgrs_poses = []
with open('/home/yixin-f/fast-lio2/src/seu/school/robustliorelo_pose.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            tgrs_poses.append(pose)
print(len(tgrs_poses))

tgrs_x = []
tgrs_y = []
tgrs_z = []
for it in tgrs_poses:
      x = it[3]
      tgrs_x.append(x)
      y = it[7]
      tgrs_y.append(y)
      z = it[11]
      tgrs_z.append(z)
print(len(tgrs_x))

tgrs_poses1 = []
with open('/home/yixin-f/fast-lio2/src/seu/school/icpliorelo_pose.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
            elements = line.strip().split()
            pose = [float(e) for e in elements]
            tgrs_poses1.append(pose)
print(len(tgrs_poses1))

tgrs_x1 = []
tgrs_y1 = []
tgrs_z1 = []
for it in tgrs_poses1:
      x = it[3]
      tgrs_x1.append(x)
      y = it[7]
      tgrs_y1.append(y)
      z = it[11]
      tgrs_z1.append(z)
print(len(tgrs_x1))

plt.title('Parkinglot')
plt.xlabel('X')
plt.ylabel('Z')
plt.axis('equal')
plt.grid(False)
# print(lio_y)
# plt.plot(gt_x, gt_y, label= 'Ground Truth', marker='.', markersize=0.05, linestyle='--', color='k')
plt.plot(lio_x, lio_y, label= 'Sparse ICP', marker='.', markersize=0.05, linestyle='-', color='b')
plt.plot(gtsam_x, gtsam_y, label= 'Fast-ICP', marker='.', markersize=0.05, linestyle='-', color='g')
plt.plot(tgrs_x, tgrs_y, label= 'Robust-ICP', marker='.', markersize=0.05, linestyle='-', color='r')
plt.plot(tgrs_x1, tgrs_y1, label= 'Ours(FR-ICP)', marker='.', markersize=0.05, linestyle='-', color='y')

plt.legend()
plt.show()
      