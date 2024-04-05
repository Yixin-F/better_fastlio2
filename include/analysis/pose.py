import numpy as np
import matplotlib.pyplot as plt

gt_poses = []
with open('/home/yixin-f/fast-lio2/src/Mulran/river01/poses.txt', 'r') as f:
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
      gt_x.append(x - 355630)
      y = it[7]
      gt_y.append(y - 4026791)
      z = it[11]
      gt_z.append(z - 19)
print(len(gt_x))

lio_poses = []
with open('/home/yixin-f/fast-lio2/src/Mulran/river01/lio.txt', 'r') as f:
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

# gtsam_poses = []
# with open('/home/yixin-f/fast-lio2/src/Mulran/DCC01/gtsam.txt', 'r') as f:
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
# with open('/home/yixin-f/fast-lio2/src/Mulran/DCC01/tgrs.txt', 'r') as f:
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

plt.title('KITTI 10')
plt.xlabel('X')
plt.ylabel('Z')
plt.axis('equal')
plt.grid(False)
# print(lio_y)
plt.plot(gt_x, gt_y, label= 'Ground Truth', marker='.', markersize=0.05, linestyle='--', color='k')
plt.plot(lio_x, lio_y, label= 'FAST-LIO2', marker='.', markersize=0.05, linestyle='-', color='b')
# plt.plot(gtsam_x, gtsam_y, label= 'LIO+Sam', marker='.', markersize=0.05, linestyle='-', color='g')
# plt.plot(tgrs_x, tgrs_y, label= 'LIO+Sam/DR', marker='.', markersize=0.05, linestyle='-', color='r')

plt.legend()
plt.show()
      