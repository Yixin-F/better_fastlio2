## show 3D poses using matplotlib

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np

# # 创建3D图形
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # 为每个文件选择不同的颜色，并读取并绘制6个txt文件的数据
# colors = ['r', 'g', 'b', 'c', 'm', 'y']
# for i, color in enumerate(colors):
#     # 读取txt文件
#     data = np.loadtxt(f'/home/yixin-f/fast-lio2/src/parkinglot/{i+1}.txt')  # 替换your_file_{i}.txt为你的文件路径
#     # 每100行取前三列数据，并将第三列元素设置为0
#     data_subset = data[::1, :12]
#     # data_subset[:, 2] = 0

#     # 随机旋转前两列元素
#     rotation_angle = np.random.rand() * 90  # 随机生成一个旋转角度
#     rotation_matrix = np.array([[np.cos(np.radians(rotation_angle)), -np.sin(np.radians(rotation_angle))],
#                                 [np.sin(np.radians(rotation_angle)), np.cos(np.radians(rotation_angle))]])
#     data_rotated = np.dot(data_subset, rotation_matrix.T)
#     # 提取旋转后的前两列数据
#     x = data_rotated[:, 3]
#     y = data_rotated[:, 7]

#     # # 提取前三列数据
#     # x = data_subset[:, 0]
#     # y = data_subset[:, 1]
#     # z = data_subset[:, 2] + 10 * i
#     z = 10 * i
#     # 绘制散点图
#     ax.scatter(x, y, z, c=color, marker = ".", linewidths = 0.1, label=f'NCLT seq.{i+1}')

# # 设置图例
# ax.legend()

# # 设置图形标题和坐标轴标签
# # ax.set_title('Single Session')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # 显示图形
# plt.show()

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 创建3D图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 为每个文件选择不同的颜色，并读取并绘制6个txt文件的数据
colors = ['r', 'g', 'b', 'c', 'm', 'y']
for i, color in enumerate(colors):
    # 读取txt文件
    data = np.loadtxt(f'/home/yixin-f/fast-lio2/src/parkinglot/{i+1}.txt')  # 替换your_file_{i}.txt为你的文件路径
    # 每100行取前三列数据，并将第三列元素设置为0
    data_subset = data[::1, :12]
    data_subset[:, 2] = 0

    # 提取前三列数据
    x = data_subset[:, 3]
    y = data_subset[:, 7]
    z = 10 * i
    # 绘制散点图
    ax.scatter(x, y, z, c=color, marker = ".", linewidths = 0.1, label=f'Parkinglot seq.{i+1}')

# 设置图例
ax.legend()

# 设置图形标题和坐标轴标签
# ax.set_title('Single Session')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()

# import matplotlib.pyplot as plt
# import numpy as np

# # 创建颜色列表
# colors = ['r', 'g', 'b', 'c', 'm', 'y']

# # 创建一个图形
# plt.figure(figsize=(10, 6))

# # 循环处理每个txt文件
# for i in range(6):
#     # 读取txt文件
#     data = np.loadtxt(f'/home/yixin-f/fast-lio2/src/parkinglot/{i+1}.txt')  # 替换your_file_{i}.txt为你的文件路径
#     # 每500行取前两列数据
#     data_subset = data[::1, :12]
#     # 提取前两列数据
#     x = data_subset[:, 3]
#     y = data_subset[:, 7]
#     # 绘制散点图
#     plt.scatter(x, y, c=colors[i], marker = '.', linewidths = 0.1, label=f'Parkinglot seq.{i+1}')

# # 设置图形标题和坐标轴标签
# # plt.title('Combined 2D Scatter Plot')
# plt.xlabel('X')
# plt.ylabel('Y')

# # 添加图例
# plt.legend()

# # 显示图形
# plt.show()

