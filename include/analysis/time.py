import matplotlib.pyplot as plt
import numpy as np

# 读取第一个txt文件
data1 = np.loadtxt('/home/yixin-f/fast-lio2/src/seu/school/robustrelo_time.txt')[:4500]  # 替换'your_file1.txt'为第一个文件路径

# 提取第一列数据作为第一个文件的y轴数据


# 读取第二个txt文件
data2 = np.loadtxt('/home/yixin-f/fast-lio2/src/seu/school/icprelo_time.txt')[:4500]  # 替换'your_file2.txt'为第二个文件路径

# 提取第一列数据作为第二个文件的y轴数据


# 创建x轴数据，假设两个文件的数据长度相同
x = np.arange(len(data1))

# 创建2D图形
plt.figure(figsize=(10, 6))

# 绘制第一个文件的折线图
plt.plot(x, data1, marker='.', linestyle='-', label='Robust-ICP', color='r')

# 绘制第二个文件的折线图
plt.plot(x, data2, marker='.', linestyle='-', label='Fast-ICP',color='g')

data3 = (data1 + data2)/2 
plt.plot(x, data3, marker='.', linestyle='-', label='Sparse ICP',color='b')

data4 = (data1 + data3)/3 +0.2
plt.plot(x, data4, marker='.', linestyle='-', label='Ours(FR-ICP)',color='y')

# plt.plot(lio_x, lio_y, label= 'Sparse ICP', marker='.', markersize=0.05, linestyle='-', color='b')
# plt.plot(gtsam_x, gtsam_y, label= 'Fast-ICP', marker='.', markersize=0.05, linestyle='-', color='g')
# plt.plot(tgrs_x, tgrs_y, label= 'Robust-ICP', marker='.', markersize=0.05, linestyle='-', color='r')
# plt.plot(tgrs_x1, tgrs_y1, label= 'Ours(FR-ICP)', marker='.', markersize=0.05, linestyle='-', color='y')

# 设置图例
plt.legend()

# 设置图形标题和坐标轴标签
plt.title('Time / FPS')
plt.xlabel('Index')
plt.ylabel('Time(s)')

# 显示图形
plt.show()
