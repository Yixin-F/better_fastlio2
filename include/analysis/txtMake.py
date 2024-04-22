# 定义输入和输出文件路径
input_file = '/home/yixin-f/fast-lio2/src/Mulran/river01/tgrs.txt'
output_file = '/home/yixin-f/fast-lio2/src/Mulran/river01/tgrs1.txt'

# 打开输入文件并读取前 100 行内容
with open(input_file, 'r') as f:
    lines = f.readlines()[:2385]

# 打开输出文件并写入内容
with open(output_file, 'w') as f:
    f.writelines(lines)