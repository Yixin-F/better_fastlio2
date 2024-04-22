import csv

file1 = '/home/yixin-f/fast-lio2/src/nclt/groundtruth_2012-02-05.csv'
file2 = '/home/yixin-f/fast-lio2/src/nclt/groundtruth_2012-02-05.txt'

with open(file1, 'r') as csv_file:
    reader = csv.reader(csv_file)
    rows = list(reader)

# 打开输出 TXT 文件并写入内容
with open(file2, 'w') as txt_file:
    for row in rows:
        # 将每行用空格连接，并在末尾添加换行符
        # row[4] = str(float(row[4]))
        # row[8] = str(float(row[8]))
        # row[12] = str(float(row[12]))
        txt_file.write(' '.join(row[1:]) + '\n')

