import csv

csv_file = '/media/yixin-f/YixinF/Dataset/Mulran/river01/sensor_data/global_pose.csv'
txt_file = '/home/yixin-f/fast-lio2/src/Mulran/river01/poses.txt'

with open(csv_file, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    with open(txt_file, 'w') as txtfile:
        for row in csvreader:
            row_without_first_element = '\t'.join(row[1:])
            txtfile.write(row_without_first_element + '\n')

