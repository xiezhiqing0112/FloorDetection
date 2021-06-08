import os
import pandas as pd

DIRE = '/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/03-Evaluation/'
PROCESSED_DIRE = '/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed/Training/03-Evaluation/'

if not os.path.exists(PROCESSED_DIRE): os.mkdir(PROCESSED_DIRE)
for file in os.listdir(DIRE):
	with open(DIRE + file, 'r') as fin:
		data = []
		acc, gyr, mag = None, None, None
		pos = []
		for line in fin:
			if line[0] == '%' or line == '': continue
			line = line.strip().split(';')
			if line[0] == 'ACCE': acc = list(map(float, line[1:6]))
			if line[0] == 'GYRO': gyr = list(map(float, line[1:6]))
			if line[0] == 'MAGN': mag = list(map(float, line[1:6]))
			if line[0] == 'POSI': pos.append(list(map(float, [line[1], line[3], line[4]])) + list(map(int, line[5:7])))
			if acc and gyr and mag:
				data.append([round((acc[0] + gyr[0] + mag[0]) / 3 * 1000), round((acc[1] + gyr[1] + mag[1]) / 3 * 1000)] + acc[2:] + gyr[2:] + mag[2:])
				acc = gyr = mag = None
	data = pd.DataFrame(data, columns = ['apptime', 'time', 'accx', 'accy', 'accz', 'gyrx', 'gyry', 'gyrz', 'magx', 'magy', 'magz'])
	data.to_csv(PROCESSED_DIRE + file, header = False, index = False)
	posi = pd.DataFrame(pos, columns = ['time', 'posix', 'posiy', 'floorId', 'buildingId'])
	# posi['time'] *= 1000
	posi.to_csv(PROCESSED_DIRE + 'P' + file, header = False, index = False)

	# /home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles