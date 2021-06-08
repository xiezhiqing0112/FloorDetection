import os
import pandas as pd
import random
from math import ceil
import numpy as np
from tqdm import tqdm
pd.set_option('max_columns', 100)

def datatype2dir(data_type):
    if data_type == 'train':
        return '01-Training'
    elif data_type == 'validation':
        return '02-Validation'
    elif data_type == 'evaluation':
        return '03-Evaluation'
    else:
        raise NameError('不支持的类型！！！')

SENSOR_TYPE = [
            'ACCE', 'GYRO', 'MAGN', 'PRES', 'LIGH',
            'PROX', 'HUMI', 'TEMP', 'AHRS', 'GNSS',
            'WIFI', 'BLUE', 'BLE4', 'SOUN', 'RFID',
            'IMUX', 'IMUL', 'IMUI', 'POSI'

]

def get_filepath_lst(data_dir, data_type):
    file_path_lst = []
    if data_type == 'train':
        data_type_dir = os.path.join(data_dir, datatype2dir(data_type))
        for dir_name in os.listdir(data_type_dir):
            cur_dir = os.path.join(data_type_dir, dir_name)
            cur_dir_filenames = os.listdir(cur_dir)
            for filename in cur_dir_filenames:
                filepath = os.path.join(cur_dir, filename)
                file_path_lst.append(filepath)
    else:
        data_type_dir = os.path.join(data_dir, datatype2dir(data_type))
        cur_dir_filenames = os.listdir(data_type_dir)
        for filename in cur_dir_filenames:
            filepath = os.path.join(data_type_dir, filename)
            file_path_lst.append(filepath)

    return file_path_lst

def generate_csv(file_path_lst, columnpath, out_dir=None):

    for filepath in file_path_lst:
        # filepath = r'D:\\2020\\IPIN2020\\Logfiles\\02-Validation\\V01.txt'
        filedir = os.path.dirname(filepath)
        filename = os.path.basename(filepath).split('.')[0]
        all_data = {
            'ACCE': [], 'GYRO': [], 'MAGN': [], 'PRES': [], 'LIGH': [],
            'PROX': [], 'HUMI': [], 'TEMP': [], 'AHRS': [], 'GNSS': [],
            'WIFI': [], 'BLUE': [], 'BLE4': [], 'SOUN': [], 'RFID': [],
            'IMUX': [], 'IMUL': [], 'IMUI': [], 'POSI': []
        }

        with open(filepath, encoding='utf-8') as file:
            for line in file.readlines():
                if line.startswith('%') or len(line) == 1:
                    continue
                data = line.strip().split(';')
                all_data[data[0]].append(data)
        columns = {}
        with open(columnpath, encoding='utf-8') as file:
            for line in file.readlines():
                data = line.strip().split(';')
                columns[data[0]] = data
        # print(all_data)
        for key, value in all_data.items():
            pd_data = pd.DataFrame(columns=columns[key], data=value)
            # print(save)
            if out_dir is None:
                out_dir = filedir
            save_dir = os.path.join(out_dir, filename)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            save_path = os.path.join(save_dir, key+'.csv')
            pd_data.to_csv(save_path)
        print(filename + "数据转化完成！！！！")

def align_data(file_path_lst, sensor_type):
    assert sensor_type in SENSOR_TYPE

    new_file_dirs = []
    for i in range(len(file_path_lst)):
        filepath = file_path_lst[i]
        filedir = os.path.dirname(filepath)
        filename = os.path.basename(filepath).split('.')[0]
        new_dir = os.path.join(filedir, filename)
        if new_dir not in new_file_dirs:
            new_file_dirs.append(os.path.join(filedir, filename))

    for i in tqdm(range(len(new_file_dirs))):
        new_dir = new_file_dirs[i]
        # print(new_dir)
        csv_path = os.path.join(new_dir, sensor_type + '.csv')
# csv_path = r'D:\2020\IPIN2020\Logfiles\01-Training\01a-Regular\T01_01\MAGN.csv'
        data_pd = pd.read_csv(csv_path)
        interval = 0.5
        total_time = data_pd.iloc[-1:, :]['AppTimestamp(s)']
        epoch = ceil(total_time/interval)
        alig_len = 100
        for index in range(epoch):
            min_th = interval * index
            max_th = interval * (index + 1)
            # print((min_th, max_th))
            indexs = data_pd[(data_pd['AppTimestamp(s)'] > min_th) & (data_pd['AppTimestamp(s)'] <= max_th)].index
            length = len(indexs)
            if length > alig_len:
                drop_num = length - alig_len
                drop_index = np.random.choice(indexs, drop_num, replace=False)
                data_pd.drop(drop_index, inplace=True)
                data_pd.reset_index(drop=True, inplace=True)
                # print(data_pd[(data_pd['AppTimestamp(s)'] > min_th) & (data_pd['AppTimestamp(s)'] <= max_th)].index)

            elif length < alig_len:
                add_num = alig_len - length
                # print(add_num)
                add_index = random.choices(list(indexs),k=add_num)
                idx = list(data_pd.index)
                idx.extend(add_index)
                idx = sorted(idx)
                # print(len(idx))
                data_pd = data_pd.iloc[idx]
                data_pd.reset_index(drop=True, inplace=True)
                # for _index in add_index:
                #     # print(_index)
                #     data_pd1 = data_pd.loc[:_index]
                #     data_pd2 = data_pd.loc[_index+1:]
                #     data_pd = data_pd1.append(data_pd.loc[_index], ignore_index=True).append(data_pd2, ignore_index=True)
                # print(data_pd[(data_pd['AppTimestamp(s)'] > min_th) & (data_pd['AppTimestamp(s)'] <= max_th)].index)
                    # print(data_pd[(data_pd['AppTimestamp(s)'] > min_th) & (data_pd['AppTimestamp(s)'] <= max_th)])
            else:
                continue
        # print(os.path.join(filedir, filename, sensor_type + '_align.csv'))
        data_pd.to_csv(os.path.join(new_dir, sensor_type + '_align.csv'))

# acc = pd.read_csv(r'D:\2020\IPIN2020\Logfiles\01-Training\01a-Regular\T01_01\ACCE_align.csv')
# print(acc)
# gyro = pd.read_csv(r'D:\2020\IPIN2020\Logfiles\01-Training\01a-Regular\T01_01\GYRO_align.csv')
# # data_concat.cat(gyro)
# data_concat = pd.concat([acc.iloc[:, 2:], gyro.iloc[:, 2:]], axis=1)
# data_concat.to_csv(r'D:\2020\IPIN2020\Logfiles\01-Training\01a-Regular\T01_01\datac_oncat_align.csv')
# print(data_concat.head())

def sensor_data_concat(file_path_lst, sensor_type_list):
    new_file_dirs = []
    for i in range(len(file_path_lst)):
        filepath = file_path_lst[i]
        filedir = os.path.dirname(filepath)
        filename = os.path.basename(filepath).split('.')[0]
        new_dir = os.path.join(filedir, filename)
        if new_dir not in new_file_dirs:
            new_file_dirs.append(os.path.join(filedir, filename))
    for i in tqdm(range(len(new_file_dirs))):
        new_dir = new_file_dirs[i]
        pd_lst = []
        savename = ""
        for sensor_type in sensor_type_list:
            savename += sensor_type
            csv_path = os.path.join(new_dir, sensor_type + '.csv')
            pd_lst.append(pd.read_csv(csv_path).iloc[:, 2:])
        data_concat = pd.concat(pd_lst, axis=1 )
        data_concat.to_csv(os.path.join(new_dir, savename + '.csv'))

# print(data_pd.iloc(data_pd[data_pd['AppTimestamp(s)'] < 10]))
if __name__ == '__main__':
    data_dir = r'/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles'
    data_type = 'train'  # train validation evaluation
    columnpath = r'/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/column.txt'
    out_dir = r'/home/mxc/wonderland/datasets/IPIN/logfiles2020/Logfiles/processed'
    
    sensor_type_lst = ['ACCE', 'GYRO','MAGN']

    file_path_lst = get_filepath_lst(data_dir, data_type)
    generate_csv(file_path_lst, columnpath)
    for i in sensor_type_lst:
        align_data(file_path_lst, i)
    sensor_type_lst = ['ACCE', 'GYRO','MAGN']
    sensor_data_concat(file_path_lst, sensor_type_lst)


