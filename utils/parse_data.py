"""
将数据写入csv [rssi1, rssi2, rssi3,..., label]
"""
import json
import os
import pandas as pd

data_root = r'/media/xzq/data-2/data/crow_server/floor_data/0904J1'
save_data_root = r'/media/xzq/data-2/data/crow_server/floor_data/0904J1/json_data'
if not os.path.exists(save_data_root):
    os.makedirs(save_data_root)

data_names = ['J1B1', 'J1F1', 'J1F2', 'J1F3']
if not os.path.exists(os.path.join(save_data_root, data_names[0])):
    for data_name in data_names:
        os.makedirs(os.path.join(save_data_root, data_name))

macs = set()
macs.add('label')

# 遍历第一遍寻找所有mac
for data_name in data_names:
    data_dir = os.path.join(data_root, data_name)
    for filename in os.listdir(data_dir):
        if not filename.endswith('.txt'):
            continue
        data_path = os.path.join(data_dir, filename)
        name = filename.split('.')[0]
        save_json_path = os.path.join(save_data_root, data_name, "%s.json" % name)
        json_data = json.load(open(data_path, 'r'))
        # with open(save_json_path, 'w') as f:      # for save json path
        #     json.dump(json_data, f, indent=4)
        datas = json_data['datas']
        for data in datas:
            wifi_data = data['wifi']
            if len(wifi_data) == 0:
                continue
            for wifi_info in wifi_data:
                mac = wifi_info['mac']
                rssi = wifi_info['rssi']
                macs.add(mac)

processed_csv_path = './processed_data.csv'
processed_pd = pd.DataFrame(columns=macs)

# 遍历第二遍生成csv文件
for data_name in data_names:
    data_dir = os.path.join(data_root, data_name)
    label_name = 0 if data_name[-2] == 'B' else int(data_name[-1])
    for filename in os.listdir(data_dir):
        if not filename.endswith('.txt'):
            continue
        data_path = os.path.join(data_dir, filename)
        json_data = json.load(open(data_path, 'r'))
        datas = json_data['datas']
        for data in datas:
            wifi_data = data['wifi']
            if len(wifi_data) == 0:
                continue
            add_dict = {'label': label_name}
            for wifi_info in wifi_data:
                mac = wifi_info['mac']
                rssi = wifi_info['rssi']
                add_dict.update({mac: rssi})
            processed_pd = processed_pd.append(add_dict, ignore_index=True)

processed_pd = processed_pd.fillna(0)
processed_pd.to_csv(processed_csv_path)
