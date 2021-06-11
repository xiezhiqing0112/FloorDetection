"""
从 小米9 中采集的数据进行解析

先获取所有mac, 再构建数据写入csv
"""
import pandas as pd
import numpy as np
import os


macs = set()
macs.add('label')
root_dir = r'/media/xzq/data-2/data/crow_server/floor_data/ict'
processed_csv_path = './processed_data.csv'
for floor_name in os.listdir(root_dir):
    csv_path = os.path.join(root_dir, floor_name, 'network.csv')
    pd_data = pd.read_csv(csv_path)
    wifi_info = pd_data['wifi']
    for line in wifi_info.values:
        wifi_num, macs_and_rssis = line.split(' ')[0], line.split(" ")[1:]
        for mac_and_rssi in macs_and_rssis:
            mac, rssi = mac_and_rssi.split(';')
            macs.add(mac)

processed_pd = pd.DataFrame(columns=macs)

for floor_name in os.listdir(root_dir):
    csv_path = os.path.join(root_dir, floor_name, 'network.csv')
    label = floor_name.split('_')[-1]
    pd_data = pd.read_csv(csv_path)
    wifi_info = pd_data['wifi']
    for line in wifi_info.values:
        wifi_num, macs_and_rssis = line.split(' ')[0], line.split(" ")[1:]
        add_dict = {'label': int(label)}
        for mac_and_rssi in macs_and_rssis:
            mac, rssi = mac_and_rssi.split(';')
            add_dict.update({mac: int(rssi)})
        processed_pd = processed_pd.append(pd.Series(add_dict), ignore_index=True)

processed_pd = processed_pd.fillna(0)
processed_pd.to_csv(processed_csv_path)