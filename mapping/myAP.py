from typing import List
import numpy as np
from collections import defaultdict
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from dtaidistance import dtw, dtw_ndim
import math
from sklearn.cluster import AffinityPropagation as AP


def slide_window_preprocess(orientations: np.ndarray, start_index, window_size):
    """
    思路:
        1.前后各oritationStartIndex个方向不要，去掉
        2.滑动窗口大小为slidingWindowSize，求得平均值aveDirection
        3.第i个方向和平均值的差，差大于35，则用平均值代替

    Args:
        orientations:
        start_index:
        window_size:

    Returns:

    """
    length = orientations.shape[0]
    assert length >= start_index
    # orientations = orientations[start_index: start_index * (-1) + window_size - 1]
    # data_unfold = np.lib.stride_tricks.sliding_window_view(orientations, window_size)
    # ret_orientations = []
    # for i, window_oris in enumerate(data_unfold):
    #     avg_ori = get_avg_orientation(window_oris)
    #     if ori_diff(avg_ori, orientations[i]) <= 35:
    #         ret_orientations.append(orientations[i])
    #     else:
    #
    #         ret_orientations.append(avg_ori)
    slide_window = []
    ret_orientations = []
    for i in range(start_index, length - start_index):
        if i < start_index + window_size:
            slide_window.append(orientations[i])
            ret_orientations.append(orientations[i])
        else:
            avg_ori = get_avg_orientation(np.array(slide_window))
            if ori_diff(avg_ori, orientations[i]) <= 35:
                slide_window.append(orientations[i])
                ret_orientations.append(orientations[i])
            else:
                slide_window.append(avg_ori)
                ret_orientations.append(avg_ori)
            slide_window.pop(0)

    return ret_orientations


def ori_diff(ori1, ori2):
    """
    计算方向差
    Args:
        ori1:
        ori2:

    Returns:

    """
    if abs(ori1 - ori2) <= 180:
        diff = abs(ori1 - ori2)
    else:
        diff = 360 - abs(ori1 - ori2)
    return diff


def get_avg_orientation(orientations: np.ndarray):
    pos_mask = orientations >= 0
    positive = np.sum(orientations[pos_mask])
    negative = np.sum(orientations[~pos_mask])
    length = orientations.shape[0]
    if positive == 0:
        return negative / length
    elif negative == 0:
        return positive / length
    else:
        if positive > abs(negative):
            return (positive - negative) / length
        else:
            return (-1) * (positive - negative) / length


def preprocess_magenatic(step_list: List):
    """

    Args:
        step_list:

    Returns:

    """
    for i in range(len(step_list)):
        step_info = step_list[i]
        pitch = step_info['pitch']
        roll = step_info['roll']
        x_mag = step_info['xMagnetic']
        y_mag = step_info['yMagnetic']
        z_mag = step_info['zMagnetic']
        yaw = step_info['yaw']
        magnetics = np.array([x_mag, y_mag, z_mag]).transpose((1, 0))
        headings = np.array([pitch, roll, yaw]).transpose((1, 0))
        new_magnetics = mag2flat(magnetics, headings)
        step_list[i].update({
            'xMagnetic': new_magnetics[:, 0].tolist(),
            'yMagnetic': new_magnetics[:, 1].tolist(),
            'zMagnetic': new_magnetics[:, 2].tolist()
        })
    return step_list


def mag2flat(magnetics: np.ndarray, headings: np.ndarray):
    """
    将载体坐标系的数据转化成航向坐标系的数据
    Args:
        magnetics: (N, 3(x, y, x))
        headings: (N, 3(pitch, roll, yaw))

    Returns:    (N, 3(x, y, x))

    """
    assert magnetics.shape == headings.shape, '地磁数据和航向数据不一致'
    n = magnetics.shape[0]
    pitch = headings[:, 0] * (math.pi / 180)  # yaw
    roll = headings[:, 1] * (math.pi / 180)  # roll
    yaw = headings[:, 2] * (math.pi / 180)  # yaw
    pitch_matrix = np.zeros((n, 3, 3))
    pitch_matrix[:, 0, 0] = np.cos(pitch)
    pitch_matrix[:, 0, 2] = np.sin(pitch)
    pitch_matrix[:, 1, 1] = 1
    pitch_matrix[:, 2, 0] = (-1) * np.sin(pitch)
    pitch_matrix[:, 2, 2] = np.cos(pitch)

    roll_matrix = np.zeros((n, 3, 3))
    roll_matrix[:, 0, 0] = 1
    roll_matrix[:, 1, 1] = np.cos(roll)
    roll_matrix[:, 1, 2] = (-1) * np.sin(roll)
    roll_matrix[:, 2, 1] = np.sin(roll)
    roll_matrix[:, 2, 2] = np.cos(roll)
    ret = np.matmul(np.matmul(pitch_matrix, roll_matrix), magnetics.reshape(n, 3, 1))
    return ret.squeeze()


# 效率较低
# class AffinityPropagation:
#
#     def __init__(self, data: List, max_iter, convergence_iter, lmd):
#         self.data = data
#         length = len(data)
#         self.R = np.zeros((length, length))
#         self.A = np.zeros((length, length))
#         self.length = length
#         self.similarity = self.cal_similarity(data)
#         self.max_iter = max_iter  # 最大迭代次数
#         self.convergence_iter = convergence_iter  # 聚类中心不发生变化次数
#         self.lmd = lmd  # 阻尼系数
#
#     def cal_similarity(self, data):
#         """
#             计算相似度矩阵
#         Args:
#             data: List(json)
#
#         Returns:
#
#         """
#         raise NotImplementedError
#
#     def update_responsibility(self, R: np.ndarray, A: np.ndarray, similarity: np.ndarray):
#         # 更新R 矩阵
#         for i in range(self.length):
#             for k in range(self.length):
#                 old_r = R[i][k]
#                 if i != k:
#                     temp = A[i][0] + R[i][0]  # 存储最大值
#                     for j in range(self.length):
#                         if j != k:
#                             if A[i][j] + R[i][j] > temp:
#                                 temp = A[i][j] + R[i][j]
#                     max_value = temp
#                     R[i][k] = similarity[i][k] - max_value
#
#                 else:
#                     temp = similarity[i][0]
#                     for j in range(self.length):
#                         if j != k:
#                             if similarity[i][j] > temp:
#                                 temp = similarity[i][j]
#                     max_value = temp
#                     R[i][k] = similarity[i][k] - max_value
#
#                 R[i][k] = (1 - self.lmd) * R[i][k] + self.lmd * old_r  # 根据阻尼系数进行衰减
#         return R
#
#     def update_availability(self, R: np.ndarray, A: np.ndarray):
#         for i in range(self.length):
#             for k in range(self.length):
#                 old_a = A[i][k]  # 之后阻尼更新时使用
#                 if i == k:
#                     temp = R[0][k]
#                     for j in range(self.length):
#                         if j != k:
#                             if R[j][k] > 0:
#                                 temp += R[j][k]
#                             else:
#                                 temp += 0
#                     max_value = temp
#                     A[i][k] = max_value
#
#                 else:
#                     temp = R[0][k]
#                     for j in range(self.length):
#                         if j != k and j != i:
#                             if R[j][k] > 0:
#                                 temp += R[j][k]
#                             else:
#                                 temp += 0
#                     max_value = temp
#                     if R[k][k] + max_value > 0:
#                         A[i][k] = 0
#                     else:
#                         A[i][k] = R[k][k] + max_value
#
#                 A[i][k] = (1 - self.lmd) * A[i][k] + self.lmd * old_a  # 根据阻尼系数进行衰减
#         return A
#
#     def fit(self):
#         cur_iter = 0
#         cur_convergence_iter = 0
#         center_inds = []  # 质心
#         while cur_convergence_iter < self.convergence_iter and cur_iter < self.max_iter:
#             self.R = self.update_responsibility(self.R, self.A, self.similarity)
#             self.A = self.update_availability(self.R, self.A)
#             for k in range(self.length):
#                 if self.R[k][k] + self.A[k][k] > 0:  # 当R[k][k] + A[k][k] > 0, 被认为是一个聚类中心
#                     if k not in center_inds:
#                         center_inds.append(k)
#                     else:
#                         cur_convergence_iter += 1
#             cur_iter += 1
#         R_plus_A = self.R + self.A
#         records = defaultdict(list)
#         for i in range(self.length):
#             loc = 0
#             max_value = -np.inf
#             for j in range(self.length):
#                 if max_value < R_plus_A[i][j]:
#                     max_value = R_plus_A[i][j]
#                     loc = j
#             records[loc].append(i)
#         print("聚类数目为 %d" % records.__len__())
#         ret_info = []  # 类别数 * 每个类别的数据数 * 每条轨迹的json_data
#         for label, label_ids in records.items():
#             temp = []
#             for i in label_ids:
#                 temp.append(self.data[i])  # 同一类的所有原子轨迹数据
#             ret_info.append(temp)
#         return center_inds, ret_info

class AffinityPropagation:
    def __init__(self, data: List, max_iter, convergence_iter, lmd):
        self.func = AP(max_iter=max_iter, convergence_iter=convergence_iter, affinity='precomputed', random_state=2021)
        self.max_iter = max_iter
        self.convergence_iter = convergence_iter
        self.lmd = lmd
        self.length = len(data)
        self.similarity = self.cal_similarity(data)

    def cal_similarity(self, data):
        raise NotImplementedError

    def fit(self):
        ap = self.func.fit(self.similarity)
        return ap.cluster_centers_indices_, ap.labels_


class APForDirection(AffinityPropagation):
    def cal_similarity(self, data):
        # 方向数据预处理
        for i in range(self.length):
            atom_trajetory = data[i]
            orientation = atom_trajetory['orientation']
            orientation = slide_window_preprocess(np.array(orientation), 100, 15)
            data[i].update({
                'orientation': orientation
            })
        similarity = np.zeros((self.length, self.length))
        for i in range(self.length):
            i_orientation = get_avg_orientation(np.array(data[i]['orientation']))
            for j in range(i + 1, self.length):
                j_orientation = get_avg_orientation(np.array(data[j]['orientation']))
                ori_diff = abs(i_orientation - j_orientation)
                store_sim = 0
                if ori_diff < 180:
                    store_sim = ori_diff
                else:
                    store_sim = 360 - ori_diff
                similarity[i][j] = similarity[j][i] = store_sim * (-1)
        mid = np.median(similarity)  # 中心参考度 取中值
        for i in range(self.length):
            similarity[i][i] = mid
        return similarity


class APForMagenatic(AffinityPropagation):

    def cal_similarity(self, data):

        magnetic_samples = []  # 原子轨迹数 * 总的地磁信号(x, y, z)
        for i in range(self.length):
            atom_trajetory = data[i]
            step_list = atom_trajetory['stepList']
            x_mag, y_mag, z_mag = [], [], []
            step_list = preprocess_magenatic(step_list)
            for j in range(len(step_list)):  # 每一步
                step_info = step_list[j]
                x_mag.extend(step_info['xMagnetic'])
                y_mag.extend(step_info['yMagnetic'])
                z_mag.extend(step_info['zMagnetic'])
            samples = [x_mag, y_mag, z_mag]
            magnetic_samples.append(np.array(samples).transpose((1, 0)))  # 转置 可能需要
        start = time.time()
        similarity = np.zeros((self.length, self.length))
        for i in range(self.length):
            for j in range(i + 1, self.length):
                # distance, _ = fastdtw(magnetic_samples[i], magnetic_samples[j], dist=euclidean)
                distance = dtw_ndim.distance_fast(magnetic_samples[i], magnetic_samples[j])
                similarity[j][i] = similarity[i][j] = (-1) * distance
        print("计算相似度耗时 ： {}".format(time.time() - start))
        mid = np.median(similarity)  # 中心参考度 取中值
        for i in range(self.length):
            similarity[i][i] = mid
        return similarity


if __name__ == '__main__':
    import json
    import os
    import time

    root_dir = r'/media/xzq/data-2/data/crow_server/python_module/mate8'
    max_iter = 500
    convergence_iter = 50
    lmd = 0.5
    data = []
    for file_name in os.listdir(root_dir):
        json_data = json.load(open(os.path.join(root_dir, file_name)))
        data.append(json_data)
    print("方向聚类开始.........")
    start = time.time()
    ap_ori = APForDirection(data, max_iter=max_iter, convergence_iter=convergence_iter, lmd=lmd)
    ori_cluster_centers_indices_, ori_labels_ = ap_ori.fit()
    print("方向聚类结束.........   \n耗时 ： {}" .format(time.time() - start))
    class_num = ori_cluster_centers_indices_.shape[0]
    count = 0       # 地磁聚类后总的类别数
    cluster_result = {}
    for label in range(class_num):
        index = np.where(ori_labels_ == label)[0]
        one_ori_data = [data[i] for i in index]
        start = time.time()
        print("地磁聚类开始..........")
        ap_mag = APForMagenatic(one_ori_data, max_iter=max_iter, convergence_iter=convergence_iter, lmd=lmd)
        mag_cluster_centers_indices_, mag_labels_ = ap_mag.fit()
        print("地磁聚类结束.........   \n耗时 ： {}".format(time.time() - start))
        for l in range(len(mag_cluster_centers_indices_)):
            one_mag_data = [one_ori_data[i] for i in np.where(mag_labels_ == l)[0]]
            cluster_result.update({
                count: one_mag_data
            })
            count += 1
        # print(mag_cluster_centers_indices_)
        # print(mag_labels_)
    print("最终聚类个数： %d" % count)
    with open('./cluster_result.json', 'w') as f:
        json.dump(cluster_result, f, indent=4)

