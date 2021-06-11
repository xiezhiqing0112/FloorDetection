import numpy as np
import os
import json


def get_rotation_matrix(gravity: np.ndarray, geomagnetic: np.ndarray):
    """
        返回：
            R：从手机设备的坐标系到真实世界坐标系的转换矩阵
            I： 将地磁数据转换到重力数据所在坐标的旋转矩阵
    :param gravity: (N, 3)
    :param geomagnetic: (N, 3)
    :return: R, I
    """
    assert gravity.shape == geomagnetic.shape
    Ax = gravity[:, 0]
    Ay = gravity[:, 1]
    Az = gravity[:, 2]
    normsqA = Ax ** 2 + Ay ** 2 + Az ** 2
    g = 9.81
    freeFallGravitySquared = 0.01 * (g ** 2)
    assert normsqA.any() >= freeFallGravitySquared

    Ex = geomagnetic[:, 0]
    Ey = geomagnetic[:, 1]
    Ez = geomagnetic[:, 2]
    Hx = Ey * Az - Ez * Ay
    Hy = Ez * Ax - Ex * Az
    Hz = Ex * Ay - Ey * Ax

    normH = np.sqrt(Hx ** 2 + Hy ** 2 + Hz ** 2)

    assert normH.any() >= 0.1

    invH = 1.0 / normH
    Hx = Hx * invH
    Hy = Hy * invH
    Hz = Hz * invH

    invA = 1.0 / np.sqrt(Ax ** 2 + Ay ** 2 + Az ** 2)
    Ax = Ax * invA
    Ay = Ay * invA
    Az = Az * invA

    Mx = Ay * Hz - Az * Hy
    My = Az * Hx - Ax * Hz
    Mz = Ax * Hy - Ay * Hx

    R = np.zeros((gravity.shape[0], 3, 3), dtype=np.float32)

    R[:, 0, 0] = Hx
    R[:, 0, 1] = Hy
    R[:, 0, 2] = Hz
    R[:, 1, 0] = Mx
    R[:, 1, 1] = My
    R[:, 1, 2] = Mz
    R[:, 2, 0] = Ax
    R[:, 2, 1] = Ay
    R[:, 2, 2] = Az

    I = np.zeros((gravity.shape[0], 3, 3), dtype=np.float32)
    invE = 1.0 / np.sqrt(Ex ** 2 + Ey ** 2 + Ez ** 2)
    c = (Ex * Mx + Ey * My + Ez * Mz) * invE
    s = (Ex * Ax + Ey * Ay + Ez * Az) * invE
    I[:, 0, 0] = 1
    I[:, 0, 1] = 0
    I[:, 0, 2] = 0
    I[:, 1, 0] = 0
    I[:, 1, 1] = c
    I[:, 1, 2] = s
    I[:, 2, 0] = 0
    I[:, 2, 1] = -s
    I[:, 2, 2] = c

    return R, I


def get_orientation(R: np.ndarray):
    """

    :param R: (N, 3, 3)
    :return:
    """
    values = np.zeros((R.shape[0], 3), dtype=np.float32)
    values[:, 0] = np.arctan2(R[:, 0, 1], R[:, 1, 1])
    values[:, 1] = np.arcsin(-R[:, 2, 1])
    values[:, 2] = np.arctan2(-R[:, 2, 0], R[:, 2, 2])
    return values


def to_degree(angrad: np.ndarray):
    return angrad * 180.0 / np.pi


# savePath =/media/xzq/data-2/data/crow_server/magneticData/sampleData/mate8/
def get_data(sample_data_dir):
    total_trajectorys = []
    for file_name in os.listdir(sample_data_dir):
        if os.path.isdir(file_name):
            continue
        trajectory_info = json.load(open(os.path.join(sample_data_dir, file_name)))
        total_trajectorys.append(trajectory_info)
    return total_trajectorys


if __name__ == '__main__':
    gravity = np.random.random((100, 3))
    geomagnetic = np.random.random((100, 3))

    R, I = get_rotation_matrix(gravity, geomagnetic)
    print(R, I)
