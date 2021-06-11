# encoding=utf-8
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn import gaussian_process


def state_step_detection(pd_data):
    """

    :param csv_path: IPIN数据处理后的CSV
    :return: pkvalue, step_length
    """
    # 初始化数据
    zhenjia = False
    FILTERING_VALUE = 0.84
    value_A = 0.0
    value_B = 0.0
    lowX = 0
    lowY = 0
    lowZ = 0
    s = 0
    sums = 0
    x = 0
    y = 0
    z = 0
    w = 0
    b = 0
    c = 0
    d = 0
    last_step = 0
    newsum = 0
    time_s2 = 0
    biaoji = 0
    newsum2 = 0
    o = 0
    num = 0
    flag = 0
    assert 'Acc_X(m/s^2)' in pd_data.columns and \
           'Acc_Y(m/s^2)' in pd_data.columns and \
           'Acc_Z(m/s^2)' in pd_data.columns, \
        "csv数据格式错误，应包含 Acc_X(m/s^2)、Acc_Y(m/s^2)、Acc_Z(m/s^2)"
    accx = pd_data['Acc_X(m/s^2)']
    accy = pd_data['Acc_Y(m/s^2)']
    accz = pd_data['Acc_Z(m/s^2)']
    # low-pass Filter
    # accx = lowX * FILTERING_VALUE + accx * (1 - FILTERING_VALUE)
    # accy = lowY * FILTERING_VALUE +accy * (1 - FILTERING_VALUE)
    # accz = lowZ * FILTERING_VALUE + accz * (1 - FILTERING_VALUE)
    acc_mean = np.sqrt(accx * accx + accy
                       * accy + accz * accz)

    acc_value = acc_mean
    station = []
    for i in range(0, len(acc_value)):
        lowX = lowX * FILTERING_VALUE + accx[i] * (1 - FILTERING_VALUE)
        lowY = lowY * FILTERING_VALUE + accy[i] * (1 - FILTERING_VALUE)
        lowZ = lowZ * FILTERING_VALUE + accz[i] * (1 - FILTERING_VALUE)
        acc_value[i] = np.sqrt(lowX * lowX + lowY * lowY + lowZ * lowZ)
        flag = 0
        value_A = acc_value[i]
        value_B = value_A - 9.8
        num = num + 1
        if value_B > 6.0 or value_B < -5.0:
            s = 0
            num = 0
        else:
            if s == 0 and flag == 0:
                flag = 1
                biaoji = 0
                num = 0
                if value_B < 0.5:
                    s = 0
                else:
                    s = 1
            else:
                if s == 1 and flag == 0:
                    time_s2 = 0
                    flag = 1
                    if value_B < 0.9 and value_B >= 0.5:
                        s = 1
                    if value_B >= 0.9:
                        s = 2
                    if value_B < 0.5:
                        s = 4
                if s == 4 and flag == 0:
                    flag = 1
                    if biaoji >= 10:
                        s = 0
                    else:
                        if value_B >= 0.5:
                            s = 1
                        else:
                            biaoji = biaoji + 1

            if s == 2 and flag == 0:
                flag = 1
                time_s2 = time_s2 + 1
                if time_s2 > 100:
                    s = 0
                else:
                    if value_B >= 0.9:
                        s = 2
                    if value_B < -0.5:
                        s = 3

            if s == 3 and flag == 0:
                flag = 1
                s = 6

            if s == 6 and flag == 0:
                flag = 1
                s = 0
                sums = sums + 1

                if sums < 4:
                    newsum = newsum + 1
                    zhenjia = False
                    if b == 0:
                        b = b + 1
                        y = x
                    else:
                        if c == 0:
                            c = c + 1
                            z = y
                        if d == 0:
                            d = d + 1
                            w = z
                else:
                    if (w - x) < 300 & (w - x) > 20:
                        newsum = newsum + 1
                        zhenjia = True
                        b = 0
                        x = w
                    else:
                        if (x - y) < 300 & (x - y) > 20:
                            newsum = newsum + 1
                            zhenjia = True
                            c = 0
                            b = 1
                            y = x
                        if (y - z) < 300 & (y - z) > 20:
                            newsum = newsum + 1
                            zhenjia = True
                            d = 0
                            c = 1
                            z = y
                        if (z - w) < 300 & (z - w) > 20:
                            newsum = newsum + 1
                            zhenjia = True
                            w = z
                            d = 1
                        else:
                            newsum = newsum + 1
                            zhenjia = False
                            sums = 1
                            b = 1
                            c = 0
                            d = 0
                            x = 40
                            y = 40
                            z = 0
                            w = 0
                if zhenjia == False:
                    o = 0
                else:
                    if (o == 0):
                        newsum2 = newsum2 + 3
                    newsum2 = newsum2 + 1
                    o = 1
        if last_step != newsum:
            station.append(i)
            last_step = newsum
    last_step_diff = np.diff(station)
    pk_values, pk_location = station, last_step_diff
    return pk_values, pk_location


if __name__ == '__main__':
    main()