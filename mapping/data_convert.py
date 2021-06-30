import pandas as pd
import numpy as np
from mapping.stateStepDetection import state_step_detection
from mapping.turnDetection import turn_detection
from mapping.utils import get_rotation_matrix, get_orientation, to_degree

turn_threshold = 65


def get_step_list(pd_data: pd.DataFrame, save_dir, base_name):
    ret_info = dict()
    pk_values, pk_locations = state_step_detection(pd_data)
    pitchs, rolls, yaws = pd_data['PitchX(deg)'].values, pd_data['RollY(deg)'].values, pd_data['YawZ(deg)'].values
    x_mags, y_mags, z_mags = pd_data['Mag_X(uT)'].values, pd_data['Mag_Y(uT)'].values, pd_data['Mag_Z(uT)'].values
    accs = pd_data[['Acc_X(m/s^2)', 'Acc_Y(m/s^2)', 'Acc_Z(m/s^2)']].values
    mags = np.concatenate((x_mags.reshape(-1, 1), y_mags.reshape(-1, 1), z_mags.reshape(-1, 1)), axis=1)

    R, I = get_rotation_matrix(accs, mags)
    values = get_orientation(R)
    orientations = to_degree(values[:, 0]).tolist()
    step_length = 0.78  # 写死
    phone_brand = "HUAWEI"
    end_x = 0
    end_y = 0
    start_x = 0
    start_y = 0
    step_list = []
    step_nums = len(pk_values)
    current_num = 0
    save_index = 0
    atom_steps = 0
    for step_index in range(step_nums - 3):
        step_info = dict()
        current_num = current_num + 1
        is_turn = False
        start_index, end_index = pk_values[step_index], pk_values[step_index + 3]
        pitch = pitchs[start_index: end_index]
        roll = rolls[start_index: end_index]
        yaw = yaws[start_index: end_index]
        # 统计到拐弯
        if (pitch.max() - pitch.min() > turn_threshold) \
                or (roll.max() - roll.min() > turn_threshold) \
                or (yaw.max() - yaw.min() > turn_threshold):
            is_turn = True
            current_num = 0
            if step_info.__len__ == 0 or atom_steps < 3:
                continue
            ret_info = {
                "endX": end_x,
                "endY": end_y,
                "stepLength": step_length,
                "name": "%s_%d_ACCEGYROMAGNAHRS.txt" % (base_name, save_index),
                "orientation": orientations[pk_values[step_index - atom_steps]: start_index],
                "phoneBrand": phone_brand,
                "startX": start_x,
                "startY": start_y,
                "stepList": step_list
            }

            print("保存 %d 条原子路径 ...." % save_index)
            save_path = os.path.join(save_dir, "%s_%d.txt" % (base_name, save_index))
            with open(save_path, 'w') as f:
                json.dump(ret_info, f, indent=4)
            step_list.clear()
            save_index += 1
            atom_steps = 0
            continue

        # 没有检测到拐弯
        x_mag = x_mags[start_index: pk_values[step_index + 1]]
        y_mag = y_mags[start_index: pk_values[step_index + 1]]
        z_mag = z_mags[start_index: pk_values[step_index + 1]]
        step_info.update({
            "currentNum": current_num,
            "isTurn": is_turn,
            "pitch": pitch.tolist(),
            "roll": roll.tolist(),
            "yaw": yaw.tolist(),
            "xMagnetic": x_mag.tolist(),
            "yMagnetic": y_mag.tolist(),
            "zMagnetic": z_mag.tolist()
        })
        atom_steps += 1
        step_list.append(step_info)


if __name__ == '__main__':
    import os
    import json

    data_root = r'/media/xzq/data-2/data/IPIN/Logfiles/01-Training/01a-Regular'
    save_root = r'/media/xzq/data-2/data/crow_server/magneticData/sampleData/ipin'
    if not os.path.exists(save_root):
        os.makedirs(save_root)

    print("convert ipin data to ori collected data ...")
    for name in os.listdir(data_root):
        if os.path.isdir(os.path.join(data_root, name)):
            csv_path = os.path.join(data_root, name, 'ACCEGYROMAGNAHRS.csv')
            pd_data = pd.read_csv(csv_path)
            get_step_list(pd_data, save_dir=save_root, base_name=name)
