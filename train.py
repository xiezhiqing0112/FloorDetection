import pandas as pd
import xgboost as xgb
from sklearn.metrics import accuracy_score, confusion_matrix
import numpy as np
from utils import softmax
from sklearn.model_selection import train_test_split, KFold, cross_val_score


def train_val_split(df_data: pd.DataFrame):
    columns = list(df_data.columns)
    columns.remove('label')
    X = df_data[columns]
    Y = df_data['label'].map({-1: 0, 6: 1, 7: 2})
    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.7, random_state=0)
    return X_train, X_test, Y_train, Y_test


def train_xgboost(df_train: pd.DataFrame, df_test: pd.DataFrame):
    """

    :param df_train: [rssi_1, rssi_2, rssi_3, ..., rssi_n, floor_label]
    :param df_test:  [rssi_1, rssi_2, rssi_3, ..., rssi_n, x, y, z, floor_label]
    :return:
    """
    train_X, train_Y = df_train.iloc[:, :-1].values, df_train.iloc[:, -1].values
    test_data = df_test.values
    # 对ori0817的中空和非中空数据进行筛选
    # 中空220 420 620 880 0 15
    # 特殊区域98 186 255 849 -10 15
    minXBound = 220
    maxXBound = 420
    minYBound = 620
    maxYBound = 880
    minZBound = 0
    maxZBound = 15
    condition = np.where((maxXBound >= test_data[:, -4]) & (test_data[:, -4] >= minXBound)
                         & (minYBound <= test_data[:, -3]) & (test_data[:, -3] <= maxYBound)
                         & (minZBound <= test_data[:, -2]) & (test_data[:, -2] <= maxZBound))
    test_X, test_Y = test_data[condition][:, :-4], test_data[condition][:, -1]
    assert train_X.shape[1] == test_X.shape[1], "train data feature dim not equal to test data feature dim"

    xg_train = xgb.DMatrix(train_X, label=train_Y)
    xg_test = xgb.DMatrix(test_X, label=test_Y)
    params = {
        'objective': 'multi:softprob',  # output each floor's prob
        'eta': 0.1,
        'slient': 1,
        'num_class': 4,
        'max_depth': 8,
        'nthread': 4,
        'eval_metric': 'merror',
        'seed': 777,
    }
    watchlist = [(xg_train, 'train'), (xg_test, 'test')]
    num_round = 50
    bst = xgb.train(params, xg_train, num_round, watchlist)

    bst.save_model('./output/model/xgb_model.model')

    pre_prob = bst.predict(xg_test)
    pd.DataFrame(pre_prob).to_csv("./output/result/result_prob.csv")
    pre = np.argmax(pre_prob, axis=1)
    pd.DataFrame(pre_prob).to_csv("./output/result/result.csv")
    acc = accuracy_score(test_Y, pre)
    print(acc)
    con_m = confusion_matrix(test_Y, pre, labels=[0, 1, 2, 3])
    print('the confusion matrix:')
    print(con_m)


def train_xgboost_kf(df_train: pd.DataFrame, is_kf=False):
    """

    :param df_train: [rssi_1, rssi_2, rssi_3, ..., rssi_n, label]
    :param is_kf: 交叉验证
    :return:
    """
    if is_kf:
        columns = list(df_train.columns)
        columns.remove('label')
        X = df_train[columns].values
        Y = df_train['label'].map({-1: 0, 6: 1, 7: 2}).values
        kf = KFold(n_splits=5, shuffle=True)
        for train_index, test_index in kf.split(X):
            train_X, test_X = X[train_index], X[test_index]
            train_Y, test_Y = Y[train_index], Y[test_index]
            xg_train = xgb.DMatrix(train_X, label=train_Y)
            xg_test = xgb.DMatrix(test_X, label=test_Y)
            params = {
                'objective': 'multi:softprob',  # output each floor's prob
                'eta': 0.1,
                'slient': 1,
                'num_class': 3,
                'max_depth': 8,
                'nthread': 4,
                'eval_metric': 'merror',
                'seed': 777,
            }
            watchlist = [(xg_train, 'train'), (xg_test, 'test')]
            num_round = 10
            bst = xgb.train(params, xg_train, num_round, watchlist)

            bst.save_model('./output/model/xgb_model.model')

            pre_prob = bst.predict(xg_test)
            pre = np.argmax(pre_prob, axis=1)
            acc = accuracy_score(test_Y, pre)
            print(acc)
            con_m = confusion_matrix(test_Y, pre, labels=[0, 1, 2, 3])
            print('the confusion matrix:')
            print(con_m)
    else:
        train_X, test_X, train_Y, test_Y = train_val_split(df_train)

        xg_train = xgb.DMatrix(train_X, label=train_Y)
        xg_test = xgb.DMatrix(test_X, label=test_Y)
        params = {
            'objective': 'multi:softprob',  # output each floor's prob
            'eta': 0.1,
            'slient': 1,
            'num_class': 3,
            'max_depth': 8,
            'nthread': 4,
            'eval_metric': 'merror',
            'seed': 777,
        }
        watchlist = [(xg_train, 'train'), (xg_test, 'test')]
        num_round = 50
        bst = xgb.train(params, xg_train, num_round, watchlist)

        bst.save_model('./output/model/xgb_model.model')

        pre_prob = bst.predict(xg_test)
        pd.DataFrame(pre_prob).to_csv("./output/result/result_prob.csv")
        pre = np.argmax(pre_prob, axis=1)
        pd.DataFrame(pre_prob).to_csv("./output/result/result.csv")
        acc = accuracy_score(test_Y, pre)
        print(acc)
        con_m = confusion_matrix(test_Y, pre, labels=[0, 1, 2])
        print('the confusion matrix:')
        print(con_m)


if __name__ == '__main__':
    # df_train = pd.read_csv('/home/xzq/meituan/floor_detection/0904train.csv', header=None)
    # df_test = pd.read_csv('/home/xzq/meituan/floor_detection/ori_test0817_huawei_rssi.csv', header=None)
    # train_xgboost(df_train, df_test)  # 师姐之前的数据运行代码

    # 从json中解析得到的数据
    df_train = pd.read_csv(r'./processed_data.csv')
    train_xgboost_kf(df_train, is_kf=False)
