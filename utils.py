import numpy as np


def softmax(X: np.ndarray, theta=1.0, axis=None):
    """
    Compute the softmax of each element along an axis of X.

    Parameters
    ----------
    X: ND-Array. Probably should be floats.
    theta (optional): float parameter, used as a multiplier
        prior to exponentiation. Default = 1.0
    axis (optional): axis to compute values along. Default is the
        first non-singleton axis.

    Returns an array the same size as X. The result will sum to 1
    along the specified axis.
    """

    # make X at least 2d
    y = np.atleast_2d(X)

    # find axis
    if axis is None:
        axis = next(j[0] for j in enumerate(y.shape) if j[1] > 1)

    # multiply y against the theta parameter,
    y = y * float(theta)

    # subtract the max for numerical stability
    y = y - np.expand_dims(np.max(y, axis=axis), axis)

    # exponentiate y
    y = np.exp(y)

    # take the sum along the specified axis
    ax_sum = np.expand_dims(np.sum(y, axis=axis), axis)

    # finally: divide elementwise
    p = y / ax_sum

    # flatten if X was 1D
    if len(X.shape) == 1: p = p.flatten()

    return p


import json

json_data = json.load(open(r'/media/xzq/data-2/data/crow_server/magneticData/sampleData/mate8/140.0-420.0--146.0-610.0-2017-02-27-10-39-17-713.txt', encoding='utf-8'))

save_path = r'/media/xzq/data-2/data/crow_server/magneticData/sampleData/mate8/140.0-420.0--146.0-610.0-2017-02-27-10-39-17-713.json'

with open(save_path, 'w') as f:
    json.dump(json_data, f, indent=4)