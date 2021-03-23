import numpy as np

def ForwardKinematics(skeleton_HT):
    # initialize
    num = 0
    HT_absolute = [0 for i in skeleton_HT]
    # Base(J0) = J1
    HT_absolute[0] = skeleton_HT[0]

    for i in skeleton_HT:

        if num == 0:
            HT_absolute[0] = skeleton_HT[0]
            num = num + 1

        else:
            HT_absolute[num] = HT_absolute[num - 1].dot(i)
            num = num + 1

    return np.array(HT_absolute)