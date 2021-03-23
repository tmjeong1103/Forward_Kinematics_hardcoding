import numpy as np

# Define Rotation Matrix (4X4)
class Rotation:
    def E():
        E = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 0]])
        return E

    def x(theta):
        theta = np.deg2rad(theta)
        Rot_x = np.array([[1, 0, 0, 0],
                          [0, np.cos(theta), -np.sin(theta), 0],
                          [0, np.sin(theta), np.cos(theta), 0],
                          [0, 0, 0, 0]])
        return Rot_x

    def y(theta):
        theta = np.deg2rad(theta)
        Rot_y = np.array([[np.cos(theta), 0, np.sin(theta), 0],
                          [0, 1, 0, 0],
                          [-np.sin(theta), 0, np.cos(theta), 0],
                          [0, 0, 0, 0]])
        return Rot_y

    def z(theta):
        theta = np.deg2rad(theta)
        Rot_z = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                          [np.sin(theta), np.cos(theta), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 0]])
        return Rot_z


# Define Translation Matrix (4X4)
def Translation(x, y, z):
    P = np.array([[0, 0, 0, x],
                  [0, 0, 0, y],
                  [0, 0, 0, z],
                  [0, 0, 0, 1]])
    return P


# calculate Homogeneous Transformation marix
def HT_matrix(R, P):
    # Note : R&P are np_array form
    H = R + P
    return H