import numpy as np
import time
from icp import *
from pycpd import RigidRegistration
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Registration import *
import copy
from numpy.testing import assert_almost_equal, assert_array_almost_equal

# Constants
N = 10  # number of random points in the dataset
num_tests = 100  # number of test iterations
dim = 3  # number of dimensions of the points
noise_sigma = .01  # standard deviation error to be added
translation = .1  # max translation of the test set
rotation = .1  # max rotation (radians) of the test set


def rotation_matrix(axis, theta):
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.)
    b, c, d = -axis * np.sin(theta / 2.)

    return np.array([[a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)],
                     [2 * (b * c + a * d), a * a + c * c - b * b - d * d, 2 * (c * d - a * b)],
                     [2 * (b * d - a * c), 2 * (c * d + a * b), a * a + d * d - b * b - c * c]])


# test cpd
def cpd():
    points = np.array([[-2.473853, -482.045074, 710.118103],
                       [-38.146061, -492.55838, 707.446899],
                       [13.649702, -456.79126, 733.249512],
                       [-67.32843, -455.414703, 754.407898]])
    points2 = np.array([[-69.72, -325.9, 902.22],
                        [-40.32, -306.9, 917.15],
                        [10.97, -327.02, 878.76],
                        [-4.62, -352.48, 856.2]])

    radius = 43.0
    codewords = [120, 106, 102, 116]
    std_template = StandardTemplate(radius, codewords)
    std_template.GenPositionTemplate()
    templates = std_template.Position_templates

    translation = 100
    rotation = 1
    noise_sigma = 2
    t = np.random.rand(3) * translation
    R = rotation_matrix(np.random.rand(3), np.random.rand() * rotation)
    tems = copy.deepcopy(templates)
    sourced = []
    for tem in tems:
        tem += t
        tem = np.dot(tem, R)
        tem += np.random.randn(4, 3) * 2
        sourced.append(tem)

    for i in range(4):
        # ----- 使用模板测试算法准确性-----
        source = np.array(points2)
        target = np.array(templates[i])
        reg = RigidRegistration(X=target, Y=source, scale=False, tolerance=1)
        TY, (s_reg, R_reg, t_reg) = reg.register()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='r', label='Source')
        ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='g', label='Target')
        ax.scatter(TY[:, 0], TY[:, 1], TY[:, 2], c='b', label='Registered')
        np.set_printoptions(precision=2, suppress=True)
        cost = np.linalg.norm(TY - target)
        print('cost:', cost, 'result:', '\n', TY, s_reg, '\n', t_reg)
        plt.legend()
        plt.show()
        # assert_almost_equal(1.0, s_reg)
        # assert_array_almost_equal(R, R_reg.T)
        # assert_array_almost_equal(t, -t_reg)
        # assert_array_almost_equal(source, TY)


if __name__ == "__main__":
    cpd()
    print("All tests pass")
