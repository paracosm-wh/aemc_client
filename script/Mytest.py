"""
TODO: 源数据中心化--->最近邻点对齐--->给定6个离散的旋转角度
"""
import numpy as np
from Registration import *
from Register import *
import matplotlib.pyplot as plt


def gaussian_kernel(x, y, sigma=1.0):
    """
    计算两个四边形之间的高斯核函数值

    参数：
    x, y: numpy数组，分别表示两个四边形的三维坐标，每行一个点，共四个点
    sigma: 高斯核函数的带宽参数，默认为1.0

    返回：
    kernel_value: 高斯核函数值
    """
    # 计算欧氏距离的平方
    distance_squared = np.sum((x - y) ** 2)

    # 计算高斯核函数值
    kernel_value = np.exp(-distance_squared / (2 * sigma ** 2))

    return kernel_value


def rotation_matrix(axis, theta):
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.)
    b, c, d = -axis * np.sin(theta / 2.)

    return np.array([[a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)],
                     [2 * (b * c + a * d), a * a + c * c - b * b - d * d, 2 * (c * d - a * b)],
                     [2 * (b * d - a * c), 2 * (c * d + a * b), a * a + d * d - b * b - c * c]])


'''
# 示例：生成两个随机四边形的三维坐标
np.random.seed(0)
quadrilateral1 = np.random.randn(4, 3)  # 四边形1
quadrilateral2 = np.random.randn(4, 3)  # 四边形2

# 计算高斯核函数值
kernel_value = gaussian_kernel(quadrilateral1, quadrilateral2)

print("高斯核函数值:", kernel_value)
'''

import numpy as np
import copy
from numpy.linalg import lstsq
import matplotlib.pyplot as plt
from Registration import *
from icp import *

# 生成一些三维点数据
points = np.array([[-2.473853, -482.045074, 710.118103],
                   [-38.146061, -492.55838, 707.446899],
                   [13.649702, -456.79126, 733.249512],
                   [-67.32843, -455.414703, 754.407898]])
points2 = np.array([[-69.72, -325.9, 902.22],
                    [-40.32, -306.9, 917.15],
                    [10.97, -327.02, 878.76],
                    [-4.62, -352.48, 856.2]])
# points = np.array([[1, 0, 0],
#                    [0, 1, 0],
#                    [-1, 0, 0],
#                    [0, -1, 0]])
# 创建模板
radius = 43.0
codewords = [120, 106, 102, 116]
std_template = StandardTemplate(radius, codewords)
std_template.GenPositionTemplate()
templates = std_template.Position_templates
# points = np.array(templates[3]).reshape(4, 3)
# TODO 检查欧式聚类的结果——欧式聚类的结果没问题
'''
r_3, c_3, = FittingCircle(points)
print("拟合圆心:", c_3, "拟合半径:", r_3)
# 构建线性方程组
A = np.column_stack((-2 * points, np.ones(len(points))))
b = np.sum(points ** 2, axis=1).reshape(-1, 1)

s = np.linalg.inv(A.T @ A) @ A.T @ b
print("拟合圆心:", s[:3])
print("拟合半径:", np.sqrt(s[3] + s[0] ** 2 + s[1] ** 2 + s[2] ** 2))
# 使用最小二乘法求解拟合圆的参数
params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

# 圆心为 (x0, y0, z0)
center = params[:3].reshape(1, 3)
# 半径为 r
r = np.sqrt(params[3] + center[0][0] ** 2 + center[0][1] ** 2 + center[0][2] ** 2)

print("圆心:", center)
print("半径:", r)

for i in range(len(points)):
    print(np.linalg.norm(points[i] - center))
    
'''

np.set_printoptions(precision=2, suppress=True)

translation = 1000
rotation = 1
noise_sigma = 2
t = np.random.rand(3) * translation
R = rotation_matrix(np.random.rand(3), np.random.rand() * rotation)

tems = copy.deepcopy(templates)
sourced = []
for tem in tems:
    tem += t
    tem = np.dot(R, tem.T).T
    tem += np.random.randn(4, 3) * 2
    sourced.append(tem)

for i in range(4):
    # ----- 使用模板测试算法准确性-----
    source = np.array(sourced[i])
    target = np.array(templates[i])
    # re, source_mean, rotate, cost = SVD2Wahba(source, target)
    rotate, t, cost = IterativeClosestPoint(source.T, target.T, 1e-6)
    print("SVD:", cost)
    # registered = np.dot(rotate, source.T).T
    registered = np.dot(rotate, source.T).T + t.T
    print('R_err', rotate.T - R, '\n', '未转置：', rotate - R)
    assert np.allclose(rotate.T, R, atol=6 * noise_sigma)

    reg = RigidRegistration(X=target, Y=source)
    reg.register()

    print('i:', i, 'q:', reg.q)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.cla()
    ax.scatter(target[:, 0], target[:, 1], target[:, 2], color='red', label='Target')
    # ax.scatter(source[:, 0], source[:, 1], source[:, 2], color='blue', label='Source')
    ax.scatter(registered[:, 0], registered[:, 1], registered[:, 2], color='green', label='Registered')
    ax.legend(loc='upper left', fontsize='x-large')
    plt.draw()
    plt.pause(1)
    # ------使用模板测试算法准确性---end-----
    # source = np.array(points)
    # target = np.array(templates[i])
    # re, source_mean, rotate, cost = SVD2Wahba(source, target)
    # print("SVD:", cost)
    # source_centered = source - source_mean
    # registered = np.dot(rotate, source_centered.T).T
    # fig = plt.figure()
    # # 只可视化x，y平面
    # # ax = fig.add_subplot(111)
    # # plt.cla()
    # # ax.scatter(target[:, 0], target[:, 1], color='red', label='Target')
    # # # ax.scatter(source_centered[:, 0], source_centered[:, 1], color='blue', label='Source')
    # # ax.scatter(registered[:, 0], registered[:, 1], color='green', label='Registered')
    #
    # ax = fig.add_subplot(111, projection='3d')
    # plt.cla()
    # # ax.scatter(source[:, 0], source[:, 1], source[:, 2], color='blue', label='Source')
    # ax.scatter(target[:, 0], target[:, 1], target[:, 2], color='red', label='Target')
    # ax.scatter(source_centered[:, 0], source_centered[:, 1], source_centered[:, 2], color='black', label='Source')
    # ax.scatter(registered[:, 0], registered[:, 1], registered[:, 2], color='green', label='Registered')
    # ax.legend(loc='upper left', fontsize='x-large')
    # plt.draw()
    # plt.pause(1)
