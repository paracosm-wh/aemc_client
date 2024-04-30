"""
TODO: 源数据中心化--->最近邻点对齐--->给定6个离散的旋转角度
"""
import numpy as np


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
from numpy.linalg import lstsq
import matplotlib.pyplot as plt
from Registration import *

# 生成一些三维点数据
points = np.array([[-2.473853, -482.045074, 710.118103],
                   [-38.146061, -492.55838, 707.446899],
                   [13.649702, -456.79126, 733.249512],
                   [-67.32843, -455.414703, 754.407898]])
# points = np.array([[1, 0, 0],
#                    [0, 1, 0],
#                    [-1, 0, 0],
#                    [0, -1, 0]])
# 创建模板
# radius = 43.0
# codewords = [120, 106, 102, 116]
# std_template = StandardTemplate(radius, codewords)
# std_template.GenPositionTemplate()
# templates = std_template.Position_templates
# points = np.array(templates[3]).reshape(4, 3)

# 构建线性方程组
A = np.column_stack((-2 * points, np.ones(len(points))))
b = np.sum(points ** 2, axis=1).reshape(-1, 1)

# s = np.linalg.inv(A.T @ A) @ A.T @ b
# print("拟合圆心:", s[:3])
# print("拟合半径:", np.sqrt(s[3] + s[0] ** 2 + s[1] ** 2 + s[2] ** 2))
# 使用最小二乘法求解拟合圆的参数
params, _, _, _ = lstsq(A, b, cond=None)

# 圆心为 (x0, y0, z0)
center = params[:3].reshape(1, 3)
# 半径为 r
r = np.sqrt(params[3] + center[0][0] ** 2 + center[0][1] ** 2 + center[0][2] ** 2)

print("圆心:", center)
print("半径:", r)

for i in range(len(points)):
    print(np.linalg.norm(points[i] - center))

# 绘制散点图以及拟合的圆
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2])
# 根据圆心和半径绘制圆
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = r * np.outer(np.cos(u), np.sin(v)) + center[0][0]
y = r * np.outer(np.sin(u), np.sin(v)) + center[0][1]
z = r * np.outer(np.ones(np.size(u)), np.cos(v)) + center[0][2]
ax.plot_surface(x, y, z, color='b', alpha=0.3)
plt.show()
