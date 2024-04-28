import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from functools import partial
from pycpd import RigidRegistration
import math
import csv

from icp import *

Length = 7
ClassArray = [17.29, 0.0, 0.0, 35, 39.0, 43.0]
stand_deviation = 1.5
ERR = 3 * stand_deviation
MinClusterNum = 3


class StandardTemplate(object):
    """
    生成三维点的标准模板
    """

    def __init__(self, radius, codewords):
        self.radius_ = radius
        self.codewords_ = codewords
        self.Position_templates = []

    def GenPositionTemplate(self):
        """
        x = r * cos(2 * pi / L * i)
        y = r * sin(2 * pi / L * i)
        z = 0
        """
        for c in self.codewords_:
            code = bin(c)[2:].zfill(Length)  # Convert to binary and zero-pad
            code_num = code.count('1')
            points = np.zeros((code_num, 3))
            index = 0
            for i in range(Length):
                if code[i] == '1' and index < code_num:
                    x = self.radius_ * math.cos(2 * math.pi / Length * i)
                    y = - self.radius_ * math.sin(2 * math.pi / Length * i)
                    points[index] = [x, y, 0]
                    index += 1
            self.Position_templates.append(points)


class FrameData(object):
    """
    将每帧数据根据欧氏聚类分为多个潜在目标
    """

    def __init__(self, frame_id, points):
        self.frame_id = frame_id
        self.points = points.astype(float)
        self.num_points = len(points)
        self.PotentialCluster = []
        self.Single_data = np.zeros((0, 3))

    def EuclideanCluster(self):
        """
        欧氏聚类
        """
        radius_err = 2 * ClassArray[-1] + ERR
        processed = [False] * self.num_points  # Flags for clustering processing

        for i in range(self.num_points):
            if processed[i]:
                continue

            points_seed_queue = np.zeros((0, 3))  # Storing points for a cluster
            seed_queue = []  # Storing indices for a cluster
            sq_idx = 0
            seed_queue.append(i)
            points_seed_queue = np.vstack((points_seed_queue, self.points[i]))
            processed[i] = True

            while sq_idx < len(seed_queue):
                re, cluster_points = self.RadiusSearch(seed_queue[sq_idx], radius_err, processed, points_seed_queue)
                if not re:
                    sq_idx += 1
                    continue

                if cluster_points.shape[0] >= MinClusterNum:
                    self.PotentialCluster.append(cluster_points)
                else:
                    for p in seed_queue:
                        processed[p] = False

        unprocessed = []
        for j in range(self.num_points):
            if processed[j]:
                continue
            else:
                unprocessed.append(j)

        for k in range(len(unprocessed)):
            np.vstack((self.Single_data, self.points[unprocessed[k]]))

    def RadiusSearch(self, seed_idx, radius, process, points_seed_queue):
        flag_num = 0
        for i in range(self.num_points):
            if not process[i]:
                dis = np.linalg.norm(self.points[i] - self.points[seed_idx])
                if dis < radius:
                    points_seed_queue = np.vstack((points_seed_queue, self.points[i]))
                    process[i] = True
                    flag_num += 1
        return flag_num > 0, points_seed_queue


def FittingCircle(points):
    """
    使用最小二乘法拟合三维点的圆心和半径
    """
    # 三维点的个数
    num = points.shape[0]
    L1 = np.ones((num, 1))
    A = np.linalg.inv(points.T @ points) @ points.T @ L1

    B = np.zeros((num - 1, 3))
    for i in range(num - 1):
        B[i, :] = points[i + 1, :] - points[i, :]

    L2 = np.zeros((num - 1, 1))
    for i in range(num - 1):
        L2[i] = (np.sum(points[i + 1, :] ** 2) - np.sum(points[i, :] ** 2)) / 2

    D = np.zeros((4, 4))
    D[:3, :3] = B.T @ B
    D[:3, 3] = A.squeeze()
    D[3, :3] = A.T.squeeze()
    D[3, 3] = 1

    L3 = np.concatenate([B.T @ L2, np.array([[1]])])
    C = np.linalg.solve(D, L3)
    return C[:3]


def visualize(X, Y, ax):
    """
        Visualize the registration process for cpd.
    """
    plt.cla()
    ax.scatter(X[:, 0], X[:, 1], X[:, 2], color='red', label='Target')
    ax.scatter(Y[:, 0], Y[:, 1], Y[:, 2], color='blue', label='Source')
    # ax.scatter(X[:, 0], X[:, 1], color='red', label='Target')
    # ax.scatter(Y[:, 0], Y[:, 1], color='blue', label='Source')
    # ax.text2D(0.87, 0.92, 'Iteration: {:d}\nQ: {:06.4f}'.format(
    #     iteration, error), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes,
    #           fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    plt.draw()
    plt.pause(1)







if __name__ == '__main__':
    # 创建模板
    radius = 43.0
    codewords = [120, 106, 102, 116]
    std_template = StandardTemplate(radius, codewords)
    std_template.GenPositionTemplate()
    templates = std_template.Position_templates
    np.set_printoptions(precision=2, suppress=True)

    """
    source_pts : 来自于动作捕捉系统的原始数据
    target_pts : 来自于标准模板
    """

    # 创建储存结果的pandas类型的dataframe
    result = pd.DataFrame(
        columns=['frame_id', 'x_0', 'y_0', 'z_0', 'x_1', 'y_1', 'z_1', 'x_2', 'y_2', 'z_2', 'x_3', 'y_3', 'z_3'])

    path = r"./1-L-middle1-raw.csv"
    # 按行读取数据
    with open(path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            frame_id = row[0]
            points = np.empty((0, 3), dtype=float)
            # 忽略空值
            for i in range(2, len(row), 3):
                if row[i] == '' or row[i + 1] == '' or row[i + 2] == '':
                    continue
                points = np.vstack((points, np.array([row[i], row[i + 1], row[i + 2]])))
            frame_data = FrameData(frame_id, points)
            # 初始化结果
            result.append([frame_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            # 欧氏聚类
            frame_data.EuclideanCluster()
            # cpd registration
            print(frame_id)
            for cluster in frame_data.PotentialCluster:
                cluster = np.array(cluster)
                cost = np.inf
                reg_result = np.zeros((0, 3))
                best_index = 0
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                for i in range(len(templates)):
                    # ---------for cpd registration---------
                    # debug 中添加绘图
                    # '''
                    # callback = partial(visualize, ax=ax)
                    reg = RigidRegistration(X=templates[i], Y=cluster,)
                    # reg = RigidRegistration(X=templates[i], Y=cluster)
                    reg.register()
                    # plt.show()
                    print('i:', i, 'q:', reg.q)
                    if reg.q < cost:
                        cost = reg.q
                        reg_result = reg.TY
                        best_index = i
                    np.set_printoptions(precision=2, suppress=True)
                    print(reg_result)
                    # '''
                    '''
                    # ---------for icp registration---------
                    R, t, rmse = IterativeClosestPoint(source_pts=cluster.T, target_pts=templates[i].T, tau=1e-13)
                    print('i:', i, 'rmse:', rmse)
                    if rmse < cost:
                        cost = rmse
                        reg_result = ApplyTransformation(cluster.T, R, t).T
                        best_index = i
                    '''
                # 保存结果
                # 保存拟合的圆心到result
                print('Best index:', best_index)
                C = FittingCircle(cluster)
                print(C.flatten(), '\n')
                # 根据所匹配的最佳模板的索引，将结果保存到最后一行的特定的列
                result.loc[-1, (3 * best_index + 1):(3 * best_index + 4)] = C.flatten()
                visualize(templates[best_index], reg_result, ax)
    result.to_csv(r"\1-L-middle1-result.csv", index=False)
