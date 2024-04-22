from pycpd import RigidRegistration
import numpy as np
import pandas as pd
import math
import csv

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
        for c in self.codewords_:
            code = bin(c)[2:].zfill(Length)  # Convert to binary and zero-pad
            code_num = code.count('1')
            points = np.zeros((code_num, 3))
            index = 0
            for i in range(Length):
                if code[i] == '1' and index < code_num:
                    x = self.radius_ * math.cos(2 * math.pi / Length * (Length - i))
                    y = self.radius_ * math.sin(2 * math.pi / Length * (Length - i))
                    points[index] = [x, y, 0]
                    index += 1
            self.Position_templates.append(points)


class FrameData(object):
    """
    将每帧数据根据欧氏聚类分为多个潜在目标
    """

    def __init__(self, frame_id, points):
        self.frame_id = frame_id
        self.points = points
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
            processed[i] = True

            while sq_idx < len(seed_queue):
                if not self.RadiusSearch(seed_queue[sq_idx], radius_err, processed, points_seed_queue):
                    sq_idx += 1
                    continue

            if len(seed_queue) >= MinClusterNum:
                self.PotentialCluster.append(points_seed_queue)
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
            np.vstack(self.Single_data, self.points[unprocessed[k]])

    def RadiusSearch(self, seed_idx, radius, process, points_seed_queue):
        flag_num = 0
        for i in range(self.num_points):
            if not process[i]:
                dis = np.linalg.norm(self.points[i] - self.points[seed_idx])
                if dis < radius:
                    points_seed_queue = np.vstack((points_seed_queue, self.points[i]))
                    process[i] = True
                    flag_num += 1
        return flag_num > 0


# class Registration(object):


if __name__ == '__main__':
    # 创建模板
    radius = 43.0
    codewords = [120, 106, 102, 116]
    std_template = StandardTemplate(radius, codewords)
    std_template.GenPositionTemplate()
    templates = std_template.Position_templates

    path = r"/home/itr-wh/Work/AEMC_data/RAL-data/1-L-middle1-raw.csv"
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
            # 欧氏聚类
            frame_data.EuclideanCluster()
            # cpd registration
