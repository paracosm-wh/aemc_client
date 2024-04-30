import numpy as np
from sklearn.metrics.pairwise import rbf_kernel
from sklearn.metrics.pairwise import pairwise_kernels
from scipy.optimize import minimize
from numpy.linalg import lstsq


def kernel_correlation(target, source, gamma=None):
    """
    Compute the kernel correlation between two point sets X and Y.

    Parameters:
    X: ndarray of shape (n_samples_X, n_features)
        First point set.
    Y: ndarray of shape (n_samples_Y, n_features)
        Second point set.
    gamma: float, default=None
        Kernel coefficient for rbf, poly, sigmoid, laplacian and chi2 kernels.
        If gamma is None, defaults to 1.0 / n_features.

    Returns:
    correlation: float
        The kernel correlation between X and Y.
    """
    # 计算点云均值并中心化
    source_mean = np.mean(source, axis=0)
    source_centered = source - source_mean

    # 计算X和Y的RBF核
    K_target = pairwise_kernels(target, metric='rbf', gamma=gamma)
    K_source = pairwise_kernels(source_centered, metric='rbf', gamma=gamma)

    # 定义优化目标函数
    def objective_function(x):
        return -np.sum(K_source * np.roll(K_target, -int(x[0]), axis=1))

    # 使用最小化优化目标函数，找到最佳的配准变换
    result = minimize(objective_function, [0], bounds=[(-len(source), len(source))])

    # 获取最佳的配准变换参数
    shift = result.x[0]

    # 计算匹配误差
    correlation = -result.fun

    return shift, correlation


def SVD2Wahba(source_points, target_points):
    """
    将问题定义为仅通过3D旋转对两个点集进行配准的Wahba问题。
    使用SVD分解求解
    """
    # 拟合的圆心作为质心
    source_mean = FittingCircle(source_points)[0]

    # 中心化
    source_centered = source_points - source_mean

    # 计算协方差矩阵
    H = np.dot(source_centered.T, target_points)

    # 使用SVD分解计算旋转矩阵
    U, _, Vt = np.linalg.svd(H)

    # 计算旋转矩阵
    R = np.dot(Vt.T, U.T)

    # 计算最后的代价
    cost = np.linalg.norm(np.dot(source_centered, R) - target_points)

    # 返回结果便于可视化对比
    re = np.dot(source_centered, R)

    return re, source_mean, R, cost


def FittingCircle(points):
    """
    使用最小二乘法拟合三维点的圆心和半径
    """
    points = np.array(points, dtype='float')
    num = points.shape[0]
    # 构造A, B矩阵
    A = np.column_stack((-2 * points, np.ones(num)))
    b = np.sum(points ** 2, axis=1).reshape(-1, 1)

    # 使用最小二乘法求解拟合圆的参数
    params, _, _, _ = np.linalg.lstsq(A, b)

    # 圆心为 (x0, y0, z0)
    center = params[:3].reshape(1, 3)
    x0, y0, z0 = center[0][:3]
    # 半径为 r
    r = np.sqrt(params[3] + x0 ** 2 + y0 ** 2 + z0 ** 2)
    return center, r
