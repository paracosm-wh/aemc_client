from numpy import *


def fit_circle_2d(x, y, w=[]):
    """
    -------------------------------------------------------------------------------
    FIT CIRCLE 2D
    - Find center [xc, yc] and radius r of circle fitting to set of 2D points
    - Optionally specify weights for points

    - Implicit circle function:
      (x-xc)^2 + (y-yc)^2 = r^2
      (2*xc)*x + (2*yc)*y + (r^2-xc^2-yc^2) = x^2+y^2
      c[0]*x + c[1]*y + c[2] = x^2+y^2

    - Solution by method of least squares:
      A*c = b, c' = argmin(||A*c - b||^2)
      A = [x y 1], b = [x^2+y^2]
    -------------------------------------------------------------------------------
    """
    A = array([x, y, ones(len(x))]).T
    b = x ** 2 + y ** 2

    # Modify A,b for weighted least squares
    if len(w) == len(x):
        W = diag(w)
        A = dot(W, A)
        b = dot(W, b)

    # Solve by method of least squares
    c = linalg.lstsq(A, b, rcond=None)[0]

    # Get circle parameters from solution c
    xc = c[0] / 2
    yc = c[1] / 2
    r = sqrt(c[2] + xc ** 2 + yc ** 2)
    return xc, yc, r


def rodrigues_rot(P, n0, n1):
    """
    -------------------------------------------------------------------------------
    RODRIGUES ROTATION
    - Rotate given points based on a starting and ending vector
    - Axis k and angle of rotation theta given by vectors n0,n1
      P_rot = P*cos(theta) + (k x P)*sin(theta) + k*<k,P>*(1-cos(theta))
    -------------------------------------------------------------------------------
    """
    # If P is only 1d array (coords of single point), fix it to be matrix
    if P.ndim == 1:
        P = P[newaxis, :]

    # Get vector of rotation k and angle theta
    n0 = n0 / linalg.norm(n0)
    n1 = n1 / linalg.norm(n1)
    k = cross(n0, n1)
    k = k / linalg.norm(k)
    theta = arccos(dot(n0, n1))

    # Compute rotated points
    P_rot = zeros((len(P), 3))
    for i in range(len(P)):
        P_rot[i] = P[i] * cos(theta) + cross(k, P[i]) * sin(theta) + k * dot(k, P[i]) * (1 - cos(theta))

    return P_rot
