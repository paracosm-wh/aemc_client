from pycpd import RigidRegistration
import numpy as np


def ch(a):
    a = np.vstack((a, [1, 2, 3]))
    return a
A = np.zeros((0, 3))
A =ch(A)
print(A)

# create 2D target points (you can get these from any source you desire)
# creating a square w/ 2 additional points. 
target = np.array([[0, 0], [0, 1], [1, 0], [1, 1], [0.5, 0], [0, 0.5]])
print('Target Points: \n', target)

# create a translation to apply to the target for testing the registration
translation = [1, 0]

# create a fake source by adding a translation to the target.
# in a real use, you would load the source points from a file or other source. 
# the only requirement is that this array also be 2-dimensional and that the 
# second dimension be the same length as the second dimension of the target array.
source = target + translation
print('Source Points: \n', source)

# create a RigidRegistration object
reg = RigidRegistration(X=target, Y=source)
# run the registration & collect the results
TY, (s_reg, R_reg, t_reg) = reg.register()

print(TY, '\n', translation, '\n', reg.diff)
# TY is the transformed source points
# the values in () are the registration parameters.
# In this case of rigid registration they are:
#     s_reg the scale of the registration
#     R_reg the rotation matrix of the registration
#     t_reg the translation of the registration
