
from math import sin, cos, asin, atan2, acos, sqrt, fabs
import numpy as np

def pcToSoSMatrix():
    """
    Create a matrix to convert point cloud positions to start of state
    positions
    """
    return xRotationMatrix(np.radians(-90))

def sosToPCMatrix():
    """
    Create a matrix to convert Start of Station positions to point cloud
    positions
    """
    return xRotationMatrix(np.radians(90))

def pcToDevMatrix():
    """
    Create a matrix to convert point cloud positions to device positions
    """
    return xRotationMatrix(np.radians(-180))

def devToPCMatrix():
    """
    Create a matrix that converts device positions to point cloud positions
    """
    return xRotationMatrix(np.radians(180))

def getPC2WorldMatrix(t, q):
    """ Create a matrix that transforms point clouds to world coordinates """
    devToIMU = pcToDevMatrix()
    mat = quaternionToRotationMatrix(q)
    mat[3, 0] = t[0]
    mat[3, 1] = t[1]
    mat[3, 2] = t[2]
    return devToIMU.dot(mat)

def getTranslationMatrix(t):
    """ Create a translation matrix """
    mat = np.zeros((4,4))
    mat[0, 0] = 1
    mat[1, 1] = 1
    mat[2, 2] = 1
    mat[3, 0] = t[0]
    mat[3, 1] = t[1]
    mat[3, 2] = t[2]
    mat[3, 3] = 1
    return mat

def xRotationMatrix(ang):
    """ Create a rotation matrix or a rotation around the X axis """
    mat = np.zeros((4,4))
    s = sin(ang)
    c = cos(ang)
    mat[0, 0] = 1
    mat[1, 1] = c
    mat[2, 1] = -s
    mat[1, 2] = s
    mat[2, 2] = c
    mat[3, 3] = 1
    return mat

def yRotationMatrix(ang):
    """ Create a rotation matrix for a rotation around the Y axis """
    mat = np.zeros((4,4))
    s = sin(ang)
    c = cos(ang)
    mat[0, 0] = c
    mat[2, 0] = s
    mat[1, 1] = 1
    mat[0, 2] = -s
    mat[2, 2] = c
    mat[3, 3] = 1
    return mat

def zRotationMatrix(ang):
    """ Create a rotation matrix for a rotation around the Z axis """
    mat = np.zeros((4,4))
    s = sin(ang)
    c = cos(ang)
    mat[0, 0] = c
    mat[1, 0] = -s
    mat[0, 1] = s
    mat[1, 1] = c
    mat[2, 2] = 1
    mat[3, 3] = 1
    return mat

def axisAngleToQuaternion(axisin, angle):
    """ Convert and axis and angle to a quaternion """
    mag = np.linalg.norm(axisin)
    axis = np.copy(axisin)/mag
    s = sin(angle/2)
    c = cos(angle/2)
    q = np.zeros(4)
    q[0] = axis[0] * s
    q[1] = axis[1] * s
    q[2] = axis[2] * s
    q[3] = c
    return q

def quaternionToAxisAngle(q):
    """ Convert a quaternion to an axis & angle """
    c = q[3]
    a = acos(c) * 2
    s = sqrt(1 - c * c)
    if fabs(s) < .00005:
        s = 1
    axis = (q[0]/s, q[1]/s, q[2]/s)
    return (axis, a)

def quaternionToRotationMatrix(q):
    """ Convert a quaternion to a rotation matrix """
    xx = q[0] * q[0]
    xy = q[0] * q[1]
    xz = q[0] * q[2]
    xw = q[0] * q[3]
    yy = q[1] * q[1]
    yz = q[1] * q[2]
    yw = q[1] * q[3]
    zz = q[2] * q[2]
    zw = q[2] * q[3]
    mat = np.zeros((4,4))
    mat[0, 0]  = 1 - 2 * ( yy + zz )
    mat[1, 0]  =     2 * ( xy - zw )
    mat[2, 0]  =     2 * ( xz + yw )
    mat[0, 1]  =     2 * ( xy + zw )
    mat[1, 1]  = 1 - 2 * ( xx + zz )
    mat[2, 1]  =     2 * ( yz - xw )
    mat[0, 2]  =     2 * ( xz - yw )
    mat[1, 2]  =     2 * ( yz + xw )
    mat[2, 2] = 1 - 2 * ( xx + yy )
    mat[3, 3] = 1
    return mat
