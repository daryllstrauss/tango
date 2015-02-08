
import unittest
import matmath
import numpy as np
import math

class TestMatrix(unittest.TestCase):

    def testRotX(self):
        mat = matmath.xRotationMatrix(math.radians(90))
        pt = np.array([1, 0, 0, 1])
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [1, 0, 0, 1])
        pt = np.array([0, 1, 0, 1])
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [0, 0, 1, 1])
        pt = np.array([0, 0, 1, 1])
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [0, -1, 0, 1])

    def testRotY(self):
        pt = np.array([0, 0, 1, 1])
        mat = matmath.yRotationMatrix(math.radians(90))
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [1, 0, 0, 1])

    def testRotZ(self):
        pt = np.array([1, 0, 0, 1])
        mat = matmath.zRotationMatrix(math.radians(90))
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [0, 1, 0, 1])

    def testQuaternionMatrix(self):
        q = matmath.axisAngleToQuaternion([1, 0, 0], np.radians(90))
        qmat = matmath.quaternionToRotationMatrix(q)
        rmat = matmath.xRotationMatrix(math.radians(90))
        np.testing.assert_almost_equal(qmat, rmat)

        q = matmath.axisAngleToQuaternion([0, 1, 0], np.radians(90))
        qmat = matmath.quaternionToRotationMatrix(q)
        rmat = matmath.yRotationMatrix(math.radians(90))
        np.testing.assert_almost_equal(qmat, rmat)

        q = matmath.axisAngleToQuaternion([0, 0, 1], np.radians(90))
        qmat = matmath.quaternionToRotationMatrix(q)
        rmat = matmath.zRotationMatrix(math.radians(90))
        np.testing.assert_almost_equal(qmat, rmat)


    def testMultipleRotates(self):
        r1 = matmath.xRotationMatrix(np.radians(90))
        r2 = matmath.zRotationMatrix(np.radians(90))
        mat = r1.dot(r2)
        pt = np.array([0, 0, 1, 1])
        npt = pt.dot(mat)
        np.testing.assert_almost_equal(npt, [1, 0, 0, 1])

    def test2M(self):
        # 2 Meters away depth scan
        pt = np.array([0, 0, 2, 1])
        print "PC", pt
        mat = matmath.pcToSoSMatrix()
        npt = pt.dot(mat)
        print "SoS  ", npt
        trans = np.array([0, 0, 0])
        quaternion = matmath.axisAngleToQuaternion([1, 0, 0], np.radians(90))
        mat = matmath.getPC2WorldMatrix(trans, quaternion)
        npt = pt.dot(mat)
        print "Device", npt
        pt = np.array([0, 1, 2, 1])
        print "PC", pt
        mat = matmath.pcToSoSMatrix()
        npt = pt.dot(mat)
        print "SoS  ", npt
        mat = matmath.getPC2WorldMatrix(trans, quaternion)
        npt = pt.dot(mat)
        print "Device", npt

