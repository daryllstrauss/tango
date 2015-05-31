
import argparse
import struct
import os.path
import numpy as np
from matmath import getPC2WorldMatrix, quaternionToAxisAngle
from math import acos, pi, sin, degrees
import json

# Point Cloud Frame
# With the unit in landscape orientation, screen facing the user:
# +X points toward the user's right.
# +Y points toward the bottom of the screen.
# +Z points in the direction of the camera's optical axis, and is measured
# perpendicular to the plane of the camera.
# The origin is the focal center of the color camera.
# The output is in units of meters.

# Start of Service Frame
# Based on the position of the device at the start of service
# with the screen facing towards the user
# +X is to the right
# +Y is the direction of the cameras
# +Z is up versus gravity

# The Device Frame
# Devices have a default orientation, for Tango that's landscape
# +X is to the right of the device
# +Y is up on the device
# +Z is the screen towards the user

# The pose is the rotation and translation of subsequent frames against
# the SoS Frame to Device Frame
# Translation (x, y, z) in meters
# Rotation as a quaternion (x, y, z, w)


def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="Scan file")
    return parser.parse_args()

def toHex(array):
    return ''.join(format(ord(x), '02x') for x in array)

def processFile(filename):
    inp = open(filename, "r")
    trans = np.array(struct.unpack("<3d", inp.read(3*8)))
    print "Offset", trans
    quaternion = np.array(struct.unpack("<4d", inp.read(4*8)))
    print "Quaternion", quaternion
    (axis, angle) = quaternionToAxisAngle(quaternion)
    print "Axis", axis, "Angle", degrees(angle)
    numPts=struct.unpack("<i", inp.read(4))[0]
    print "Numpts", numPts
    cvtmat = getPC2WorldMatrix(trans, quaternion)
    (base, ext) = os.path.splitext(filename)
    out = open(base+".xyz", "w")
    data = [];
    for i in xrange(numPts):
        buf = inp.read(3*4)
        if i == 0 :
            print "Hex", toHex(buf)
        if buf == '':
            break
        cam = np.array(struct.unpack("<3f", buf))
        dev = np.array([cam[0], cam[1], cam[2], 1])
        imu = dev.dot(cvtmat)
        print >>out, imu[0], imu[1], imu[2]
        data.append([imu[0], imu[1], imu[2]])
    ijPts = struct.unpack("<i", inp.read(4))[0]
    print "ijPts", ijPts
    out.close()
    out2 = open(base+".json", "w")
    json.dump({'data': data}, out2)
    out2.close()

if __name__ == '__main__':
    args = parseArgs()
    processFile(args.file)
