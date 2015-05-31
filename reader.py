
import argparse
import struct
import os.path
import random
import numpy as np
from matmath import getPC2WorldMatrix, quaternionToAxisAngle
from math import acos, pi, sin, degrees, atan
import json
from PIL import Image

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

def writeHeader(out, numPts):
    print >>out, "ply"
    print >>out, "format ascii 1.0"
    print >>out, "comment Created by Daryll Strauss"
    print >>out, "element vertex", numPts
    print >>out, "property float x"
    print >>out, "property float y"
    print >>out, "property float z"
    print >>out, "property uchar red"
    print >>out, "property uchar green"
    print >>out, "property uchar blue"
    print >>out, "end_header"

def findColor(pic, hFOV, vFOV, pt):
    angx = atan(pt[0]/pt[2])
    angy = atan(pt[1]/pt[2])
    xperc = (angx + hFOV/2)/hFOV
    yperc = (angy + vFOV/2)/vFOV
    xpos = int(xperc * pic.size[0]+0.5)
    ypos = int(yperc * pic.size[1]+0.5)
    pixel = pic.getpixel((xpos, ypos))
    r = pixel[0]
    g = pixel[1]
    b = pixel[2]
    return (r, g, b)

def processFile(filename):
    inp = open(filename, "r")
    pic = Image.open("20150531/tango00001.png")

    # Read translation
    trans = np.array(struct.unpack("<3d", inp.read(3*8)))
    print "Offset", trans

    # Read rotation
    quaternion = np.array(struct.unpack("<4d", inp.read(4*8)))
    print "Quaternion", quaternion
    (axis, angle) = quaternionToAxisAngle(quaternion)
    print "Axis", axis, "Angle", degrees(angle)

    # Read Field of VIew
    (hFOV, vFOV) = struct.unpack("<2d", inp.read(2*8))
    print "hFOV=", hFOV, "vFOV=", vFOV

    # Read points
    numPts=struct.unpack("<i", inp.read(4))[0]
    print "Numpts", numPts
    cvtmat = getPC2WorldMatrix(trans, quaternion)
    (base, ext) = os.path.splitext(filename)
    out = open(base+".ply", "w")
    writeHeader(out, numPts)
    data = [];
    for i in xrange(numPts):
        buf = inp.read(3*4)
        if buf == '':
            break
        cam = np.array(struct.unpack("<3f", buf))
        dev = np.array([cam[0], cam[1], cam[2], 1])
        imu = dev.dot(cvtmat)
        (r, g, b) = findColor(pic, hFOV, vFOV, dev)
        print >>out, imu[0], imu[1], imu[2], r, g, b
        data.append((imu[0], imu[1], imu[2]))

    # Read IJ (Not Yet Implemented)
    ijPts = struct.unpack("<i", inp.read(4))[0]
    print "ijPts", ijPts
    out.close()
    out2 = open(base+".json", "w")
    json.dump({'points': data}, out2)
    out2.close()

if __name__ == '__main__':
    args = parseArgs()
    processFile(args.file)
