
import argparse
import struct
import os.path

def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="Scan file")
    return parser.parse_args()

def processFile(filename):
    inp = open(filename, "r")
    trans=struct.unpack("<3d", inp.read(3*8))
    print trans
    quaternion=struct.unpack("<4d", inp.read(4*8))
    print quaternion
    numPts=struct.unpack("<i", inp.read(4))
    print numPts
    cnt = 0
    (base, ext) = os.path.splitext(filename)
    out = open(base+".xyz", "w")
    while 1:
        buf = inp.read(3*4)
        if buf == '':
            break
        pos = struct.unpack("<3f", buf)
        print >>out, pos[0], pos[1], pos[2]
        cnt += 1
    print "Read %d Points" % (cnt,)

if __name__ == '__main__':
    args = parseArgs()
    processFile(args.file)
