import lgsvl
from lgsvl.utils import transform_to_matrix
import os
import sys
import math
import time
import random
import numpy as np
from PIL import Image
import argparse
import glob

def parse_pcd_file(pcd_file):
        header = {}
        while True:
            ln = pcd_file.readline().strip()
            field = ln.decode('ascii').split(' ', 1)
            header[field[0]] = field[1]
            if ln.startswith(b"DATA"):
                break

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.uint8),
        ])

        # print(header)

        # print(header['POINTS'].split('#')[0])

        size = int(header['POINTS'].split('#')[0]) * dtype.itemsize
        buf = pcd_file.read(size)
        lst = np.frombuffer(buf, dtype).tolist()
        out = []
        for row in lst:
            out.append((row[0], row[1], row[2], row[3] / 255))
        pc = np.array(out).astype(np.float32)

        return pc

parser = argparse.ArgumentParser()

parser.add_argument('-d', '--dataset', type=str)

args = parser.parse_args()

pcd_list = glob.glob(os.path.join(args.dataset, "velodyne_pcd", "left", "*.pcd"))

os.makedirs(os.path.join(args.dataset, "velodyne"), exist_ok=True)

for pcd_name in pcd_list:
    with open(pcd_name, "rb") as f:
        pc = parse_pcd_file(f)

    bin_file = os.path.join(args.dataset, "velodyne", pcd_name.split('/')[-1].split('.')[0] + ".bin")
    pc.tofile(bin_file)

    print(pcd_name.split('/')[-1].split('.')[0] + ".bin")
