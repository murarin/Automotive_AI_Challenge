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
from pypcd import pypcd
import shutil

def read_calib_file(filepath):
    data = {}
    with open(filepath, 'r') as f:
        for line in f.readlines():
            line = line.rstrip()
            if len(line)==0: continue
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass
    return data

def pcd_to_pcd(pcd_path, v2v):
    # print(pcd_path)

    pc = pypcd.PointCloud.from_path(pcd_path)

    v2v = np.reshape(v2v, [3, 4])
    v2v = np.vstack([v2v, [0, 0, 0, 1]])

    # print(len(pc.pc_data['intensity']))

    for i, row in enumerate(pc.pc_data):
        xyz1 = np.matrix([[row[0]],
                        [row[1]],
                        [row[2]],
                        [1]
                        ])

        # print(xyz1.shape)

        out_xyz1 = np.dot(v2v, xyz1)
        # print(out_xyz1)

        pc.pc_data['x'][i] = out_xyz1[0]
        pc.pc_data['y'][i] = out_xyz1[1]
        pc.pc_data['z'][i] = out_xyz1[2]
        pc.pc_data['intensity'][i] = row[3] / 255.

        # out.append((row[0], row[1], row[2], row[3] / 255))
        # out.append((out_xyz1[0], out_xyz1[1], out_xyz1[2], row[3] / 255))

    # pc = np.array(out).astype(np.float32)

    # print(min(pc.pc_data['z']))

    return pc



def parse_pcd_file(pcd_file, v2v):

    # print(v2v)

    v2v = np.reshape(v2v, [3, 4])
    v2v = np.vstack([v2v, [0, 0, 0, 1]])
    v2v = np.matrix(v2v)
    # print(v2v.shape)



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
        # print(row[0])
        xyz1 = np.matrix([[row[0]],
                        [row[1]],
                        [row[2]],
                        [1]
                        ])

        # print(xyz1.shape)

        out_xyz1 = np.dot(v2v, xyz1)
        # print(out_xyz1)

        # out.append((row[0], row[1], row[2], row[3] / 255))
        out.append((out_xyz1[0], out_xyz1[1], out_xyz1[2], row[3] / 255))

    pc = np.array(out).astype(np.float32)

    return pc

parser = argparse.ArgumentParser()

parser.add_argument('-d', '--dataset', type=str)
parser.add_argument('-v', '--velo', type=str)

args = parser.parse_args()

for args.velo in ["left", "right"]:

    pcd_list = glob.glob(os.path.join(args.dataset, "velodyne_pcd_master", args.velo, "*.pcd"))
    # pc_top = pypcd.PointCloud.from_path(os.path.join(args.dataset, "velodyne_pcd_master", "top", "000000.pcd"))
    # print(min(pc_top.pc_data['z']))

    # os.makedirs(os.path.join(args.dataset, "velodyne_v2v"), exist_ok=True)
    # os.makedirs(os.path.join(args.dataset, "velodyne_pcd_v2v"), exist_ok=True)

    os.makedirs(os.path.join(args.dataset, "velodyne_v2v", args.velo), exist_ok=True)
    os.makedirs(os.path.join(args.dataset, "velodyne_pcd_v2v", args.velo), exist_ok=True)

    for pcd_name in pcd_list:

        calib_file = os.path.join(args.dataset, "calib", pcd_name.split('/')[-1].split('.')[0] + ".txt")
        calib_data = read_calib_file(calib_file)

        # pcd_to_pcd
        if args.velo == 'left':
            pc_pcd = pcd_to_pcd(pcd_name, calib_data['Tr_leftVelo_to_topVelo'])

        elif args.velo == 'right':
            pc_pcd = pcd_to_pcd(pcd_name, calib_data['Tr_rightVelo_to_topVelo'])

        else:
            print("left or right")
            os.exit()

        pcd_save_file = os.path.join(args.dataset, "velodyne_pcd_v2v", args.velo, pcd_name.split('/')[-1].split('.')[0] + ".pcd")

        pc_pcd.save_pcd(pcd_save_file, compression="binary")

        print(pcd_name.split('/')[-1].split('.')[0] + ".pcd")



        # pcd_to_bin
        with open(pcd_name, "rb") as f:

            if args.velo == 'left':
                pc = parse_pcd_file(f, calib_data['Tr_leftVelo_to_topVelo'])

            elif args.velo == 'right':
                pc = parse_pcd_file(f, calib_data['Tr_rightVelo_to_topVelo'])

            else:
                print("left or right")
                os.exit()

        bin_file = os.path.join(args.dataset, "velodyne_v2v", args.velo, pcd_name.split('/')[-1].split('.')[0] + ".bin")

        pc.tofile(bin_file)

        print(pcd_name.split('/')[-1].split('.')[0] + ".bin")

# topのコピー
shutil.copytree(os.path.join(args.dataset, "velodyne_pcd_master", "top"), os.path.join(args.dataset, "velodyne_pcd_v2v", "top"))
shutil.copytree(os.path.join(args.dataset, "velodyne_master", "top"), os.path.join(args.dataset, "velodyne_v2v", "top"))
