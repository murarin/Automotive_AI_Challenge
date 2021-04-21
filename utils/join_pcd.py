import os
import glob
import argparse
import subprocess
import sys
from pypcd import pypcd
import numpy as np

def string_split(data):
    return data.split('/')[-1]

def pcd_to_bin(pcd_file):
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

    size = int(header['POINTS'].split('#')[0]) * dtype.itemsize
    buf = pcd_file.read(size)
    lst = np.frombuffer(buf, dtype).tolist()
    out = []

    for row in lst:
        out.append((row[0], row[1], row[2], row[3]))

    pc = np.array(out).astype(np.float32)

    return pc

def pcd_concat(top_path, left_path, right_path):
    top_pc = pypcd.PointCloud.from_path(top_path)
    right_pc = pypcd.PointCloud.from_path(right_path)
    left_pc = pypcd.PointCloud.from_path(left_path)

    tr_pc = pypcd.cat_point_clouds(top_pc, right_pc)
    trl_pc = pypcd.cat_point_clouds(tr_pc, left_pc)

    return trl_pc

parser = argparse.ArgumentParser()

parser.add_argument('-d', '--dataset', type=str)

args = parser.parse_args()

pcd_path = os.path.join(args.dataset, "velodyne_pcd_v2v")


pcd_list = glob.glob(os.path.join(pcd_path, "top", "*.pcd"))

pcd_list = sorted(pcd_list)
pcd_list = map(string_split, pcd_list)

os.makedirs(os.path.join(args.dataset, "velodyne_pcd"), exist_ok=True)
os.makedirs(os.path.join(args.dataset, "velodyne"), exist_ok=True)

for data_name in pcd_list:

    print(data_name)

    # try:
    #     subprocess.run(("./join_pcd",
    #                     "-i",
    #                     "{}".format(os.path.join(pcd_path, "top", data_name)),
    #                     "{}".format(os.path.join(pcd_path, "left", data_name)),
    #                     "{}".format(os.path.join(pcd_path, "right", data_name)),
    #                     "-o",
    #                     "{}".format(os.path.join(args.dataset, "velodyne_pcd", data_name))), shell=False, check=True)
    #
    # except subprocess.CalledProcessError:
    #     print('外部プログラムの実行に失敗しました', file=sys.stderr)

    # join pcd
    pc_pcd = pcd_concat(os.path.join(pcd_path, "top", data_name), os.path.join(pcd_path, "left", data_name), os.path.join(pcd_path, "right", data_name))

    pcd_save_file = os.path.join(args.dataset, "velodyne_pcd", data_name)

    pc_pcd.save_pcd(pcd_save_file, compression="binary")


    # pcd to bin
    with open(os.path.join(args.dataset, "velodyne_pcd", data_name), "rb") as f:
        pc = pcd_to_bin(f)

    bin_file = os.path.join(args.dataset, "velodyne", data_name.split('.')[0]+ ".bin")

    pc.tofile(bin_file)

    print(data_name.split('.')[0]+ ".bin")






# training_pcd_path = os.path.join(args.dataset, "training", "velodyne_pcd_v2v")

# # testing
# # testing_pcd_path = os.path.join(args.dataset, "testing", "velodyne_pcd_v2v")

# training_pcd_list = glob.glob(os.path.join(training_pcd_path, "top", "*.pcd"))

# # testing
# # testing_pcd_list = glob.glob(os.path.join(testing_pcd_path, "top", "*.pcd"))

# training_pcd_list = sorted(training_pcd_list)
# training_pcd_list = map(string_split, training_pcd_list)

# # testing
# # testing_pcd_list = sorted(testing_pcd_list)
# # testing_pcd_list = map(string_split, testing_pcd_list)

# os.makedirs(os.path.join(args.dataset, "training", "velodyne_pcd"), exist_ok=True)

# # testing
# # os.makedirs(os.path.join(args.dataset, "testing", "velodyne_pcd"), exist_ok=True)

# for data_name in training_pcd_list:

#     print(os.path.join(training_pcd_path, "top", data_name))

#     try:
#         subprocess.run(("./join_pcd",
#                         "-i",
#                         "{}".format(os.path.join(training_pcd_path, "top", data_name)),
#                         "{}".format(os.path.join(training_pcd_path, "left", data_name)),
#                         "{}".format(os.path.join(training_pcd_path, "right", data_name)),
#                         "-o",
#                         "{}".format(os.path.join(args.dataset, "training", "velodyne_pcd", data_name))), shell=False, check=True)

#     except subprocess.CalledProcessError:
#         print('外部プログラムの実行に失敗しました', file=sys.stderr)


# # testing
# # for data_name in testing_pcd_list:
# #     result = subprocess.run('./join_pcd -i {} {} {} -o {}'.format(
# #         os.path.join(testing_pcd_path, "top", data_name),
# #         os.path.join(testing_pcd_path, "left", data_name),
# #         os.path.join(testing_pcd_path, "right", data_name),
# #         os.path.join(args.dataset, "testing", "velodyne_pcd", data_name)
# #     ))

# #     if result.returncode != 0:
# #         print("Error!!")
# #         os.exit()