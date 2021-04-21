import os
import glob
from sklearn.model_selection import train_test_split

def string_split(data):
    return data.split('/')[-1].split('.')[0]

kitti_dataset_path = "/home/mprg/aichallenge_final_datasets/lgsvl_kitti_full/"

os.makedirs(os.path.join(kitti_dataset_path, "ImageSets"), exist_ok=True)

train_data_list = glob.glob(os.path.join(kitti_dataset_path, "training", "label_2", "*.txt"))
test_data_list = glob.glob(os.path.join(kitti_dataset_path, "testing", "label_2", "*.txt"))

train_list, val_list = train_test_split(train_data_list, test_size=0.2)
trainval_list = train_data_list
test_list = test_data_list

train_list = sorted(train_list)
trainval_list = sorted(trainval_list)
val_list = sorted(val_list)
test_list = sorted(test_list)

train_list = map(string_split, train_list)
trainval_list = map(string_split, trainval_list)
val_list = map(string_split, val_list)
test_list = map(string_split, test_list)

with open(os.path.join(kitti_dataset_path, "train.txt"), "w") as f:
    f.write('\n'.join(train_list))

with open(os.path.join(kitti_dataset_path, "trainval.txt"), "w") as f:
    f.write('\n'.join(trainval_list))

with open(os.path.join(kitti_dataset_path, "val.txt"), "w") as f:
    f.write('\n'.join(val_list))

with open(os.path.join(kitti_dataset_path, "test.txt"), "w") as f:
    f.write('\n'.join(test_list))

# print(len(train_data_list))