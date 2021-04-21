import numpy as np
import glob
from sklearn.model_selection import train_test_split
import os
import shutil

light_color = np.array(["blue", "red", "yellow", "unknown"])

path = "/home/mprg/datas_new/traffic_light_dataset/all/"
out_path = "/home/mprg/datas_new/traffic_light_dataset/"

for lc in light_color:
    light_all = glob.glob("/home/mprg/datas_new/traffic_light_dataset/all/{}/*".format(lc))

    light_train, light_test = train_test_split(light_all, test_size=0.2, shuffle=True)

    for tra in light_train:
        basename = os.path.basename(tra)
        shutil.copy(tra, out_path + "train/{}/{}".format(lc, basename))

    for tes in light_test:
        basename = os.path.basename(tes)
        shutil.copy(tes, out_path + "test/{}/{}".format(lc, basename))



