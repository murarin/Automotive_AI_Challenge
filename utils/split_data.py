from sklearn.model_selection import train_test_split
import shutil
import os

data_path = "/data1/murarin/Automotive_AI_Challenge/datasets/Object_dataset"

datalist = []
with open(data_path + "/all/datalist.txt", 'r') as f:
    line = f.readline()

    while line:
        datalist.append(line.strip())
        line = f.readline()

train, test = train_test_split(datalist, test_size=0.1, shuffle=True)


for t in train:
    shutil.copy(data_path + "/all/rgb/{}.png".format(t), data_path + "/train/rgb/{}.png".format(t))
    shutil.copy(data_path + "/all/info2/{}.csv".format(t), data_path + "/train/info/{}.csv".format(t))

    with open(data_path + "/train/datalist.txt", 'a') as f:
        f.write(t + '\n')



for te in test:
    shutil.copy(data_path + "/all/rgb/{}.png".format(te), data_path + "/test/rgb/{}.png".format(te))
    shutil.copy(data_path + "/all/info2/{}.csv".format(te), data_path + "/test/info/{}.csv".format(te))

    with open(data_path + "/test/datalist.txt", 'a') as f:
        f.write(te + '\n')



