import glob
import os

rgb_list = glob.glob("./rgb/*")

print(len(rgb_list))
for rgb in rgb_list:
    file_name = os.path.splitext(os.path.basename(rgb))[0]

    with open("datalist.txt", "a") as f:
        f.write(file_name + '\n')
