import cv2
import numpy as np
import glob
import os
import time

WIDTH = 100
HEIGHT = 150

def scale_to_width(img, width):
    scale = width / img.shape[1]
    return cv2.resize(img, dsize=None, fx=scale, fy=scale)


def scale_to_height(img, height):
    scale = height / img.shape[0]
    return cv2.resize(img, dsize=None, fx=scale, fy=scale)



path = "/home/mprg/datas_new/color/"
out_path = "/home/mprg/datas_new/traffic_light_dataset/all/"

all_red = glob.glob(path + "red/*")
all_blue = glob.glob(path + "blue/*")
all_yellow = glob.glob(path + "yellow/*")
all_unknown = glob.glob(path + "unknown/*")

start_t = time.time()
for tlr in all_yellow:
    img = cv2.imread(tlr)

    h, w, c = img.shape

    if h > HEIGHT:
        img = scale_to_height(img, HEIGHT)
        h, w, c = img.shape

    if w > WIDTH:
        img = scale_to_width(img, WIDTH)
        h, w, c = img.shape

    if h < HEIGHT or w < WIDTH:
        brank = np.zeros((HEIGHT, WIDTH, 3))

        #print(brank.shape)
        #print(img.shape)

        left_x = round(WIDTH / 2.0 - w / 2.0)
        left_y = round(HEIGHT / 2.0 - h / 2.0)

        #print(h)
        #print(w)

        brank[left_y:left_y + h, left_x:left_x + w] = img

        img = brank

    basename = os.path.basename(tlr)
    cv2.imwrite(out_path + "yellow/{}".format(basename), img)

elapsed_time = time.time() - start_t
elapsed_time_1 = elapsed_time / len(all_yellow)
print("elapsed_time:{0}".format(elapsed_time) + "[sec]")
print("elapsed_time_1:{0}".format(elapsed_time_1) + "[sec]")

#ピクセル確認
'''
img = cv2.imread("/home/mprg/datas_new_backup/rgb_cropD/1585019916_9000_0.png")
img1 = cv2.imread("/home/mprg/datas_new/traffic_light_dataset/all/red/1585019916_9000_0.png")

h, w, c = img.shape

left_x = round(WIDTH / 2.0 - w / 2.0)
left_y = round(HEIGHT / 2.0 - h / 2.0)

print(img[5,5])
print(img1[left_y + 5,left_x + 5])
'''





