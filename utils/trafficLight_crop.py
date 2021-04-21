import cv2
import numpy
import glob
import csv
import os

def my_makedirs(path):
    if not os.path.isdir(path):
        os.makedirs(path)

my_makedirs("/home/mprg/datas_new/rgb_cropD")
my_makedirs("/home/mprg/datas_new/semseg_cropD")

my_makedirs("/home/mprg/datas_new/rgb_cropL")
my_makedirs("/home/mprg/datas_new/semseg_cropL")

my_makedirs("/home/mprg/datas_new/rgb_cropLL")
my_makedirs("/home/mprg/datas_new/semseg_cropLL")

data_path = "/home/mprg/datas_new/"

file_num = 0

for i in range(14):

    tlr_info_list = glob.glob("/home/mprg/datas_new/data%d/trafficLight_info/*" % i)

    for tlr_info in tlr_info_list:
        file_num += 1

        file_name = os.path.splitext(os.path.basename(tlr_info))[0]

        print("%d: %s" % (file_num, tlr_info))

        if not os.path.isfile("/home/mprg/datas_new/data%d/rgb/%s.png" % (i, file_name)) \
            or not os.path.isfile("/home/mprg/datas_new/data%d/semseg/%s.png" % (i, file_name)):
            continue

        rgb_img = cv2.imread("/home/mprg/datas_new/data%d/rgb/%s.png" % (i, file_name))
        semseg_img = cv2.imread("/home/mprg/datas_new/data%d/semseg/%s.png" % (i, file_name))

        with open(tlr_info)as f:
            reader = next(csv.reader(f))

            tlr_num = 0
            for row in csv.reader(f):

                topLeft_x_D = int(row[17])
                topLeft_y_D = int(row[18])
                botRight_x_D = int(row[19])
                botRight_y_D = int(row[20])

                rgb_crop_D = rgb_img[topLeft_y_D:botRight_y_D, topLeft_x_D:botRight_x_D]
                semseg_crop_D = semseg_img[topLeft_y_D:botRight_y_D, topLeft_x_D:botRight_x_D]

                height, width, channels = rgb_crop_D.shape

                height_L = height * 0.25
                width_L = width * 0.25

                height_LL = height * 0.5
                width_LL = width * 0.5

                topLeft_x_L = max(0, topLeft_x_D - width_L)
                topLeft_y_L = max(0, topLeft_y_D - height_L)
                botRight_x_L = min(720, botRight_x_D + width_L)
                botRight_y_L = min(540, botRight_y_D + height_L)

                topLeft_x_L = int(topLeft_x_L)
                topLeft_y_L = int(topLeft_y_L)
                botRight_x_L = int(botRight_x_L)
                botRight_y_L = int(botRight_y_L)

                topLeft_x_LL = max(0, topLeft_x_D - width_LL)
                topLeft_y_LL = max(0, topLeft_y_D - height_LL)
                botRight_x_LL = min(720, botRight_x_D + width_LL)
                botRight_y_LL = min(540, botRight_y_D + height_LL)

                topLeft_x_LL = int(topLeft_x_LL)
                topLeft_y_LL = int(topLeft_y_LL)
                botRight_x_LL = int(botRight_x_LL)
                botRight_y_LL = int(botRight_y_LL)


                rgb_crop_L = rgb_img[topLeft_y_L:botRight_y_L, topLeft_x_L:botRight_x_L]
                semseg_crop_L = semseg_img[topLeft_y_L:botRight_y_L, topLeft_x_L:botRight_x_L]

                rgb_crop_LL = rgb_img[topLeft_y_LL:botRight_y_LL, topLeft_x_LL:botRight_x_LL]
                semseg_crop_LL = semseg_img[topLeft_y_LL:botRight_y_LL, topLeft_x_LL:botRight_x_LL]

                cv2.imwrite("/home/mprg/datas_new/rgb_cropD/%s_%d.png" % (file_name, tlr_num), rgb_crop_D)
                cv2.imwrite("/home/mprg/datas_new/semseg_cropD/%s_%d.png" % (file_name, tlr_num), semseg_crop_D)

                cv2.imwrite("/home/mprg/datas_new/rgb_cropL/%s_%d.png" % (file_name, tlr_num), rgb_crop_L)
                cv2.imwrite("/home/mprg/datas_new/semseg_cropL/%s_%d.png" % (file_name, tlr_num), semseg_crop_L)

                cv2.imwrite("/home/mprg/datas_new/rgb_cropLL/%s_%d.png" % (file_name, tlr_num), rgb_crop_LL)
                cv2.imwrite("/home/mprg/datas_new/semseg_cropLL/%s_%d.png" % (file_name, tlr_num), semseg_crop_LL)

                tlr_num += 1


