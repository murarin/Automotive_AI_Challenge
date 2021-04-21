import shutil
import cv2
import numpy as np
import glob
import os
import csv


def my_makedirs(path):
    if not os.path.isdir(path):
        os.makedirs(path)

#my_makedirs("/home/mprg/datas_new/color/red")
#my_makedirs("/home/mprg/datas_new/color/yellow")
#my_makedirs("/home/mprg/datas_new/color/blue")
#my_makedirs("/home/mprg/datas_new/color/unknown")

#my_makedirs("/home/mprg/datas_new/color/blue_rgb")
#my_makedirs("/home/mprg/datas_new/color/red_rgb")
#my_makedirs("/home/mprg/datas_new/color/yellow_rgb")
#my_makedirs("/home/mprg/datas_new/color/unknown_rgb")


#tlr_list_all = glob.glob("/home/mprg/datas_new/rgb_cropD/*")

my_makedirs("/home/mprg/datas_new/color/hinan")

r_list = glob.glob("/home/mprg/datas_new/color/red/*")
b_list = glob.glob("/home/mprg/datas_new/color/blue/*")
y_list = glob.glob("/home/mprg/datas_new/color/yellow/*")
u_list = glob.glob("/home/mprg/datas_new/color/unknown/*")

print(len(u_list))

a_list = glob.glob("/home/mprg/Desktop/ar/*")

def check_the_number_of_data():
    for a in a_list:
        basename = os.path.basename(a)
        if os.path.isfile("/home/mprg/datas_new/color/yellow/%s" % basename):
            print("/home/mprg/datas_new/color/yellow/%s" % basename)
            print(a)

        elif os.path.isfile("/home/mprg/datas_new/color/red/%s" % basename):
            print("/home/mprg/datas_new/color/red/%s" % basename)
            print(a)

        elif os.path.isfile("/home/mprg/datas_new/color/unknown/%s" % basename):
            print("/home/mprg/datas_new/color/unknown/%s" % basename)
            print(a)
            os.remove("/home/mprg/datas_new/color/unknown/%s" % basename)


    print(len(r_list) + len(b_list) + len(y_list) + len(u_list))

    # img_number = 0
    # while True:
    #     semseg_basename = os.path.basename(u_list[img_number])
    #     basename = semseg_basename.split('_')[0]
    #     basename1 = semseg_basename.split('_')[1]
    #     rgb_tlr_number = semseg_basename.split('_')[-1].split('.')[0]

    #     shutil.copy("/home/mprg/datas_new/all_rgb/%s_%s.png" % (basename, basename1), "/home/mprg/datas_new/color/unknown_rgb/")

    #     img_number += 1



def sorting_confirmation():

    img_number = 0
    while True:
        cv2.namedWindow("cropD",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
        cv2.namedWindow("rgb")
        cv2.namedWindow("semseg",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

        cv2.moveWindow("cropD", 1920 + 1920 - 700, 0)
        cv2.moveWindow("rgb", 1920 + 500, 0)
        cv2.moveWindow("semseg", 1920 + 1920 - 700, 350)

        img = cv2.imread(r_list[img_number])

        semseg_basename = os.path.basename(r_list[img_number])

        semseg_img = cv2.imread("/home/mprg/datas_new/semseg_cropD/%s" % semseg_basename)

        rgb_basename = semseg_basename.split('_')[0]
        rgb_basename1 = semseg_basename.split('_')[1]
        rgb_tlr_number = semseg_basename.split('_')[-1].split('.')[0]

        rgb = cv2.imread("/home/mprg/datas_new/all_rgb/%s_%s.png" % (rgb_basename, rgb_basename1))

        with open("/home/mprg/datas_new/all_tlr_info/%s_%s.csv" % (rgb_basename, rgb_basename1)) as f:
            reader = csv.reader(f)
            l = [row for row in reader]

            tlr_rec = l[int(rgb_tlr_number) + 1]
            cv2.rectangle(rgb, (int(tlr_rec[17]), int(tlr_rec[18])), (int(tlr_rec[19]), int(tlr_rec[20])), (0, 0, 255), 1)

        cv2.imshow("cropD", img)
        cv2.imshow("semseg", semseg_img)
        cv2.imshow("rgb", rgb)

        key = cv2.waitKey(0)

        if key == ord('d'):
            img_number += 1

        elif key == ord('q'):
            break

        elif key == ord('a'):
            img_number -= 1
            continue

        else:
            continue

        print("確認完了まであと: %s" % str(len(r_list) - img_number))


        cv2.destroyAllWindows()

    cv2.destroyAllWindows()


def sorting():

    tlr_list = []
    for conf_tlr in tlr_list_all:
        basename = os.path.basename(conf_tlr)

        if os.path.isfile("/home/mprg/datas_new/color/red/%s" % basename) \
            or os.path.isfile("/home/mprg/datas_new/color/yellow/%s" % basename) \
            or os.path.isfile("/home/mprg/datas_new/color/blue/%s" % basename) \
            or os.path.isfile("/home/mprg/datas_new/color/unknown/%s" % basename):

            continue

        else:
            print(conf_tlr)
            tlr_list.append(conf_tlr)

    print("仕分け完了まで残り: %s" % len(tlr_list))

    img_number = 0


    while True:

        cv2.namedWindow("now",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
        cv2.namedWindow("next",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
        cv2.namedWindow("rgb")

        cv2.moveWindow("now", 1920 + 1920 - 700, 0)
        cv2.moveWindow("next", 1920 + 1920 - 300, 0)
        cv2.moveWindow("rgb", 1920 + 500, 0)


        cv2.namedWindow("semseg_now",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)
        cv2.namedWindow("semseg_next",cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL)

        cv2.moveWindow("semseg_now", 1920 + 1920 - 700, 350)
        cv2.moveWindow("semseg_next", 1920 + 1920 - 300, 350)

        img = cv2.imread(tlr_list[img_number])
        img2 = cv2.imread(tlr_list[img_number + 1])

        semseg_basename = os.path.basename(tlr_list[img_number])
        semseg_basename2 = os.path.basename(tlr_list[img_number + 1])

        semseg_img = cv2.imread("/home/mprg/datas_new/semseg_cropD/%s" % semseg_basename)
        semseg_img2 = cv2.imread("/home/mprg/datas_new/semseg_cropD/%s" % semseg_basename2)

        rgb_basename = semseg_basename.split('_')[0]
        rgb_basename1 = semseg_basename.split('_')[1]
        rgb_tlr_number = semseg_basename.split('_')[-1].split('.')[0]

        rgb = cv2.imread("/home/mprg/datas_new/all_rgb/%s_%s.png" % (rgb_basename, rgb_basename1))

        with open("/home/mprg/datas_new/all_tlr_info/%s_%s.csv" % (rgb_basename, rgb_basename1)) as f:
            reader = csv.reader(f)
            l = [row for row in reader]

            tlr_rec = l[int(rgb_tlr_number) + 1]
            cv2.rectangle(rgb, (int(tlr_rec[17]), int(tlr_rec[18])), (int(tlr_rec[19]), int(tlr_rec[20])), (0, 0, 255), 1)

        cv2.imshow("now", img)
        cv2.imshow("next", img2)

        cv2.imshow("semseg_now", semseg_img)
        cv2.imshow("semseg_next", semseg_img2)
        cv2.imshow("rgb", rgb)

        print(tlr_list[img_number])

        key = cv2.waitKey(0)

        if key == ord('f'):
            shutil.copy(tlr_list[img_number], "/home/mprg/datas_new/color/red/")

        elif key == ord('k'):
        shutil.copy(tlr_list[img_number], "/home/mprg/datas_new/color/yellow/")

        elif key == ord('j'):
            shutil.copy(tlr_list[img_number], "/home/mprg/datas_new/color/blue/")

        elif key == 32:
            shutil.copy(tlr_list[img_number], "/home/mprg/datas_new/color/unknown/")

        elif key == ord('q'):
            break

        elif key == ord('a'):
            img_number -= 1
            basename = os.path.basename(tlr_list[img_number])

            remove_file = glob.glob("/home/mprg/datas_new/color/**/%s" % basename, recursive=True)

            os.remove(remove_file[0])
 
            continue

        else:
            continue

        img_number += 1

        print("仕分け完了まで残り: %s" % str(len(tlr_list) - img_number))

        cv2.destroyAllWindows()

    cv2.destroyAllWindows()

