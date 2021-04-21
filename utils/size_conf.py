import cv2
import glob

path = "/home/mprg/datas_new/rgb_cropD/*"

all_tlr = glob.glob(path)

max_w = 0
max_h = 0
min_w = 1000
min_h = 1000

for tlr in all_tlr:
    img = cv2.imread(tlr)

    h, w, c = img.shape

    if max_w < w:
        max_w = w
        max_w_path = tlr

    if max_h < h:
        max_h = h
        max_h_path = tlr

    if min_w > w:
        min_w = w
        min_w_path = tlr

    if min_h > h:
        min_h = h
        min_h_path = tlr

    #max_w = max(max_w, w)
    #max_h = max(max_h, h)
    #min_w = min(min_w, w)
    #min_h = min(min_h, h)


print("max_w: {}".format(max_w))
print("max_h: {}".format(max_h))
print("min_w: {}".format(min_w))
print("min_h: {}".format(min_h))

print("max_w_path: {}".format(max_w_path))
print("max_h_path: {}".format(max_h_path))
print("min_w_path: {}".format(min_w_path))
print("min_h_path: {}".format(min_h_path))

'''
max_w: 117
max_h: 169
min_w: 5
min_h: 7
max_w_path: /home/mprg/datas_new/rgb_cropD/1585102424_8032_0.png
max_h_path: /home/mprg/datas_new/rgb_cropD/1585102424_8032_0.png
min_w_path: /home/mprg/datas_new/rgb_cropD/1585113866_7599_1.png
min_h_path: /home/mprg/datas_new/rgb_cropD/1585113866_7599_1.png
'''
