import csv
import glob
import os
import cv2


info_list = glob.glob("./info/*")

for info in info_list:
    basename = os.path.basename(info)

    with open(info) as f:
        reader = next(csv.reader(f))

        with open("./info2/{}".format(basename), 'w') as fw:
            writer = csv.writer(fw)

            for row in csv.reader(f):
                #if row[1] == "Pedestrian":
                #     continue

                xmin = float(row[3]) - float(row[5]) / 2.0
                ymin = float(row[4]) - float(row[6]) / 2.0
                xmax = float(row[3]) + float(row[5]) / 2.0
                ymax = float(row[4]) + float(row[6]) / 2.0

                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(719, xmax) #width - 1
                ymax = min(539, ymax) #height - 1

                if row[1] == "Car":
                    label = 0

                else:
                    print("error")
                    sys.exit()

                writer.writerow([xmin, ymin, xmax, ymax, label])

