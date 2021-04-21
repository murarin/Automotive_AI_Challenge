import lgsvl
from lgsvl.utils import transform_to_matrix
import os
import sys
import math
from math import sin, cos, tan, radians
import time
import random
import numpy as np
from PIL import Image
import argparse


def save_ground_truth(self):
    t0 = time.time()
    labels = self.parse_ground_truth()
    txt_file = os.path.join(self.LABEL_PATH, self.get_filename("txt"))
    with open(txt_file, "w") as f:
        for label in labels:
            f.write("{}\n".format(label))
        print("{} ({:.3f} s)".format(txt_file, time.time() - t0))

def parse_ground_truth(self):
        camera_mat = transform_to_matrix(self.sensor_camera.transform)
        ego_mat = transform_to_matrix(self.ego_state.transform)
        tf_mat = np.dot(np.linalg.inv(ego_mat), np.linalg.inv(camera_mat))

        labels = []
        for npc, npc_state in zip(self.npcs, self.npcs_state):

            # print("a", npc.transform)
            # print("b", npc_state.transform)


            # npc_tf = self.get_npc_tf_in_cam_space(npc_state.transform, tf_mat)
            npc_tf = self.get_npc_tf_in_cam_space(npc.transform, tf_mat)

            location = self.get_location(npc_tf)
            rotation_y = self.get_rotation_y(npc_tf)
            height, width, length = self.get_dimension(npc.bounding_box)
            alpha = self.get_alpha(location, rotation_y)

            corners_3D = self.get_corners_3D(location, rotation_y, (height, width, length))
            corners_2D = self.project_3D_to_2D(corners_3D)

            p_min, p_max = corners_2D[:, 0], corners_2D[:, 0]
            for i in range(corners_2D.shape[1]):
                p_min = np.minimum(p_min, corners_2D[:, i])
                p_max = np.maximum(p_max, corners_2D[:, i])

            left, top = p_min
            right, bottom = p_max

            if npc.name in ['Sedan', 'SUV', 'Jeep', 'Hatchback', 'BoxTruck', 'SchoolBus']:
                label = "Car -1 -1 {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}" \
                            .format(alpha, left, top, right, bottom, height, width, length, location[0], location[1], location[2], rotation_y)

            else:
                # if height > 10.0:
                #     height = 1.70

                label = "Pedestrian -1 -1 {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}" \
                            .format(alpha, left, top, right, bottom, height, width, length, location[0], location[1], location[2], rotation_y)

                print("{}: {}".format(npc.name, height))

            labels.append(label)

        return labels
