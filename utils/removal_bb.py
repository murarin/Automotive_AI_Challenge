import os
import numpy as np
from OpenGL.GL import glLineWidth
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import glob
import argparse
import time


class Object3d(object):
    ''' 3d object label '''
    def __init__(self, label_file_line):
        data = label_file_line.split(' ')
        data[1:] = [float(x) for x in data[1:]]

        self.type = data[0] # 'Car', 'Pedestrian', ...
        self.truncation = data[1] # truncated pixel ratio [0..1]
        self.occlusion = int(data[2]) # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[3] # object observation angle [-pi..pi]

        self.xmin = data[4] # left
        self.ymin = data[5] # top
        self.xmax = data[6] # right
        self.ymax = data[7] # bottom
        self.box2d = np.array([self.xmin,self.ymin,self.xmax,self.ymax])

        self.h = data[8] # box height
        self.w = data[9] # box width
        self.l = data[10] # box length (in meters)
        self.t = (data[11],data[12],data[13]) # location (x,y,z) in camera coord.
        self.ry = data[14] # yaw angle (around Y-axis in camera coordinates) [-pi..pi]

    def print_object(self):
        print('Type, truncation, occlusion, alpha: %s, %d, %d, %f' % \
            (self.type, self.truncation, self.occlusion, self.alpha))
        print('2d bbox (x0,y0,x1,y1): %f, %f, %f, %f' % \
            (self.xmin, self.ymin, self.xmax, self.ymax))
        print('3d bbox h,w,l: %f, %f, %f' % \
            (self.h, self.w, self.l))
        print('3d bbox location, ry: (%f, %f, %f), %f' % \
            (self.t[0],self.t[1],self.t[2],self.ry))

def inverse_rigid_trans(Tr):
    inv_Tr = np.zeros_like(Tr) # 3x4
    inv_Tr[0:3,0:3] = np.transpose(Tr[0:3,0:3])
    inv_Tr[0:3,3] = np.dot(-np.transpose(Tr[0:3,0:3]), Tr[0:3,3])
    return inv_Tr

class Calibration(object):
    def __init__(self, calib_filepath, from_video=False):
        if from_video:
            calibs = self.read_calib_from_video(calib_filepath)
        else:
            calibs = self.read_calib_file(calib_filepath)
        self.P = calibs['P2']
        self.P = np.reshape(self.P, [3,4])
        self.V2C = calibs['Tr_velo_to_cam']
        self.V2C = np.reshape(self.V2C, [3,4])
        # print(self.V2C)
        # print(np.transpose(self.V2C))
        self.C2V = inverse_rigid_trans(self.V2C)
        self.R0 = calibs['R0_rect']
        self.R0 = np.reshape(self.R0,[3,3])

        self.c_u = self.P[0,2]
        self.c_v = self.P[1,2]
        self.f_u = self.P[0,0]
        self.f_v = self.P[1,1]
        self.b_x = self.P[0,3]/(-self.f_u) # relative
        self.b_y = self.P[1,3]/(-self.f_v)

    def read_calib_file(self, filepath):
        data = {}
        with open(filepath, 'r') as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line)==0: continue
                key, value = line.split(':', 1)

                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass
        return data

    def read_calib_from_video(self, calib_root_dir):
        data = {}
        cam2cam = self.read_calib_file(os.path.join(calib_root_dir, 'calib_cam_to_cam.txt'))
        velo2cam = self.read_calib_file(os.path.join(calib_root_dir, 'calib_velo_to_cam.txt'))
        Tr_velo_to_cam = np.zeros((3,4))
        Tr_velo_to_cam[0:3,0:3] = np.reshape(velo2cam['R'], [3,3])
        Tr_velo_to_cam[:,3] = velo2cam['T']
        data['Tr_velo_to_cam'] = np.reshape(Tr_velo_to_cam, [12])
        data['R0_rect'] = cam2cam['R_rect_00']
        data['P2'] = cam2cam['P_rect_02']
        return data

    def cart2hom(self, pts_3d):
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n,1))))
        return pts_3d_hom

    # ===========================
    # ------- 3d to 3d ----------
    # ===========================
    def project_velo_to_ref(self, pts_3d_velo):
        pts_3d_velo = self.cart2hom(pts_3d_velo) # nx4
        return np.dot(pts_3d_velo, np.transpose(self.V2C))

    def project_ref_to_velo(self, pts_3d_ref):
        pts_3d_ref = self.cart2hom(pts_3d_ref) # nx4
        return np.dot(pts_3d_ref, np.transpose(self.C2V))

    def project_rect_to_ref(self, pts_3d_rect):
        return np.transpose(np.dot(np.linalg.inv(self.R0), np.transpose(pts_3d_rect)))

    def project_ref_to_rect(self, pts_3d_ref):
        return np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref)))

    def project_rect_to_velo(self, pts_3d_rect):
        pts_3d_ref = self.project_rect_to_ref(pts_3d_rect)
        return self.project_ref_to_velo(pts_3d_ref)

    def project_velo_to_rect(self, pts_3d_velo):
        pts_3d_ref = self.project_velo_to_ref(pts_3d_velo)
        return self.project_ref_to_rect(pts_3d_ref)

    # ===========================
    # ------- 3d to 2d ----------
    # ===========================
    def project_rect_to_image(self, pts_3d_rect):
        pts_3d_rect = self.cart2hom(pts_3d_rect)
        pts_2d = np.dot(pts_3d_rect, np.transpose(self.P)) # nx3
        pts_2d[:,0] /= pts_2d[:,2]
        pts_2d[:,1] /= pts_2d[:,2]
        return pts_2d[:,0:2]

    def project_velo_to_image(self, pts_3d_velo):
        pts_3d_rect = self.project_velo_to_rect(pts_3d_velo)
        return self.project_rect_to_image(pts_3d_rect)

    # ===========================
    # ------- 2d to 3d ----------
    # ===========================
    def project_image_to_rect(self, uv_depth):
        n = uv_depth.shape[0]
        x = ((uv_depth[:,0]-self.c_u)*uv_depth[:,2])/self.f_u + self.b_x
        y = ((uv_depth[:,1]-self.c_v)*uv_depth[:,2])/self.f_v + self.b_y
        pts_3d_rect = np.zeros((n,3))
        pts_3d_rect[:,0] = x
        pts_3d_rect[:,1] = y
        pts_3d_rect[:,2] = uv_depth[:,2]
        return pts_3d_rect

    def project_image_to_velo(self, uv_depth):
        pts_3d_rect = self.project_image_to_rect(uv_depth)
        return self.project_rect_to_velo(pts_3d_rect)

def load_velo_scan(velo_filename):
    scan = np.fromfile(velo_filename, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    return scan
def read_label(label_filename):
    lines = [line.rstrip() for line in open(label_filename)]
    objects = [Object3d(line) for line in lines]
    return objects

class kitti_object(object):
    def __init__(self, root_dir):
        self.root_dir = root_dir
        # self.split = split
        # self.split_dir = os.path.join(root_dir, split)
        #
        # self.lidar_dir = os.path.join('velodyne', self.split_dir)
        # self.label_dir = os.path.join('label', self.split_dir)
        # self.calib_dir = os.path.join('calib', self.split_dir)

        self.lidar_dir = os.path.join(root_dir, 'velodyne')
        self.label_dir = os.path.join(root_dir, 'label_2_extend')
        self.calib_dir = os.path.join(root_dir, 'calib')

    def get_lidar(self, idx):
        lidar_filename = os.path.join(self.lidar_dir, '%06d.bin'%(idx))
        return load_velo_scan(lidar_filename)

    def get_calibration(self, idx):
        calib_filename = os.path.join(self.calib_dir, '%06d.txt'%(idx))
        return Calibration(calib_filename)

    def get_label_objects(self, idx):
        label_filename = os.path.join(self.label_dir, '%06d.txt'%(idx))
        return read_label(label_filename)

def rotx(t):
    ''' x-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])
def roty(t):
    ''' y-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])
def rotz(t):
    ''' z-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])
def project_to_image(pts_3d, P):
    n = pts_3d.shape[0]
    pts_3d_extend = np.hstack((pts_3d, np.ones((n,1))))
    #print(('pts_3d_extend shape: ', pts_3d_extend.shape))
    pts_2d = np.dot(pts_3d_extend, np.transpose(P)) # nx3
    pts_2d[:,0] /= pts_2d[:,2]
    pts_2d[:,1] /= pts_2d[:,2]
    return pts_2d[:,0:2]

def compute_box_3d(obj, P):
    R = roty(obj.ry)

    l = obj.l
    w = obj.w
    h = obj.h

    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]

    corners_3d = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))

    corners_3d[0,:] = corners_3d[0,:] + obj.t[0]
    corners_3d[1,:] = corners_3d[1,:] + obj.t[1]
    corners_3d[2,:] = corners_3d[2,:] + obj.t[2]

    if np.any(corners_3d[2,:]<0.1):
        corners_2d = None
        return corners_2d, np.transpose(corners_3d)

    corners_2d = project_to_image(np.transpose(corners_3d), P)

    return corners_2d, np.transpose(corners_3d)

def compute_orientation_3d(obj, P):

    R = roty(obj.ry)

    orientation_3d = np.array([[0.0, obj.l],[0,0],[0,0]])

    orientation_3d = np.dot(R, orientation_3d)
    orientation_3d[0,:] = orientation_3d[0,:] + obj.t[0]
    orientation_3d[1,:] = orientation_3d[1,:] + obj.t[1]
    orientation_3d[2,:] = orientation_3d[2,:] + obj.t[2]

    if np.any(orientation_3d[2,:]<0.1):
        orientation_2d = None
        return orientation_2d, np.transpose(orientation_3d)

    orientation_2d = project_to_image(np.transpose(orientation_3d), P)
    return orientation_2d, np.transpose(orientation_3d)

def create_bbox_mesh(p3d, gt_boxes3d):
    b = gt_boxes3d
    for k in range(0,4):
        i,j=k,(k+1)%4
        p3d.add_line([b[i,0],b[i,1],b[i,2]], [b[j,0],b[j,1],b[j,2]])
        i,j=k+4,(k+1)%4 + 4
        p3d.add_line([b[i,0],b[i,1],b[i,2]], [b[j,0],b[j,1],b[j,2]])
        i,j=k,k+4
        p3d.add_line([b[i,0],b[i,1],b[i,2]], [b[j,0],b[j,1],b[j,2]])

class plot3d(object):
    def __init__(self):
        self.app = pg.mkQApp()
        self.view = gl.GLViewWidget()
        coord = gl.GLAxisItem()
        glLineWidth(3)
        coord.setSize(3,3,3)
        self.view.addItem(coord)
    def add_points(self, points, colors):
        points_item = gl.GLScatterPlotItem(pos=points, size=2, color=colors)
        self.view.addItem(points_item)
    def add_line(self, p1, p2):
        lines = np.array([[p1[0], p1[1], p1[2]],
                          [p2[0], p2[1], p2[2]]])
        lines_item = gl.GLLinePlotItem(pos=lines, mode='lines',
                                       color=(1,0,0,1), width=3, antialias=True)
        self.view.addItem(lines_item)
    def show(self):
        self.view.show()
        self.app.exec_()

def show_lidar_with_boxes(pc_velo, objects, calib):
    p3d = plot3d()
    points = pc_velo[:, 0:3]
    pc_inte = pc_velo[:, 3]
    pc_color = inte_to_rgb(pc_inte)
    p3d.add_points(points, pc_color)
    for obj in objects:
        if obj.type=='DontCare':continue
        # Draw 3d bounding box
        box3d_pts_2d, box3d_pts_3d = compute_box_3d(obj, calib.P)
        box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)
        create_bbox_mesh(p3d, box3d_pts_3d_velo)
        # Draw heading arrow
        ori3d_pts_2d, ori3d_pts_3d = compute_orientation_3d(obj, calib.P)
        ori3d_pts_3d_velo = calib.project_rect_to_velo(ori3d_pts_3d)
        x1,y1,z1 = ori3d_pts_3d_velo[0,:]
        x2,y2,z2 = ori3d_pts_3d_velo[1,:]
        p3d.add_line([x1,y1,z1], [x2,y2,z2])
    p3d.show()

def remove_label(pc_velo, objects, calib):
    points = pc_velo[:, 0:3]
    pc_inte = pc_velo[:, 3]
    pc_color = inte_to_rgb(pc_inte)

    points_in_box = []
    labels = []
    for obj in objects:

        box3d_pts_2d, box3d_pts_3d = compute_box_3d(obj, calib.P)
        box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)

        xmin = min(box3d_pts_3d_velo[0][0], box3d_pts_3d_velo[2][0])
        xmax = max(box3d_pts_3d_velo[0][0], box3d_pts_3d_velo[2][0])
        ymin = min(box3d_pts_3d_velo[0][1], box3d_pts_3d_velo[2][1])
        ymax = max(box3d_pts_3d_velo[0][1], box3d_pts_3d_velo[2][1])
        zmin = min(box3d_pts_3d_velo[0][2], box3d_pts_3d_velo[4][2])
        zmax = max(box3d_pts_3d_velo[0][2], box3d_pts_3d_velo[4][2])

        num_points = 0
        for p in points:

            if (xmin <= p[0] and p[0] <= xmax) \
                and (ymin <= p[1] and p[1] <= ymax) \
                and (zmin <= p[2] and p[2] <= zmax):

                points_in_box.append(p)
                num_points += 1

        # ポイント数がしきい値以上ならlabelファイルに追加する

        if obj.type == 'Car':
            # if num_points >= 20:
            if num_points >= 10:
                label = "{} -1 -1 {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}" \
                        .format(obj.type, obj.alpha, obj.xmin, obj.ymin, obj.xmax, obj.ymax, obj.h, obj.w, obj.l, obj.t[0], obj.t[1], obj.t[2], obj.ry)

                labels.append(label)

        else:
            # if num_points >= 10:
            if num_points >= 5:
                label = "{} -1 -1 {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}" \
                        .format(obj.type, obj.alpha, obj.xmin, obj.ymin, obj.xmax, obj.ymax, obj.h, obj.w, obj.l, obj.t[0], obj.t[1], obj.t[2], obj.ry)

                labels.append(label)


    return labels, points_in_box


def save_ground_truth(dir, id, labels):
    """Saves the ground truth data as a txt."""

    #####
    # labels, extends_labels = self.parse_ground_truth()


    txt_file =  os.path.join(dir, "label_2", "{:06d}.{}".format(id, "txt"))

    with open(txt_file, "w") as f:
        for label in labels:
            f.write("{}\n".format(label))





def inte_to_rgb(pc_inte):
    minimum, maximum = np.min(pc_inte), np.max(pc_inte)
    ratio = 2 * (pc_inte-minimum) / (maximum - minimum)
    b = (np.maximum((1 - ratio), 0))
    r = (np.maximum((ratio - 1), 0))
    g = 1 - b - r
    return np.stack([r, g, b, np.ones_like(r)]).transpose()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--dataset', type=str)
    args = parser.parse_args()

    dataset = kitti_object(args.dataset)

    data_list = glob.glob(os.path.join(args.dataset, "calib", "*.txt"))

    os.makedirs(os.path.join(args.dataset, "label_2"), exist_ok=True)


    for data_idx in range(len(data_list)):

        t0 = time.time()

        # data_idx = i

        # PC
        lidar_data = dataset.get_lidar(data_idx)
        # print(lidar_data.shape)

        # OBJECTS
        objects = dataset.get_label_objects(data_idx)
        # objects[0].print_object()

        # CALIB
        calib = dataset.get_calibration(data_idx)
        # print(calib.P)

        # Remove
        labels, obj_points = remove_label(lidar_data, objects, calib)

        # Save
        save_ground_truth(args.dataset, data_idx, labels)

        print("{} ({:.3f} s)".format(data_idx, time.time() - t0))

        # Show
        # show_lidar_with_boxes(lidar_data, objects, calib)
