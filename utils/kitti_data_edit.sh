# 最初にvelodyne_pcdとvelodyne，label_2をそれぞれvelodyne_pcd_master、velodyne_master、label_2_extendにしておくこと
# left_right_to_top leftとrightのデータをtopと同じ座標に変換する (in: velodyne_pcd_master, out: velodyne_pcd_v2v velodyne_v2v)
python3 left_right_to_top.py -d kitti/testing

# left_right_top_concat 変換されたleftとrightをtopと結合する (in: velodyne_pcd_v2v, out: velodyne_pcd velodyne)
python3 join_pcd.py -d kitti/testing

# removal_bb 点群数が少なかったらBB削除  (in: label_2_extend, out: label_2)
python3 removal_bb.py -d kitti/testing

# pcd_to_bin
