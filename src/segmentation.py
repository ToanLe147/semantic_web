#!/usr/bin/env python

import numpy as np
import pcl
import pcl.pcl_visualization
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# // initialize PointClouds
cloud = pcl.PointCloud()
viewer = pcl.pcl_visualization.CloudViewing()

rospy.init_node("test")
list_pc = []
sim = True


def callback(msg):
    global viewer, sim
    if sim:
        field_names = ['x', 'y', 'z']
    else:
        field_names = [field.name for field in msg.fields]

    list_pc = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))
    # print(list_pc)
    cloud.from_list(list_pc)
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.1)
    seg.set_optimize_coefficients(True)
    indices, model = seg.segment()
    # print(indices)
    new_list_pc = [list_pc[i] for i in indices]
    cloud.from_list(new_list_pc)
    viewer.ShowMonochromeCloud(cloud, b'sample cloud')


sub = rospy.Subscriber("/camera/depth/points", PointCloud2, callback)

rospy.spin()
