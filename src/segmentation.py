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
    cloud.from_list(list_pc)

    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # Segmentation Object of of Floor
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    inliers, coefficients = seg.segment()
    # Extract inliers
    # cloud_table = cloud.extract(inliers, negative=False)
    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Segmentation Object out of Table
    seg2 = cloud_objects.make_segmenter()
    seg2.set_model_type(pcl.SACMODEL_PLANE)
    seg2.set_method_type(pcl.SAC_RANSAC)
    seg2.set_distance_threshold(0.01)
    inliers, coefficients = seg2.segment()
    cloud_objects = cloud_objects.extract(inliers, negative=True)

    # tree = cloud_filtered.make_kdtree()
    # ec = cloud_filtered.make_EuclideanClusterExtraction()
    # ec.set_ClusterTolerance(0.1)
    # ec.set_MinClusterSize(100)
    # ec.set_MaxClusterSize(25000)
    # ec.set_SearchMethod(tree)
    # cluster_indices = ec.Extract()
    #
    # print('cluster_indices : ' + str(cluster_indices.count) + " count.")
    #
    # cloud_cluster_list = []
    #
    # for j, indices in enumerate(cluster_indices):
    #     print('indices = ' + str(len(indices)))
    #
    #     for i, indice in enumerate(indices):
    #         cloud_cluster_list.append([
    #                                 cloud_filtered[indice][0],
    #                                 cloud_filtered[indice][1],
    #                                 cloud_filtered[indice][2]
    #                             ])
    #
    # cloud_cluster = pcl.PointCloud()
    # cloud_cluster.from_list(cloud_cluster_list)

    # Visualization
    viewer.ShowMonochromeCloud(cloud_objects, b'sample cloud')


sub = rospy.Subscriber("/camera/depth/points", PointCloud2, callback)

rospy.spin()
