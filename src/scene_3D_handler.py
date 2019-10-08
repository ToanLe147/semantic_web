#!/usr/bin/env python

import pcl
import pcl.pcl_visualization
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class Segmentor:
    def __init__(self, sim=True):
        self.pc_input = rospy.Subscriber("/camera/depth/points", PointCloud2,
        self.callback)
        self.cloud_list = []
        self.sim = sim
        self.objects = []

    def callback(self, msg):
        cloud = pcl.PointCloud()
        if self.sim:
            field_names = ['x', 'y', 'z']
        else:
            field_names = [field.name for field in msg.fields]
        list_pc = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))
        self.cloud_list = list_pc
        cloud.from_list(list_pc)
        cloud_objects = self.Segmentation(self.Segmentation(cloud))
        self.objects = self.EuclideanCluster(cloud_objects)
        # self.Visualization(self.objects[2])

    def Segmentation(self, cloud):
        '''
        Remove plan such as table or floor
        '''
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        inliers, coefficients = seg.segment()
        # Extract outliers
        result_cloud = cloud.extract(inliers, negative=True)
        return result_cloud

    def EuclideanCluster(self, cloud):
        '''
        Extract each object clusters and store for later use
        '''
        objects = []

        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.01)
        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        for j, indices in enumerate(cluster_indices):
            cloud_cluster = pcl.PointCloud()
            cloud_cluster_list = []

            for i, indice in enumerate(indices):
                cloud_cluster_list.append([
                                        cloud[indice][0],
                                        cloud[indice][1],
                                        cloud[indice][2]
                                    ])

            cloud_cluster.from_list(cloud_cluster_list)
            objects.append(cloud_cluster)

            print('indices {} = '.format(j) + str(len(cloud_cluster_list)))
        print('=====')
        return objects

    def Visualization(self, cloud):
        self.viewer = pcl.pcl_visualization.CloudViewing()
        self.viewer.ShowMonochromeCloud(cloud, b'Sample Cloud')


def main():
    handler = Segmentor()
    rospy.init_node("point_cloud_handler", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
