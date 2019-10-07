#!/usr/bin/env python

# Import modules
from pcl_helper import *
import pcl

# TODO: Define functions as required
def voxel_grid_downsampling(cloud_pcd, leaf_size = 0.01):
    #create a voxel grid filter object for input cloud
    vox = cloud_pcd.make_voxel_grid_filter()

    # set the voxel leaf size for the filter
    vox.set_leaf_size(leaf_size, leaf_size,leaf_size)

    # call the filter function to obtain the resultant downsample point cloud separately
    cloud_filter = vox.filter()

    return cloud_filter

def pass_through_filter(cloud_pcd, axis_min = 0.77, axis_max = 1.1, filter_axis = 'z'):
    #create a pass thought filter 
    pass_through_filter = cloud_pcd.make_passthrough_filter()

    #assign the axis range to the passthought filter
    pass_through_filter.set_filter_field_name(filter_axis)
    pass_through_filter.set_filter_limits(axis_min, axis_max)

    cloud_filter = pass_through_filter.filter()

    return cloud_filter

def segmentaion_object(cloud_pcd, max_distance = 0.005, model_type = pcl.SACMODEL_PLANE, method_type = pcl.SAC_RANSAC):
    seg = cloud_pcd.make_segmenter()

    #set the model that you wish to fit
    seg.set_model_type(model_type)
    seg.set_method_type(method_type)

    # max distance for a point to be considered fitting the model
    seg.set_distance_threshold(max_distance)

    # call the segment function to obtain set of inlier indices and model coefficents
    inliers_bound, coefficents = seg.segment()

    outliers = cloud_pcd.extract(inliers_bound, negative = False)
    inliers = cloud_pcd.extract(inliers_bound, negative = True)

    return outliers, inliers

def eulidean_cluster_extraction(choud_pcd, cluster_tolerance = 0.04, min_cluster_size = 150, max_cluster_size = 2500):
    white_cloud = XYZRGB_to_XYZ(choud_pcd)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    ec.set_SearchMethod(tree)

    return white_cloud, ec.Extract()

def assign_cluster_colors(extracted_clusters, origin_cloud):
    cluster_colors = get_color_list(len(extracted_clusters))
    color_cluster_point_list = []



    for j, indices in enumerate(extracted_clusters):
        for i,indice in enumerate(indices):
            color_cluster_point_list.append([origin_cloud[indice][0], 
                                            origin_cloud[indice][1],
                                            origin_cloud[indice][2],
                                            rgb_to_float(cluster_colors[j])])

    return color_cluster_point_list

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    vox = voxel_grid_downsampling(cloud)
    # TODO: PassThrough Filter
    pass_through = pass_through_filter(vox)

    # TODO: RANSAC Plane Segmentation
    cloud_table, cloud_objects = segmentaion_object(pass_through)

    # TODO: Euclidean Clustering
    white_cloud, white_ec = eulidean_cluster_extraction(cloud_objects)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    color_list = assign_cluster_colors(white_ec,white_cloud)

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster = pcl_to_ros(cluster_cloud)


    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_objects_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cluster)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('cluster', anonymous = True)
    # TODO: Create SubscribersTT
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size = 1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects",PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size = 1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()