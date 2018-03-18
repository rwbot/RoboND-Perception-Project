#!/usr/bin/env python
# rwbot
# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

############################## Exercise-2 TODOs: ##############################

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(8)
    # Set threshold scale factor
    x = 0.3
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    #print("Outlier Done")

    # TODO: Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.003
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    #print("Vox Done")

    # TODO: PassThrough Filter
    # PassThrough filter 0.6 < z < 1.1
    pass_z = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    #passthrough.set_filter_field_name(filter_axis)
    pass_z.set_filter_field_name('z')
    #passthrough.set_filter_limits(axis_min, axis_max)
    pass_z.set_filter_limits(0.6, 1.1)
    # Obtain the resultant point cloud.
    cloud_filtered = pass_z.filter()
    #print("Pass Z Done")

    # PassThrough filter 0.4 < x < 1
    pass_x = cloud_filtered.make_passthrough_filter()
    pass_x.set_filter_field_name('x')
    pass_x.set_filter_limits(0.4, 1)
    cloud_filtered = pass_x.filter()
    #print("Pass X Done")

    # # PassThrough filter -0.5 < y < 0.5
    # pass_y = cloud_filtered.make_passthrough_filter()
    # pass_y.set_filter_field_name('y')
    # pass_y.set_filter_limits(-0.5, 0.5)
    # cloud_filtered = pass_y.filter()


    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    # 0.01 sometimes has trouble with the width of the book ¿ ?
    max_distance = 0.069
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    #print("RANSAC Done")

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
#---------------------------------------------
    #Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: 0.001, 10, 250 respectively are terrible choices
    # Experiment - 0.02, 40, 4000 works most of the time
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(40)
    ec.set_MaxClusterSize(4000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            in_0 = white_cloud[indice][0]
            in_1 = white_cloud[indice][1]
            in_2 = white_cloud[indice][2]
            in_c = rgb_to_float(cluster_color[j])
            color_cluster_point_list.append([in_0, in_1, in_2, c])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    #print("Clustering Done")

############################### Exercise-3 TODOs: #####################################
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        #print("Predicting..........")
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)

        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        #Default: chists = compute_color_histograms(sample_cloud, using_hsv=False)
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass




#++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):

    #rospy.loginfo("I will publish to the topic %s", topic)
    rospy.loginfo('Detected {} objects'.format(len(detected_objects)))

    print("Start pr2_mover")
    # TODO: Initialize variables
    output_dict = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox = rospy.get_param('/dropbox')
    test_scene_num = Int32()
    test_scene_num.data = 3

    # TODO: Parse parameters into individual variables
    #object_name = [element['name'] for element in object_list_param]
    red_position = dropbox[0]['position']
    green_position = dropbox[1]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for pick_obj in object_list_param:

        object_name = String()
        object_name.data = pick_obj['name']
        print("Looping through pick list  ", object_name.data)

        centroid = [] # to be list of tuples (x, y, z)
        pick_pose = Pose()
        pick_pose.position.x = 0
        pick_pose.position.y = 0
        pick_pose.position.z = 0

        arm_name = String()
        object_group = String()
        object_group.data = pick_obj['group']

        if object_group.data == 'green':
            # TODO: Assign the arm to be used for pick_place
            arm_name.data = 'right'
            drop_position = green_position
        else:
            arm_name.data = 'left'
            drop_position = red_position
        print("Arm    ",arm_name.data)

        # TODO: Create 'place_pose' for the object
        place_pose = Pose()
        place_pose.position.x = drop_position[0]
        place_pose.position.y = drop_position[1]
        place_pose.position.z = drop_position[2]


        # TODO: Loop through the detected_object list
        for detected_object in detected_objects:
            print("Looping through detected objects for    ",detected_object.label)
            # Find the matching detected object
            if detected_object.label == object_name.data:
                print("Matching detected object")
                # TODO: Get the PointCloud for a given object and obtain it's centroid
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                #scalar_centroid = [np.asscalar(element) for element in centroid]

                # TODO: Create 'pick_pose' for the object
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])

                print(pick_pose.position)




        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        output_dict.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))
        print("Writing Dictionary")

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        #print("Wait for 'pick_place_routine' service to come up")

        #try:
            #pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            # TODO: Insert your message variables to be sent as a service request
            #response = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
            #print ("Response: ",response.success)
        #except rospy.ServiceException, e:
            #print("Service call failed: {}".format(e))

    print("OUTPUTTING YAML")
    # TODO: Output your request parameters into output yaml file
    yaml_filename = "output_" + str(test_scene_num.data) + ".yaml"
    send_to_yaml(yaml_filename, output_dict)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)
    print("rwbot node initialized")
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)
    print("Created Suscribers")

    # TODO: Create Publishers
    #pcl_test_pub = rospy.Publisher("/test_cloud", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
    print("Created Publishers")

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    print("Loaded Model")

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
