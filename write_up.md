## Exercise 1: Pipeline for filtering and RANSAC plane fitting


```
a. Convert the ROS message into PCL point cloud format.

b. To reduce the amount of noise (unwanted features) from the image, a statistical
outlier filter is applied. It uses the mean distance of neighboring points to
determine whether a point is one from the wanted data (inlier), or if it is a random
noise point (outlier). I found ​ x = 0.3 ​ to be the best value that cleans away noise
while still preserving actual data.

c. A Voxel (portmanteau of volume and element) Downsampling filter was applied
to average the data in a voxel in order reduce the resolution of each voxel. This
significantly lowers the amount of memory and processing power needed to
process the image. The parameter controlling this is the LEAF_SIZE, and I found
a leaf size of 0.003 to be the most suitable value.

d. To further reduce noise, all the space irrelevant of the desired data was cropped
out using a Passthrough filter. It functions just like your everyday cropping tool,
but in all 3 dimensions. The region ​ 0.6 < z < 1.1 ​ removed the table, and ​ 0.4 < x
< 1.0 ​ removed the dropboxes on either side.

e. RANSAC plane segmentation was used to establish a boundary plane separating
the objects on the table from the table itself. The ideal threshold for determining
inliers was ​ 0.069 ​. I found that any higher than that and my larger objects (book
and biscuits box) would be excluded.
```
## Exercise 2: Pipeline including clustering for segmentation

```
a. Using the cloud of extracted objects, Euclidean clustering was applied to
differentiate which points belonged to which object. These parameters were tricky
to optimize, but after playing with the ​Visualizing DBSCAN Clustering​ tool and
messing with the parameters, I found my most optimal ones to be
ClusterTolerance(0.02), MinClusterSize(40) and MaxClusterSize(4000).

b. Each cluster is then given its own color and reconverted to a ROS message,
which RViz uses to generate the resulting point cloud.
```
## Exercise 3: Features extracted and SVM trained. Object recognition implemented.

```
a. The criteria used to identify the objects were color distribution and the pattern of
the object’s surface normals. These features were quantified by implementing the
skeleton histogram functions, provided in features.py. The parameters of the
histograms most optimal in real world conditions were bins=32 and HSV color
space, rather than RGB color space. The color characteristics of HSV were
significantly more defined, whereas colors in RGB data would become
indifferentiable under certain conditions, for example light intensity. The ranges of
the histogram were range=(0, 256) for color, and range=(-1, 1) for surface
normals.

b. Using these features, the capture_features.py script then iterated through 100
random orientations of each object and capturing its current color and normal
characteristics. Logically, the quality of the features become more accurate as
the number of orientations its exposed to increases. Since each orientation takes
about 4 seconds, and there being 8 objects to classify, it soon became apparent
that finding a ‘sweet spot’ of iterations was necessary. In my case, 100 was an
optimal number to generate a decent training set.

c. Using the generated training_set.sav the train_svm script was used to implement
the actual object recognition using the Support Vector Machine (SVM) machine
learning algorithm. It outputted the trained classification model (model.sav) used
to detect objects.
```

# Pick and Place Setup
```
For all three tabletop setups ( ​ test*.world ​ ), perform object recognition, then read
in respective pick list ( ​ pick_list_*.yaml ​ ). Next construct the messages that would
comprise a valid ​ PickPlace ​ request output them to ​ .yaml ​ format.

1. After object recognition on the objects was performed, the detected objects are
assigned their labels and published onto the ROS parameter server under
/detected_objects.

2. The function pr2_mover() takes the list of detected objects to use and identify the
tabletop objects in the Pick and Place world.

3. There are 3 worlds, with each having an increased number of objects to pick out.
The pick list is published to the parameter server under /object_list, along with
each object’s destined dropbox.

4. pr2_mover first reads in the parameters /object_list and /dropbox. From
/object_list, the objects to pick are assigned to object_list_param. From /dropbox,
the destination poses of the boxes are read, and assigned to green_position and
red_position.

5. object_list_param is then iterated. For each picked object, it’s name parameter is
extracted and assigned to object_name of type std_msgs/String.

6. The object’s group parameter is read into object_group is then used to assign the
picked object’s arm_name of type std_msgs/String and place_pose of type
geometry_msgs/Pose.

7. The detected_objects list is then iterated, comparing detected_object.label and
object_name.data. If matched, the detected object’s cloud data is used to
calculate the centroid as described in the lesson. The centroid is then assigned
to pick_pose of type geometry_msgs/Pose.

8. With all 5 ros_msgs found, the YAML file containing the output is generated
using the provided make_yaml_dict(test_scene_num, arm_name, object_name,
pick_pose, place_pose) function.
```

# Results
![world-1](https://github.com/rwbot/RoboND-Perception-Project/blob/master/world1.png?raw=true)
![world-2](https://github.com/rwbot/RoboND-Perception-Project/blob/master/world2.png?raw=true)
![world-3](https://github.com/rwbot/RoboND-Perception-Project/blob/master/world3.png?raw=true)

# Improvements
```
When playing around with all the parameters, leaf_size, maxclustersize, number of bins
etc, I found the process very tedious. Even though I used bash aliases to reduce the
typing effort of roslaunching to a few letters, it took a significant amount of time to
screenshot each output and log the current parameters of that iteration. And then I’d
have to manually open each screenshot in order to compare the difference in effects of
each parameter side by side. That was only for comparing a pair of values of just 1 filter
variable. Having several different filters, comparison becomes much more complex.
I thought of a wonderful solution after playing around with ​Visualizing DBSCAN Clustering
tool, which allows easy manipulation of parameters and seeing their effects in real time.
I imagined something like the live filter interface on our phone cameras.
```
![filters](https://www.askdavetaylor.com/wp-content/uploads/2014/03/iphone-camera-wrong-mode-3-620x348.png)
```
Exploring the interplay between filter parameters would be so much more intuitive if there was a
live feed of each filter. And also live feeds of different permutations of exaggerated parameter
combinations, which would give an intuitive understanding of the effect of each parameter.
It was a good idea, and I attempted it with such charisma, but I didn’t get very far before I was
humbled by the processing power of my Macbook Air. I had to close all apps before I loaded the
project simulator just for it to run, so there was no version of this running on my laptop. So
looking forward to Term 2 when I would have the nVidia Tegra processors, I’ll attempt it again.
```
