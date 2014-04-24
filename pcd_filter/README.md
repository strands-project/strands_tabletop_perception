This package filters points from a point cloud and can be used for processing training data for the object classifier.
It saves the indices of the point cloud that are on the heighest table plane parallel to the largest plane (this will mostly be the floor) and within a distance *chop\_z\_max* from the camera. It saves these point indices as *.pcd file in the same folder as the input files adding a filename prefix *object\_indices\_*. This file is later used for the object classifier to train the objects.

To gather input data, put your training objects individually on an empty plane (e.g. floor) and grab the point clouds with a pcd grabber. Put them into a directory structure like:
```
_my_input_dir
|
+-- _apple
|   +-- 000.pcd
|   +-- 001.pcd
+-- _banana
|   +-- 000.pcd
|   +-- 001.pcd
|   +-- 002.pcd
+-- _mug
|   +-- 000.pcd
|   +-- 001.pcd
+-- _mouse
|   +-- 000.pcd
|   +-- 001.pcd
|   +-- 002.pcd
|   +-- 003.pcd
 ```

Finally, filter the pcd files in *my\_input\_dir* by running:
```
rosrun pcd_filter pcd_filter_node _input_dir:=~/my_input_dir  _input_dir:=/home/thomas/Projects/v4r/trunk/bin/records/ _chop_z_max:=1.1 _visualize:=true _force_refilter:=true
```
, where *_visualize* can be set to visualize the filtered point clouds and *_force_refilter* to do a new filtering process for already filtered .pcd files (i.e. pcd files where corresponding *object\_indices\_* already exist).


