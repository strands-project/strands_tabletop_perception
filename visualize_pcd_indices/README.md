This package visualizes all points of the point clouds in directory *my\_input\_dir* that have a corresponding file with saved point indices (file prefix: *object\_indices\_*).

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

For instance:

To show all point clouds of all classes, run:
```
rosrun visualize_pcd_indices visualize_pcd_indices_node _dir:=my_input_dir
```

to show all point clouds of a particular class (e.g. banana), run:

```
rosrun visualize_pcd_indices visualize_pcd_indices_node _dir:=my_input_dir/banana
```

One possibility to compute the indices of the point cloud above a table plane is to use the catkin package *pcd\_filter* (https://github.com/strands-project/strands_tabletop_perception/tree/hydro-devel/pcd_filter).
