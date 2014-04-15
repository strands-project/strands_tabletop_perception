It is based on the *object_classifier* ros package (read instructions there on how to train it).

You can launch the multi-view service, single-view service and segmentation module as follows:

roslaunch mv_object_classifier mv_classifier_from_files_demo.launch models_dir:=/media/DATA/Cat200_ModelDatabase__small/ training_dir:=/media/DATA/Cat200_ModelDatabase__small_trained/ chop_z:=1 mv_visualize_output:=true

and then:

rosrun mv_object_classifier mv_object_classifier_demo_from_file

which will use the PCD files stored in data.
