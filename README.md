# STRANDS Tabletop Perception

Tabletop perception for STRANDS. The tabletop perception is realized as a ROS action server. Given the information about a table in the environment, i.e. its pose and its shape specified as a polygon, the robot plans multiple views to perceive the tabletop, segments the data from the depth camera at each of the planned views, and classifies the extracted segements for a given set of object classes. The individual components of the system are described below.

The information about the tables is stored in the ros datacentre (MongoDB). This information can either be added through the autonoumous table detection or a maunual process using a marker (cf. to the descriptions below). 

## Autonomous table detection (KTH, Nils)

## Manual table annotation (BHAM, Chris)

## View planning for tabletop perception (BHAM, Lars)

1. Make sure that you have the table information available in the ros datacentre

2. Make sure that you have a octomap server running with a local 3D map:
```
roslaunch perceive_tabletop_action octomao.launch
```
3.  
Launch the view planning components and action server:
```
roslaunch perceive_tabletop_action perceive_tabletop.launch
```

## 3D Object recognition (TUW, Aitor, Thomas, Michael) 

## Run the 'perceive tabletop' action

Run the tabletop action client with a table id (known by the ros datacentre):
```
rosrun perceive_tabletop_action PerceiveTabletopActionClient.py table_id:='table27'
```

## Appendix: table and object data management using MongoDB (BHAM, Chris) 
