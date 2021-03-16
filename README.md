# ComputerVision-Project
Force / Torque sensor fusion with vision using RGB camera for peg in hole task:

- Object detection 
- 3D  Pose Estimation 
- Pick and Place Task
- Force Torque sensor Reading and fusion with the Camera 

## Camera calibration
We have implemented codes for calibration of camera using Checkerboard picture, it could be used to get the camera interinsic and extrinsic parameters

## object detection codes:
this code is concerned about integrating the yolo with the robot by reading the XYZ coordinated from the camera sent from the txt file and do some transformations then publish them into pose stamped message type

- you will need to put it inside your workspace and catkin_make or build it 
- this is a ROS code you will need to chmod it first chmod +x baseobjectpose_v3.py inside the folder
- there are 2 files needed inside the code which are the gripperpos and the txt file for the yolo (bo7sen.txt)

```bash
$ rosrun my_iiwa_pkg baseobjectpose_v3.py
```



## Peg in hole 
This code was completly developed and done by Eng.Karam El maghout 

this code is concerned for the implementation on the real hardware and it contains 
```bash
$ rosrun my_iiwa_pkg peg_in_hole6
```


## Control system
this code may be used for controlling the sequence between the codes to run in the system
(it still needs some modifications)

```bash
$ python systemcontrol.py
```

