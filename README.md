# MasterUiA_HumanDetection
Human detection and tracking using RGB-D imaging with Yolo and ROS.

These packages has been developed as part of a Master Project at the University of Agder.
The task has been do detect and track people in in a 3D space.
This has been done with six RGB-D cameras, yolov3-tiny and ROS.

The darknet_ros_UiA package, based on "https://github.com/leggedrobotics/darknet_ros", does the detection.
The using_markers package makes cylinders around the humans to vizualize the detections
The person_pointcloud package aims at finding the points in the pointcloud belonging to the human.

Note: include-files not found during compilation can be found by copying from src to devel:
`cp ~/catkin_ws/src/wp3_humandetector/darknet_ros_UiA/darknet_ros/include/darknet_ros_msgs/* ~/catkin_ws/devel/include/darknet_ros_msgs/`
