## Overview

This package is based on "YOLO ROS: Real-Time Object Detection for ROS" (https://github.com/leggedrobotics/darknet_ros).
The changes that are done from the original package is to get the yolo node to subscribe to a depth image as well as the original rgb-image. Based on the depth measurements and camera intrinsic parameters, the 3D-coordinates of the detected object can be calculated.
The code is written specificially for the kinect v2. 

## Outputs

X,Y,G is the 3D-coordinates of the object in the cameras local coordinate system.
Xg,Yg,Zg is the 3D-coordinates of the object in the global coordinate-system.


## Parameters

topic_in_depth: depth image to be subscribed
topic_in_rgb: color image to be subscribed
thresHold: threshold for object detection
lengthAdded: enlonge the vector(mm) from camera to object, set as 0 if not needed
Cx,Cy,fx,fy: camera intrinsic parameters
xr,yr,zr: angle(rad) of rotation between global and local coordinate-system
xt,yt,zt: translation(meters) between global and local coordinate-system

## Other

The "sensor_fusion" scripts uses information from all the nodes to find a mean value for the coordinates of a person. This doesnt work for more than one person.
