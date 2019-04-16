## Overview

This package subscribes to all the point clouds from all nodes, together with coordinates of person, and the output is the pointcloud of only the person.

The "crop box" crops the point cloud to a 1,2mx1,2mx2.0m box around the person.

The "segmentation"-scripts uses the cropped pointcloud and segments it.
The parameter minPoints set the threshold for how many points there must be in a cluster to be kept.
