# RAW PCL RECORDER

This package allows you to subscribe to and save any pointcloud topic. 

Example:
```
ros2 launch raw_pcl_recorder raw_pcl_recorder.launch.py data_path:=/tum/data topic_name:=/sensing/lidar/concatenated/pointcloud
```

After that you can follow this tutorial: https://carla.readthedocs.io/projects/ros-bridge/en/latest/pcl_recorder/

You might have too many files for pcl_concatenate_points_pcd 
If thats the cause you can use the tools/concatenater.sh script. Instead of concatenating all at once you concatenate smaller groups. After that concatenate these again to one file.

The recommended pcl density for autoware localization ist 0.2.