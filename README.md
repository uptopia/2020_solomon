# Solomon2020

# avoidance strategy
```
roslaunch realsense2_camera rs_camera.launch 
rosrun pcl_utils get_pointcloud_server
rviz
rosrun pcl_utils get_pointcloud_client
```


# pushpin reflector recognition
```
roslaunch realsense2_camera rs_rgbd_pushpin.launch
roslaunch yolo_detection yolo_get_pushpin.launch
```
# transparent object 
```
roslaunch realsense2_camera rs_rgbd_pushpin.launch
roslaunch yolo_detection yolo_get_transparent.launch
```
