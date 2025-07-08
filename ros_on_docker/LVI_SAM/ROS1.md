# Reference implementation

https://github.com/TixiaoShan/LVI-SAM

*Datasets*
https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV

# Build and run docker image:

```
$ ros_on_docker/ros_on_docker_nvidia/run_ros1_docker_gpu_lvi_sam.sh
```

# Experiments on "garden.bag" dataset

Console 1:
```
rosrun lvi_sam run.launch
```

Console 2:
```
$ rosbag run -r /data/LVI-SAM/garden.bag
```

# Nodes:
```
$ rosnode list
/lvi_sam_featureExtraction
/lvi_sam_imageProjection
/lvi_sam_imuPreintegration
/lvi_sam_mapOptmization
/lvi_sam_republish
/lvi_sam_robot_state_publisher
/lvi_sam_rviz
/lvi_sam_visual_feature
/lvi_sam_visual_loop
/lvi_sam_visual_odometry
/play_1751947983836050462
```

# Topics:
```
$ rostopic list
/camera/image_raw
/camera/image_raw/compressed
/clock
/imu_raw
/joint_states
/lvi_sam/lidar/deskew/cloud_deskewed
/lvi_sam/lidar/deskew/cloud_info
/lvi_sam/lidar/feature/cloud_corner
/lvi_sam/lidar/feature/cloud_info
/lvi_sam/lidar/feature/cloud_surface
/lvi_sam/lidar/imu/path
/lvi_sam/lidar/mapping/cloud_registered
/lvi_sam/lidar/mapping/cloud_registered_raw
/lvi_sam/lidar/mapping/loop_closure_constraints
/lvi_sam/lidar/mapping/loop_closure_corrected_cloud
/lvi_sam/lidar/mapping/loop_closure_history_cloud
/lvi_sam/lidar/mapping/map_global
/lvi_sam/lidar/mapping/map_local
/lvi_sam/lidar/mapping/odometry
/lvi_sam/lidar/mapping/path
/lvi_sam/lidar/mapping/trajectory
/lvi_sam/vins/depth/depth_cloud
/lvi_sam/vins/depth/depth_feature
/lvi_sam/vins/depth/depth_image
/lvi_sam/vins/depth/depth_image/mouse_click
/lvi_sam/vins/feature/feature
/lvi_sam/vins/feature/feature_img
/lvi_sam/vins/feature/feature_img/mouse_click
/lvi_sam/vins/feature/restart
/lvi_sam/vins/loop/keyframe_pose
/lvi_sam/vins/loop/match_frame
/lvi_sam/vins/loop/match_image
/lvi_sam/vins/loop/match_image/mouse_click
/lvi_sam/vins/odometry/camera_pose
/lvi_sam/vins/odometry/camera_pose_visual
/lvi_sam/vins/odometry/extrinsic
/lvi_sam/vins/odometry/history_cloud
/lvi_sam/vins/odometry/imu_propagate
/lvi_sam/vins/odometry/imu_propagate_ros
/lvi_sam/vins/odometry/key_poses
/lvi_sam/vins/odometry/keyframe_point
/lvi_sam/vins/odometry/keyframe_pose
/lvi_sam/vins/odometry/odometry
/lvi_sam/vins/odometry/path
/lvi_sam/vins/odometry/point_cloud
/lvi_sam_republish/compressed/parameter_descriptions
/lvi_sam_republish/compressed/parameter_updates
/odometry/gps
/odometry/imu
/points_raw
/rosout
/rosout_agg
/tf
/tf_static
```

*Odometry topics*
```
$ rostopic list | grep -i odometry
/lvi_sam/lidar/mapping/odometry
/lvi_sam/vins/odometry/camera_pose
/lvi_sam/vins/odometry/camera_pose_visual
/lvi_sam/vins/odometry/extrinsic
/lvi_sam/vins/odometry/history_cloud
/lvi_sam/vins/odometry/imu_propagate
/lvi_sam/vins/odometry/imu_propagate_ros
/lvi_sam/vins/odometry/key_poses
/lvi_sam/vins/odometry/keyframe_point
/lvi_sam/vins/odometry/keyframe_pose
/lvi_sam/vins/odometry/odometry
/lvi_sam/vins/odometry/path
/lvi_sam/vins/odometry/point_cloud
/odometry/gps
/odometry/imu
```

*Cloud topics*
```
$ rostopic list | grep -i cloud
/lvi_sam/lidar/deskew/cloud_deskewed
/lvi_sam/lidar/deskew/cloud_info
/lvi_sam/lidar/feature/cloud_corner
/lvi_sam/lidar/feature/cloud_info
/lvi_sam/lidar/feature/cloud_surface
/lvi_sam/lidar/mapping/cloud_registered
/lvi_sam/lidar/mapping/cloud_registered_raw
/lvi_sam/lidar/mapping/loop_closure_corrected_cloud
/lvi_sam/lidar/mapping/loop_closure_history_cloud
/lvi_sam/vins/depth/depth_cloud
/lvi_sam/vins/odometry/history_cloud
/lvi_sam/vins/odometry/point_cloud
```


# Bag info:
```
$ rosbag info /data/LVI-SAM/garden.bag
path:        /data/LVI-SAM/garden.bag
version:     2.0
duration:    30:43s (1843s)
start:       Jun 17 2020 19:44:01.38 (1592423041.38)
end:         Jun 17 2020 20:14:45.19 (1592424885.19)
size:        12.2 GB
messages:    995479
compression: none [13707/13707 chunks]
types:       sensor_msgs/CompressedImage [8f7a12909da2c9d3332d540a0977563f]
             sensor_msgs/Imu             [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/PointCloud2     [1158d486dd51d683ce2f1be655c3c181]
topics:      /camera/image_raw/compressed    55309 msgs    : sensor_msgs/CompressedImage
             /imu_raw                       921889 msgs    : sensor_msgs/Imu
             /points_raw                     18281 msgs    : sensor_msgs/PointCloud2
```


# Odometry topics

```
root@przemek-ai:/catkin_ws# rostopic info /odometry/imu
Type: nav_msgs/Odometry

Publishers:
 * /lvi_sam_imuPreintegration

Subscribers:
 * /lvi_sam_visual_odometry


root@przemek-ai:/catkin_ws# rostopic info /odometry/gps
Type: nav_msgs/Odometry

Publishers: None

Subscribers:
 * /lvi_sam_mapOptmization


root@przemek-ai:/catkin_ws# rostopic info /lvi_sam/lidar/mapping/odometry
Type: nav_msgs/Odometry

Publishers:
 * /lvi_sam_mapOptmization

Subscribers:
 * /lvi_sam_imuPreintegration


root@przemek-ai:/catkin_ws# rostopic info /lvi_sam/vins/odometry/odometry
Type: nav_msgs/Odometry

Publishers:
 * /lvi_sam_visual_odometry

Subscribers: None
```
