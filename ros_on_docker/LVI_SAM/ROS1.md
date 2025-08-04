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
roslaunch lvi_sam run.launch
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
See launch details in `./src/LVI-SAM/launch/include/module_sam.launch`

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

## Parameters

```
# rosparam list
/PROJECT_NAME
/lvi_sam/Horizon_SCAN
/lvi_sam/N_SCAN
/lvi_sam/downsampleRate
/lvi_sam/edgeFeatureMinValidNum
/lvi_sam/edgeThreshold
/lvi_sam/extrinsicRPY
/lvi_sam/extrinsicRot
/lvi_sam/extrinsicTrans
/lvi_sam/globalMapVisualizationLeafSize
/lvi_sam/globalMapVisualizationPoseDensity
/lvi_sam/globalMapVisualizationSearchRadius
/lvi_sam/historyKeyframeFitnessScore
/lvi_sam/historyKeyframeSearchNum
/lvi_sam/historyKeyframeSearchRadius
/lvi_sam/historyKeyframeSearchTimeDiff
/lvi_sam/imuAccBiasN
/lvi_sam/imuAccNoise
/lvi_sam/imuGravity
/lvi_sam/imuGyrBiasN
/lvi_sam/imuGyrNoise
/lvi_sam/imuTopic
/lvi_sam/loopClosureEnableFlag
/lvi_sam/mappingCornerLeafSize
/lvi_sam/mappingProcessInterval
/lvi_sam/mappingSurfLeafSize
/lvi_sam/numberOfCores
/lvi_sam/odometrySurfLeafSize
/lvi_sam/pointCloudTopic
/lvi_sam/rotation_tollerance
/lvi_sam/savePCD
/lvi_sam/savePCDDirectory
/lvi_sam/surfFeatureMinValidNum
/lvi_sam/surfThreshold
/lvi_sam/surroundingKeyframeDensity
/lvi_sam/surroundingKeyframeSearchRadius
/lvi_sam/surroundingKeyframeSize
/lvi_sam/surroundingkeyframeAddingAngleThreshold
/lvi_sam/surroundingkeyframeAddingDistThreshold
/lvi_sam/timeField
/lvi_sam/useImuHeadingInitialization
/lvi_sam/z_tollerance
/lvi_sam_republish/compressed/mode
/robot_description
/rosdistro
/roslaunch/uris/host_przemek_ai__41459
/roslaunch/uris/host_przemek_ai__46253
/rosversion
/run_id
/vins_config_file  <-- "/catkin_ws/src/LVI-SAM/config/params_camera.yaml"
```

## Transforms

Original 'garden.bag' dataset uses the following transforms:
- in `params_lidar.yaml` (identical as for LIO-SAM, see: `../LIO_SAM/ROS1.md` for more details):
```
  extrinsicRot: [-1, 0, 0, 0, 1, 0, 0, 0, -1]  # Ry_180deg (pitch 180deg)
  extrinsicRPY: [0, 1, 0, -1, 0, 0, 0, 0, 1]   # Rz_270deg (yaw 270deg)
```
- in `params_camera.yaml`:
```
  # TODO TODO TODO <- revisit this transform
  data: [ 0, 0, -1,
         -1, 0, 0,
          0, 1, 0]  # Ry_270deg (pitch -90deg) @ Rz_270deg (yaw -90deg)
```

As for rotation from camera to IMU so far I tried:
- Rz(np.radians(-90)) @ Ry(np.radians(-90))
- Rz(np.radians(90)) @ Ry(np.radians(90)) (inverted order) - VINS does not report errors, but VIO and LIO are not in sync

*Original camera calibration*:
```
# camera model
model_type: MEI
camera_name: camera

# Mono camera config
image_width: 720
image_height: 540
mirror_parameters:
   xi: 1.9926618269451453
distortion_parameters:
   k1: -0.0399258932468764
   k2: 0.15160828121223818
   p1: 0.00017756967825777937
   p2: -0.0011531239076798612
projection_parameters:
   gamma1: 669.8940458885896
   gamma2: 669.1450614220616
   u0: 377.9459252967363
   v0: 279.63655686698144
# fisheye_mask: "/config/fisheye_mask_720x540.jpg"
```



# Experiments on "ConSLAM" dataset

Console 1:
```
cp /data/ConSLAM_data/LVI_SAM/ConSLAM_lidar_params_local.yaml /catkin_ws/src/LVI-SAM/config/params_lidar.yaml
roslaunch lvi_sam run.launch
```

Console 2 (with topic remap):
```
rosbag play /data/ConSLAM_data/sequence_02.bag --topics imu/data pp_points/synced2rgb pp_rgb/synced2points /pp_points/synced2rgb:=/points_raw /imu/data:=/imu_raw /pp_rgb/synced2points:=/camera/image_raw
```


Updates in `params_lidar.yaml` for ConSLAM dataset
```
  # Topics
  pointCloudTopic: "/points_raw"  # Point cloud data
  imuTopic: "/imu_raw"  # OR "/imu/data" if remapped
```

```
    extrinsicRot: [-1.0, 0.0, 0.0,
                      0.0, -1.0, 0.0,
                      0.0, 0.0, 1.0] == Rx_180deg @ Ry_180deg == Rz_180deg
    extrinsicRPY: [-1.0, 0.0, 0.0,
                   0.0, -1.0, 0.0,
                   0.0, 0.0, 1.0] == Rz_180deg
```

Updates in `params_camera.yaml`:
```
  imuTopic: "/imu_raw" # OR "/imu/data" if remapped
```

*Updated camera calibration* (using `/data/ConSLAM_data/data_calib/data_calib/calib_rgb.yaml`)

```
cp /data/LVI_SAM/fisheye_mask_2064x1544.jpg src/LVI-SAM/config/
```

```
# camera model
model_type: MEI
camera_name: camera

# Mono camera config
image_width: 2064
image_height: 1544
mirror_parameters:  # TODO: what is this? (check in VINS-Mono)
   xi: 1.9926618269451453
distortion_parameters:
   k1: -0.054366
   k2: 0.118389
   p1: 0.001781
   p2: -0.003908
projection_parameters:
   gamma1: 2351.85909
   gamma2: 2353.56734
   u0: 1017.24032
   v0: 788.70774
fisheye_mask: "/config/fisheye_mask_2064x1544.jpg"
```

*ConSLAM LIDAR-to-RGB transform*
```
0.05758374, -0.99833897, -0.00184651, -0.0081025,
-0.0017837,   0.0017467,  -0.99999688, -0.0631593,
0.99833909,  0.05758685, -0.00168015, -0.0215235,
0.0, 0.0, 0.0, 1.0
```
lidar-to-rgb (L2Cam)
[ 0, -1,  0,
  0,  0, -1,
  1,  0,  0]   # [NED] Ry_270deg (pitch -90deg) @ Rz_90deg (yaw 90deg)

*ConSLAM LIDAR-to-IMU rotation*
```
  -0.999512  -0.0311334 -0.00249871
  0.0311862     -0.9992  -0.0250271
-0.00171754  -0.0250928    0.999684
```
lidar-to-imu (L2Imu == L2Imu.T)
[-1,  0,  0,
  0, -1,  0,
  0,  0,  1]


Update for `params_camera.yaml`:
```
  # TODO TODO TODO <- revisit this transform
  data: [ 0, 0, -1,
         -1, 0, 0,
          0, 1, 0]  # [NED] Ry_90deg (pitch 90deg) @ Rz_90deg (yaw 90deg)
```

VINS autocalibration (estimate_extrinsic: 2) produces:
```
   0.197369   0.0115071   -0.980262
    0.97976   0.0317422    0.197641
  0.0333899    -0.99943 -0.00500922
```

Considering <- WORKS with `estimate_extrinsic: 1` and `estimate_td: 1`
L2Imu @ L2Cam.T =
[ 0,  0, -1,
  1,  0,  0,
  0, -1,  0]   # [NED] Ry_270deg @ Rz_90deg @ Rz_180deg


Discarted:
L2Cam @ L2Imu.T =
[ 0,  1,  0,
  0,  0, -1,
 -1,  0,  0]   # [NED] Ry_90deg @ Rz_90deg @ Rz_180deg


*Remaping of topics*:
```bash
rosbag play /data/ConSLAM_data/sequence_02.bag --topics imu/data pp_points/synced2rgb pp_rgb/synced2points /pp_points/synced2rgb:=/points_raw /imu/data:=/imu_raw /pp_rgb/synced2points:=/camera/image_raw
```

Header of /pp_rgb/synced2points:
```
$ rostopic echo /camera/image_raw --noarr
```
```
header: 
  seq: 3249
  stamp: 
    secs: 1650963684
    nsecs: 982393331
  frame_id: "pp_rgb"
height: 1544
width: 2064
encoding: "rgb8"
is_bigendian: 0
step: 6192
data: "<array type: uint8, length: 9560448>"
```

## Outcome of ConSLAM sequence 02:
VINS becomes lost in the very dark corrido (about 70% into the recording) and it breaks the entire solution.

NOTE: restart at 250/420 sec or earlier to allow for some map building before entering the dark corridor.

TODO: investigate if that cammera had rolling shutter
TODO: VINS calculates a large X offest between camera and IMU (between -1.4m and -2.3m) -> this is likely to be due to incorrect camera calibraiton.