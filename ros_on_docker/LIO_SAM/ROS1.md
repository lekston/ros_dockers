# Getting started

This section describes how to run LIO-SAM on ROS1 docker on ubuntu 20.04.
The X11 Display server must be running for the RVIZ to work.
Additionally, `xhost +local:root` must be run (from the console used to run docker containers) to allow the container to connect to the X server.

## Download LIO-SAM `walking_dataset.bag`
Download LIO-SAM `walking_dataset.bag` to: `/opt/mnt/data/10_slam/LIO_SAM/`.
If another path is used the update the volume mapping in the `run_ros1_docker_lio_sam.sh` script.

## Build and run the docker image
```bash
cd ./ros_on_docker/ros_on_docker/
./run_ros1_docker_lio_sam.sh
```

## Run the LIO-SAM and record the outputs
Then in the container start tmux and in 4 separate consoles run:
* Console 1: roscore
```bash
source /opt/ros/noetic/setup.bash
roscore
```

* Console 2: prepare to record the outputs of LIO-SAM:
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM
rosbag record -o /data/LIO_SAM/walking_dataset_cloud_registered.bag /lio_sam/mapping/cloud_registered /lio_sam/mapping/cloud_registered /odometry/imu /lio_sam/imu/path
```

* Console 3: launch the LIO-SAM:
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM
roslaunch lio_sam run.launch
```

* Console 4: replay the LIO-SAM bag:
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM
rosbag play /data/LIO_SAM/walking_dataset.bag
```
If remapping of topics is needed use:
```bash
rosbag play /data/LIO_SAM/walking_dataset.bag /old_topic_name:=/new_topic_name
```

## Convert LIO-SAM bag to laz
Please update the `<date-of-recording>` in the following command with the actual date of the bag file creation.

```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM

# need to provide both the bag file and the output directory
rosrun cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_<date-of-recording>.bag /data/LIO_SAM/session_HDmapping
```

# Docker details

## Docker build

*Build LIO-SAM image on ROS1*
```bash
sudo docker build -t ros1-dev-liosam -f Dockerfile_ros1_lio_sam .
```

## Docker run command

NOTES:
- assuming the following directory structure:
```bash
/opt/mnt/data/10_slam/
├── LIO_SAM
│   └── walking_dataset.bag
```

*Run LIO-SAM on ROS1*
```bash
sudo docker run -it --rm     --gpus all     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --volume="`pwd`:/root/src_wip"     --volume="/opt/mnt/data/10_slam/:/data/"     --network=host     --name ros1-dev-liosam     ros1-dev-liosam
```
# Data

```
rosbag info /data/LIO_SAM/walking_dataset.bag
```

# Package overview

```
rosls lio_sam
```

## LIO-SAM Nodes

```
rosnode list
/ekf_gps
/lio_sam_featureExtraction    # 1b
/lio_sam_imageProjection      # 0
/lio_sam_imuPreintegration    # 1a
/lio_sam_mapOptmization       # 2
/navsat
/robot_state_publisher
```

## Parameters

Source:
LIO-SAM/include/lio_sam/utility.hpp

### Parameters controlling topics
```
declare_parameter("pointCloudTopic", "points");
get_parameter("pointCloudTopic", pointCloudTopic);
declare_parameter("imuTopic", "imu/data");
get_parameter("imuTopic", imuTopic);
declare_parameter("odomTopic", "lio_sam/odometry/imu");
get_parameter("odomTopic", odomTopic);
declare_parameter("gpsTopic", "lio_sam/odometry/gps");
get_parameter("gpsTopic", gpsTopic);
```

### Parameters controlling frames
```
declare_parameter("lidarFrame", "laser_data_frame");
get_parameter("lidarFrame", lidarFrame);
declare_parameter("baselinkFrame", "base_link");
get_parameter("baselinkFrame", baselinkFrame);
declare_parameter("odometryFrame", "odom");
get_parameter("odometryFrame", odometryFrame);
declare_parameter("mapFrame", "map");
get_parameter("mapFrame", mapFrame);
```

## LIO-SAM Topics

### Topics from LIO-SAM

```
$ rostopic list
/clock
/diagnostics
/joint_states
/lio_loop/loop_closure_detection
/lio_sam/deskew/cloud_deskewed
/lio_sam/deskew/cloud_info
/lio_sam/feature/cloud_corner
/lio_sam/feature/cloud_info
/lio_sam/feature/cloud_surface
/lio_sam/imu/path
/lio_sam/mapping/cloud_registered
/lio_sam/mapping/cloud_registered_raw
/lio_sam/mapping/icp_loop_closure_corrected_cloud
/lio_sam/mapping/icp_loop_closure_history_cloud
/lio_sam/mapping/loop_closure_constraints
/lio_sam/mapping/map_global
/lio_sam/mapping/map_local
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/mapping/path
/lio_sam/mapping/slam_info
/lio_sam/mapping/trajectory
/odometry/gps
/odometry/gpsz
/odometry/imu
/odometry/imu_incremental
/odometry/navsat
/set_pose
/tf
/tf_static
```

### Topics from rosbag (walking_dataset.bag)

```
topics:      /diagnostics                                                1299 msgs    : diagnostic_msgs/DiagnosticArray
             /gx5/gps/fix                                                2623 msgs    : sensor_msgs/NavSatFix
             /gx5/nav/odom                                               6557 msgs    : nav_msgs/Odometry
             /gx5/nav/status                                             6557 msgs    : std_msgs/Int16MultiArray
             /imu_correct                                              327859 msgs    : sensor_msgs/Imu
             /imu_raw                                                  327870 msgs    : sensor_msgs/Imu
             /points_raw                                                 6502 msgs    : sensor_msgs/PointCloud2
             /rosout                                                      355 msgs    : rosgraph_msgs/Log                     (6 connections)
             /rosout_agg                                                  338 msgs    : rosgraph_msgs/Log
             /velodyne_nodelet_manager/bond                              2624 msgs    : bond/Status                           (3 connections)
             /velodyne_nodelet_manager_cloud/parameter_descriptions         1 msg     : dynamic_reconfigure/ConfigDescription
             /velodyne_nodelet_manager_cloud/parameter_updates              1 msg     : dynamic_reconfigure/Config
             /velodyne_nodelet_manager_driver/parameter_descriptions        1 msg     : dynamic_reconfigure/ConfigDescription
             /velodyne_nodelet_manager_driver/parameter_updates             1 msg     : dynamic_reconfigure/Config
             /velodyne_packets                                           6502 msgs    : velodyne_msgs/VelodyneScan
```

## LIO-SAM CloudInfo Message

```
rosmsg info lio_sam/cloud_info
```

## LIO-SAM ImageProjection Node

Input topics to first node

```
    subImu = create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, qos_imu,
        std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
        imuOpt);
    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        odomTopic + "_incremental", qos_imu,
        std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
        odomOpt);
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointCloudTopic, qos_lidar,
        std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
        lidarOpt);

    pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/deskew/cloud_deskewed", 1);
    pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
        "lio_sam/deskew/cloud_info", qos);
```

## LIO-SAM FeatureExtraction Node

```
    subLaserCloudInfo = create_subscription<lio_sam::msg::CloudInfo>(
        "lio_sam/deskew/cloud_info", qos,
        std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

    pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
        "lio_sam/feature/cloud_info", qos);
    pubCornerPoints = create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/feature/cloud_corner", 1);
    pubSurfacePoints = create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio_sam/feature/cloud_surface", 1);
```

## LIO-SAM MapOptimization Node

```
    pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
    pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
    pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos);
    pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
        "lio_sam/mapping/odometry_incremental", qos);
    pubPath = create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
    br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    subCloud = create_subscription<lio_sam::msg::CloudInfo>(
        "lio_sam/feature/cloud_info", qos,
        std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
    subGPS = create_subscription<nav_msgs::msg::Odometry>(
        gpsTopic, 200,
        std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
    subLoop = create_subscription<std_msgs::msg::Float64MultiArray>(
        "lio_loop/loop_closure_detection", qos,
        std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

    srvSaveMap = create_service<lio_sam::srv::SaveMap>("lio_sam/save_map", saveMapService);
    pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

    pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_local", 1);
    pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
    pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

```

## LIO-SAM IMU Preintegration Node

```
    subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
        "lio_sam/mapping/odometry", qos,
        std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
        laserOdomOpt);
    subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(
        odomTopic+"_incremental", qos_imu,
        std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
        imuOdomOpt);

    pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
    pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);
```

# Experiments

## Recording 1 (was incomplete due to missing imu topic)

```bash
rosbag record -o /data/LIO_SAM/walking_dataset_cloud_registered.bag /lio_sam/mapping/cloud_registered /lio_sam/mapping/cloud_registered /odometry/imu /lio_sam/imu/path
```

*Output*
```bash
$ rosrun cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag /data/LIO_SAM/session_HDmapping_walking_dataset /cloud_registered:=/lio_sam/mapping/cloud_registered
...
[INFO] [1746165112.065840987]: Received message on topic: /lio_sam/mapping/cloud_registered
[INFO] [1746165112.066110984]: Processing 4359 points
[INFO] [1746165112.067624115]: Processed 8718 points!
start loading pc
loading pc finished
adding chunk [1]
adding chunk [2]
adding chunk [3]
adding chunk [4]
adding chunk [5]
adding chunk [6]
adding chunk [7]
reamaining points: 135555
cleaning points
points cleaned
start indexing chunks_trajectory
number of trajectory elements: 0
number of trajectory elements: 0
number of trajectory elements: 0
number of trajectory elements: 0
number of trajectory elements: 0
number of trajectory elements: 0
number of trajectory elements: 0
start transforming chunks_pc to local coordinate system
computing [1] of: 7
computing [2] of: 7
computing [3] of: 7
computing [4] of: 7
computing [5] of: 7
computing [6] of: 7
computing [7] of: 7
Directory has been created.
saving file: '"/data/LIO_SAM/session_HDmapping_walking_dataset/session.json"'
```

*Result*: Conversion to `session.json` failed due to missing imu topic (incorrect remapping).

## Recording 2 (OK)

```bash
rosbag record -o /data/LIO_SAM/walking_dataset_cloud_registered.bag /lio_sam/mapping/cloud_registered /lio_sam/mapping/cloud_registered /odometry/imu /lio_sam/imu/path
```

*Result*: correctly created: `20250506_0153_LIO_SAM_outputs/session.json` for `walking_dataset.bag`.

## Recording 3 (ConSLAM dataset)

*Input data*: `ConSLAM_data/seq1_recording.bag` (source: https://drive.google.com/drive/folders/1ZFSAI-DmhU5XbhveFJgODD9IlSUrq65U)

*Remapping of topics*:
```bash
rosbag play /data/ConSLAM_data/seq1_recording.bag --topics imu/data pp_points/synced2rgb /pp_points/synced2rgb:=/points_raw /imu/data:=/imu_raw
```

* Missing transforms*
- world to map
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map &
```
- between base_link and IMU (180deg yaw / around yawZ axis in ENU)
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 3.1416 0 0 base_link imu_link &
```
- between base_link and LiDAR (identity)
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar_link &
```

*Recording command*
```bash
rosbag record -o /data/ConSLAM_data/seq1_recording_cloud_registered.bag /lio_sam/mapping/cloud_registered /lio_sam/mapping/path /lio_sam/mapping/odometry /odometry/imu /lio_sam/imu/path
```

*Observations*:
- LIO-SAM uses transform definitions from `params.yaml` (not from tf!)
- params.yaml file is formatted during installation:
   > pre-installation example is in `ConSLAM_params.yaml`
   > post-installation requires replacing namespace "/**/ros__parameters:" with "/lio_sam:"

```bash
roslaunch lio_sam run.launch params_file:=/data/ConSLAM_data/LIO_SAM/ConSLAM_params.yaml
```

*Result*:

### Details

#### Transforms for ConSLAM dataset (Confirmed):
    extrinsicRot: [-1.0, 0.0, 0.0,
                      0.0, -1.0, 0.0,
                      0.0, 0.0, 1.0] == Rx_180deg @ Ry_180deg == Rz_180deg
    extrinsicRPY: [-1.0, 0.0, 0.0,
                   0.0, -1.0, 0.0,
                   0.0, 0.0, 1.0] == Rz_180deg

*Issues with IMU data*
- no covariances in IMU messages
- reversed axes (x,y,z)
- jumps in time (seq_01 recording is NOT continuous!)

IMU message example:
```
---
header:
  seq: 27219
  stamp:
    secs: 1647338599
    nsecs: 104913549
  frame_id: "imu_link"
orientation:
  x: -0.01629422977566719
  y: -0.022902904078364376
  z: -0.9911065101623535
  w: -0.13006895780563354
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.1418188214302063
  y: 0.01865998469293118
  z: 0.02285167947411537
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.12156610935926439
  y: 0.6140567064285278
  z: 9.877775192260742
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```


#### Reference IMU from `walking_dataset.bag`:
Transforms (Note: the Z axis of IMU acc had to be flipped so it is not a right-handed system!):
    extrinsicRot:    [-1.0,  0.0,  0.0,
                       0.0,  1.0,  0.0,
                       0.0,  0.0, -1.0 ] == Ry_180deg
    extrinsicRPY: [ 0.0,  1.0,  0.0,
                   -1.0,  0.0,  0.0,
                    0.0,  0.0,  1.0 ] == Rz_270deg

Definitions:
  - "extrinsicRot" transforms IMU gyro and acceleometer measurements to lidar frame.
  - "extrinsicRPY" transforms IMU orientation to lidar frame.

IMU message example:
```
header:
  seq: 272735
  stamp:
    secs: 1574367439
    nsecs: 475170295
  frame_id: "imu_link"
orientation:
  x: -0.07098197937011719
  y: -0.0364164337515831
  z: -0.9764178991317749
  w: 0.20060765743255615
orientation_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
angular_velocity:
  x: -0.07487387955188751
  y: 0.2663656175136566
  z: 0.39450016617774963
angular_velocity_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
linear_acceleration:
  x: 1.4558014538884163
  y: 0.7472857086360455
  z: -10.466804870367051
linear_acceleration_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
```