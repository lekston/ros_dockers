# Docker run command

sudo docker run -it --rm     --gpus all     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --volume="`pwd`:/root/src_wip"     --volume="/opt/mnt/data/10_slam/:/data/"     --network=host     --name ros1-dev-liosam     ros1-dev-liosam

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