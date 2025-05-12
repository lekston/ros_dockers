# Docker

## Docker build

*Build base ROS2-GPU image*
```bash
sudo docker build -t ros2-dev-gpu-s1 -f Dockerfile_ros2_lio_sam .
```

*Build LIO-SAM image*
```bash
sudo docker build -t ros2-dev-liosam -f Dockerfile_ros2_lio_sam .
```

## Docker runcommand

## run command
```bash
sudo docker run -it --rm     --gpus all     --env="DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --volume="`pwd`:/root/src_wip"     --volume="/opt/mnt/data/10_slam/:/data/"     --network=host     --name ros2-dev-liosam     ros2-dev-gpu-liosam
```

tmux

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch lio_sam run.launch.py

```
ros2 node list
/lio_sam_featureExtraction
/lio_sam_imageProjection
/lio_sam_imuPreintegration
/lio_sam_imuPreintegration
/lio_sam_mapOptimization
```

ros2 interface show lio_sam/msg/CloudInfo

ros2 node info /lio_sam_imageProjection
ros2 node info /lio_sam_featureExtraction

ros2 pkg prefix lio_sam

## Parameters

```bash
vim /ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml
```

*Required Update*
imuTopic: "/imu_correct"

### LIO-SAM CloudInfo Message

```
ros2 interface show --no-comments lio_sam/msg/CloudInfo
```

```
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
int32[] start_ring_index
int32[] end_ring_index
int32[]  point_col_ind #
float32[] point_range #
int64 imu_available
int64 odom_available
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw
sensor_msgs/PointCloud2 cloud_deskewed  #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        uint32 height
        uint32 width
        PointField[] fields
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8
                string name      #
                uint32 offset    #
                uint8  datatype  #
                uint32 count     #
        bool    is_bigendian #
        uint32  point_step   #
        uint32  row_step     #
        uint8[] data         #
        bool is_dense        #
sensor_msgs/PointCloud2 cloud_corner    #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                builtin_interfaces/Time stamp                                                                                                      [0/297]
                        int32 sec
                        uint32 nanosec
                string frame_id
        uint32 height
        uint32 width
        PointField[] fields
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8
                string name      #
                uint32 offset    #
                uint8  datatype  #
                uint32 count     #
        bool    is_bigendian #
        uint32  point_step   #
        uint32  row_step     #
        uint8[] data         #
        bool is_dense        #
sensor_msgs/PointCloud2 cloud_surface   #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        uint32 height
        uint32 width
        PointField[] fields
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8
                string name      #
                uint32 offset    #
                uint8  datatype  #
                uint32 count     #
        bool    is_bigendian #
        uint32  point_step   #
        uint32  row_step     #
        uint8[] data         #
        bool is_dense        #
```
#### Detailed

```bash
ros2 interface show --all-comments lio_sam/msg/CloudInfo
```

```
# Cloud Info
std_msgs/Header header
        # Standard metadata for higher-level stamped data types.
        # This is generally used to communicate timestamped data
        # in a particular coordinate frame.

        # Two-integer timestamp that is expressed as seconds and nanoseconds.
        builtin_interfaces/Time stamp
                # This message communicates ROS Time defined here:
                # https://design.ros2.org/articles/clock_and_time.html

                # The seconds component, valid over all int32 values.
                int32 sec

                # The nanoseconds component, valid in the range [0, 10e9).
                uint32 nanosec

        # Transform frame with which this data is associated.
        string frame_id

int32[] start_ring_index
int32[] end_ring_index

int32[]  point_col_ind # point column index in range image
float32[] point_range # point range

int64 imu_available
int64 odom_available

# Attitude for LOAM initialization
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

# Initial guess from imu pre-integration
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
        # This message holds a collection of N-dimensional points, which may
        # contain additional information such as normals, intensity, etc. The
        # point data is stored as a binary blob, its layout described by the
        # contents of the "fields" array.
        #
        # The point cloud data may be organized 2d (image-like) or 1d (unordered).
        # Point clouds organized as 2d images may be produced by camera depth sensors
        # such as stereo or time-of-flight.

        # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
        std_msgs/Header header
                # Standard metadata for higher-level stamped data types.
                # This is generally used to communicate timestamped data
                # in a particular coordinate frame.

                # Two-integer timestamp that is expressed as seconds and nanoseconds.
                builtin_interfaces/Time stamp
                        # This message communicates ROS Time defined here:
                        # https://design.ros2.org/articles/clock_and_time.html

                        # The seconds component, valid over all int32 values.
                        int32 sec

                        # The nanoseconds component, valid in the range [0, 10e9).
                        uint32 nanosec

                # Transform frame with which this data is associated.
                string frame_id

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        uint32 height
        uint32 width

        # Describes the channels and their layout in the binary data blob.
        PointField[] fields
                # This message holds the description of one point entry in the
                # PointCloud2 message format.
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8

                # Common PointField names are x, y, z, intensity, rgb, rgba
                string name      # Name of field
                uint32 offset    # Offset from start of point struct
                uint8  datatype  # Datatype enumeration, see above
                uint32 count     # How many elements in the field


       bool    is_bigendian # Is this data bigendian?
        uint32  point_step   # Length of a point in bytes
        uint32  row_step     # Length of a row in bytes
        uint8[] data         # Actual point data, size is (row_step*height)

        bool is_dense        # True if there are no invalid points
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
        # This message holds a collection of N-dimensional points, which may
        # contain additional information such as normals, intensity, etc. The
        # point data is stored as a binary blob, its layout described by the
        # contents of the "fields" array.
        #
        # The point cloud data may be organized 2d (image-like) or 1d (unordered).
        # Point clouds organized as 2d images may be produced by camera depth sensors
        # such as stereo or time-of-flight.

        # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
        std_msgs/Header header
                # Standard metadata for higher-level stamped data types.
                # This is generally used to communicate timestamped data
                # in a particular coordinate frame.

                # Two-integer timestamp that is expressed as seconds and nanoseconds.
                builtin_interfaces/Time stamp
                        # This message communicates ROS Time defined here:
                        # https://design.ros2.org/articles/clock_and_time.html

                        # The seconds component, valid over all int32 values.
                        int32 sec

                        # The nanoseconds component, valid in the range [0, 10e9).
                        uint32 nanosec

                # Transform frame with which this data is associated.
                string frame_id

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        uint32 height
        uint32 width

        # Describes the channels and their layout in the binary data blob.
        PointField[] fields
                # This message holds the description of one point entry in the
                # PointCloud2 message format.
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5

                uint8 UINT32  = 6                                                                                                                 [25/521]
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8

                # Common PointField names are x, y, z, intensity, rgb, rgba
                string name      # Name of field
                uint32 offset    # Offset from start of point struct
                uint8  datatype  # Datatype enumeration, see above
                uint32 count     # How many elements in the field

        bool    is_bigendian # Is this data bigendian?
        uint32  point_step   # Length of a point in bytes
        uint32  row_step     # Length of a row in bytes
        uint8[] data         # Actual point data, size is (row_step*height)

        bool is_dense        # True if there are no invalid points
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
        # This message holds a collection of N-dimensional points, which may
        # contain additional information such as normals, intensity, etc. The
        # point data is stored as a binary blob, its layout described by the
        # contents of the "fields" array.
        #
        # The point cloud data may be organized 2d (image-like) or 1d (unordered).
        # Point clouds organized as 2d images may be produced by camera depth sensors
        # such as stereo or time-of-flight.

        # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
        std_msgs/Header header
                # Standard metadata for higher-level stamped data types.
                # This is generally used to communicate timestamped data
                # in a particular coordinate frame.

                # Two-integer timestamp that is expressed as seconds and nanoseconds.
                builtin_interfaces/Time stamp
                        # This message communicates ROS Time defined here:
                        # https://design.ros2.org/articles/clock_and_time.html

                        # The seconds component, valid over all int32 values.
                        int32 sec

                        # The nanoseconds component, valid in the range [0, 10e9).
                        uint32 nanosec

                # Transform frame with which this data is associated.
                string frame_id

        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        uint32 height
        uint32 width

        # Describes the channels and their layout in the binary data blob.
        PointField[] fields
                # This message holds the description of one point entry in the
                # PointCloud2 message format.
                uint8 INT8    = 1
                uint8 UINT8   = 2
                uint8 INT16   = 3
                uint8 UINT16  = 4
                uint8 INT32   = 5
                uint8 UINT32  = 6
                uint8 FLOAT32 = 7
                uint8 FLOAT64 = 8

                # Common PointField names are x, y, z, intensity, rgb, rgba
                string name      # Name of field
                uint32 offset    # Offset from start of point struct
                uint8  datatype  # Datatype enumeration, see above
                uint32 count     # How many elements in the field

        bool    is_bigendian # Is this data bigendian?
        uint32  point_step   # Length of a point in bytes
        uint32  row_step     # Length of a row in bytes
        uint8[] data         # Actual point data, size is (row_step*height)

        bool is_dense        # True if there are no invalid points
```