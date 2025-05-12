# Building cpp_pubsub

cpp_pubsub compilation (ROS1 version from FAST-LIVO)

Based on: https://github.com/marcinmatecki/fast-livo-to-hdmapping (ROS1 version)

## dependencies
```Dockerfile
RUN apt install nlohmann-json3-dev
```

```Dockerfile
RUN git clone https://github.com/LASzip/LASzip.git ./odometry_subscriber/src/3rdparty/LASzip
```

TODO: update to use submodules
```bash
git submodule update --init --recursive
```

Progress:
- updated to use livox_ros_driver2
  * CMakeLists.txt
  * package.xml

Important: source the livox drivers before building the cpp_pubsub package
```bash
source /livox_sdk/catkin_ws/devel/setup.bash
```

Submodules
```
[submodule "src/cpp_pubsub/src/3rdparty/LASzip"]
        active = true
        url = https://github.com/LASzip/LASzip.git
```

## build
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## datasets
LIO-SAM datasets for ROS1 are available here:
https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq

## run
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM

# need to provide both the bag file and the output directory
rosrun cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag /data/LIO_SAM/session_HDmapping /
```

## debug

```bash
rosrun --prefix 'gdb --args' cpp_pubsup listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag
/data/LIO_SAM/session_HDmapping
```

# Progress

- cpp_pubsup converter requires a topic with nav_msgs::Odometry - currently used "/odometry/imu"
- planned to use "/lio_sam/mapping/odometry" (required re-doing a recording)

# Related sources

- HDMapping:
  * `./apps/split_multi_livox/split_multi_livox.cpp`
  * `./apps/lidar_odometry_step_1/lidar_odometry.h`
  * `./apps/lidar_odometry_step_1/lidar_odometry.cpp`
  * `./core/include/point_clouds.h`
  * `./core/src/point_clouds.cpp`

# TODOs:
- consider a more meaningful name for cpp_pubsub (e.g.: bag2laz)
- add parameter handling for cpp_pubsub (so that topic names can be selected at runtime)
- streamline ROS1 and ROS2 versions of the cpp_pubsub:
  - ROS1 source: https://github.com/marcinmatecki/fast-livo-to-hdmapping
  - ROS2 source: https://github.com/marcinmatecki/kiss-icp-to-hdmapping