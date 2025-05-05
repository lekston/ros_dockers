# Building cpp_pubsub

cpp_pubsub compilation (ROS1 version from FAST-LIVO)

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
catkin_make
```

## run 
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM

# need to provide both the bag file and the output directory
rosrun cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag /data/LIO_SAM/session_HDmapping /
```

# Sources

- HDMapping:
  * `./apps/split_multi_livox/split_multi_livox.cpp`
  * `./apps/lidar_odometry_step_1/lidar_odometry.h`
  * `./apps/lidar_odometry_step_1/lidar_odometry.cpp`
  * `./core/include/point_clouds.h`
  * `./core/src/point_clouds.cpp`
