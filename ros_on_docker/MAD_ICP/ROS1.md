# Datasets

HILTI 2021:
https://hilti-challenge.com/dataset-2021.html

## Build and run the docker image
```bash
$ sudo ./ros_on_docker_nvidia/run_ros1_docker_gpu_mad_icp.sh
```

# Running

Arrange your data so that every single rosbag is placed in a dedicated directory.

Dataset 1 "RPG Drone Testing Arena" (6 DoF Ground truth available):
```
mad_icp --data-path /data/HILTI_2021/construction_site_1/ --estimate-path /data/HILTI_2021/construction_site_1/output/ --dataset-config hilti_2021
```

Dataset 2 "Construction Site Outdoor 1":
```
mad_icp --data-path /data/HILTI_2021/uzh_tracking_area_run2/ --estimate-path /data/HILTI_2021/uzh_tracking_area_run2/output/ --dataset-config hilti_2021
```

Optional args:
`--noviz` - starts without the build-in (based on `open3d`) visualization window.
`--noros` - starts without ROS publisher

## Running on NTU Viral dataset

```
mad_icp --data-path /data/NTU_Viral_dataset/eee_02/ --estimate-path /data/NTU_Viral_dataset/eee_02/output/ --dataset-config /catkin_ws/src/mad-icp/mad_icp/configurations/datasets/ntu_viral.cfg --noviz
```

*Recording*
```
rosbag record -o /data/NTU_Viral_dataset/MAD_ICP/eee_02_results.bag /cloud/current /cloud/complete  /odometry/imu
```

NOTE: consider remapping to `/cloud_registered`

## Running on Hilti 2021 dataset

```
rosbag info /data/HILTI_2021/construction_site_1/Construction_Site_1.bag
rosbag info /data/HILTI_2021/uzh_tracking_area_run2/uzh_tracking_area_run2.bag
```