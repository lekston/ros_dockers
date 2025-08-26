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

## Contine here

```
rosbag info /data/HILTI_2021/construction_site_1/Construction_Site_1.bag
rosbag info /data/HILTI_2021/uzh_tracking_area_run2/uzh_tracking_area_run2.bag
```