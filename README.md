# FAST-LIO-LOCALIZATION2

A simple localization framework that can re-localize in built maps based on [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2). 

## 1. Features
- Realtime 3D global localization in a pre-built point cloud map.
  By fusing low-frequency global localization (about 0.5~0.2Hz), and high-frequency odometry from FAST-LIO, the entire system is computationally efficient.

<div align="center"><img src="doc/demo.GIF" width=90% /></div>

- Eliminate the accumulative error of the odometry.

<div align="center"><img src="doc/demo_accu.GIF" width=90% /></div>

- The initial localization can be provided either by rough manual estimation from RVIZ2, or pose from another sensor/algorithm.

<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="center">
<img src="doc/demo_init.GIF" width=49.6% />
<img src="doc/demo_init_2.GIF" width = 49.6% >
</div>


## 2. Prerequisites
### 2.1 Dependencies for FAST-LIO-ROS2

Technically, if you have built and run FAST-LIO-ROS2 before, you may skip section 2.1.

This part of dependency is consistent with FAST-LIO-ROS2, please refer to the documentation https://github.com/Ericsii/FAST_LIO_ROS2

### 2.2 Dependencies for localization module

- python 3.8

- tf_transformations
```shell
pip install tf_transformations
```

- ros2_numpy
```shell
pip install ros2-numpy
```

check your numpy version, must be <1.24

- [Open3D](https://www.open3d.org/docs/release/getting_started.html)

```shell
pip install open3d
```
- pcl_ros
```shell
sudo apt-get install ros-$distro-pcl-ros
```

- To resolve numpy float issue - replace 'np.float' with 'np.float64' or just 'float' in the following file
```shell
cd /usr/lib/python3/dist-packages/transforms3d/
sudo nano quaternions.py
# replace 'np.float' with 'np.float64' or just 'float'
```


## 3. Build
Clone the repository and colcon build:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git
    cd FAST_LIO_LOCALIZATION2
    git submodule update --init
    cd ../..
    colcon build --symlink-install
    source install/setup.bash
```
- Remember to source the livox_ros_driver before build (follow [livox_ros_driver](https://github.com/hku-mars/FAST_LIO#13-livox_ros_driver))
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
  ```export PCL_ROOT={CUSTOM_PCL_PATH}```


## 4. Run Localization

### 4.2 Run

1. Run localization, here we take Livox Mid360 as an example:

```shell
ros2 launch fast_lio_localization localization.launch.py pcd_map_topic:=cloud_pcd map:=/path/to/your/map.pcd
```

Please modify `/path/to/your/map.pcd` to your own map point cloud file path (downsample it for fast rviz2 visualisation, we noticed a performance issue with dense pointclouds).

Wait for 3~5 seconds until the map cloud shows up in RVIZ;

Or if you are running realtime

```shell
roslaunch livox_ros_driver livox_lidar_msg.launch
```
Please set the **publish_freq** in **livox_lidar_rviz.launch** to **10Hz**, to ensure there are enough points for global localization in a single scan. 
Support for higher frequency is coming soon.

2. Provide initial pose
Use Rviz2 to provide an initial pose by using the '2D Pose Estimate' Tool in RVIZ2.

Note that, during the initialization stage, it's better to keep the robot still until the initialization succeeds. 


## Related Works
1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package
2. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
3. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): The integration of FAST-LIO with [Scan-Context](https://github.com/irapkaist/scancontext) **loop closure** module.
4. [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization): A simple system that can relocalize a robot on a built map based on LIO-SAM.

## 5.Additional Features
Added configuration to allow different lidar mounting orientation. Can be used if the lidar mounting is not parallel to the floor, such as in unitree g1 or other similar robots.
Modify mid360.yaml:
```
use_odom_transform: true  # true: transform scan to odom frame before matching (must match mapping config) -> change this to false if your lidar is already mounted parallel to the floor
odom_roll: 0.0        # Roll angle (degrees) from camera_init to odom
odom_pitch: 0.0        # Pitch angle (degrees) from camera_init to odom
odom_yaw: 0.0           # Yaw angle (degrees) from camera_init to odom
```

Transformation tree will become:
```
odom -> camera_init -> body -> base_link (where base_link is the frame parallel to the floor)
```


## Acknowledgments
Thanks for the authors of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization). This package is build on top of the work done by the ROS1 package of Fast-Lio-Localization - https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
