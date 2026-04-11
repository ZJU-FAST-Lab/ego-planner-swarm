# OAK-D Pro VIO + EGO-Planner Setup

## Overview
- This flow connects `OAK-D Pro stereo + IMU -> VINS-Fusion -> /vins_estimator/odometry -> ego-planner/grid_map`.
- The new one-shot launch is `run_oakd_vio_planner.launch`.
- Topic names stay fully parameterized. You can keep your existing OAK bridge or ROS driver.
- The planner keeps the existing depth interface and can start with manual camera extrinsics before replacing them with VINS extrinsics.

## Build
```bash
cd /home/xhn/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Launch Order
1. Start your OAK-D Pro bridge or driver so that left image, right image, depth image, and IMU are already publishing.
2. Start the integrated launch:
```bash
roslaunch ego_planner run_oakd_vio_planner.launch \
  left_image_topic:=/oakd/left/image_rect \
  right_image_topic:=/oakd/right/image_rect \
  imu_topic:=/oakd/imu \
  depth_topic:=oakd/stereo/depth
```
3. If you want custom waypoints, pass them directly:
```bash
roslaunch ego_planner run_oakd_vio_planner.launch \
  point_num:=3 \
  point0_x:=2.0 point0_y:=0.0 point0_z:=1.0 \
  point1_x:=2.0 point1_y:=2.0 point1_z:=1.0 \
  point2_x:=0.0 point2_y:=2.0 point2_z:=1.0
```

## Temporary Parameters To Fill
- `camera_fx`, `camera_fy`, `camera_cx`, `camera_cy`
- `depth_scale`, `depth_min`, `depth_max`
- `cam_tx`, `cam_ty`, `cam_tz`
- `cam_qx`, `cam_qy`, `cam_qz`, `cam_qw`
- `left_image_topic`, `right_image_topic`, `imu_topic`, `depth_topic`

These are enough to get the integrated chain running before final calibration is available.

## Parameters To Replace After Calibration
- `config/oakd_pro/left.yaml`
- `config/oakd_pro/right.yaml`
- `body_T_cam0`
- `body_T_cam1`
- `acc_n`, `gyr_n`, `acc_w`, `gyr_w`
- `camera_fx`, `camera_fy`, `camera_cx`, `camera_cy`
- `depth_scale`

If VINS extrinsic output becomes reliable, keep:
- `extrinsic_source:=param_and_topic`
- `vio_extrinsic_topic:=/vins_estimator/extrinsic`

If you want planner to trust only the VINS extrinsic topic, switch to:
```bash
extrinsic_source:=topic
```

## Main Interfaces
- VIO odom: `/vins_estimator/odometry`
- VIO path: `/vins_estimator/path`
- VIO extrinsic: `/vins_estimator/extrinsic`
- Planner depth input: `depth_topic`
- Planner real odom input: `vio_odom_topic`

## Verification
```bash
rostopic hz /vins_estimator/odometry
rostopic echo -n 1 /vins_estimator/extrinsic
rostopic echo -n 1 /drone_0_grid_map/occupancy
rostopic echo -n 1 /drone_0_grid_map/occupancy_inflate
```

## Troubleshooting
- No IMU:
  - Check `imu_topic` and timestamp continuity.
  - If VINS keeps printing `wait for imu`, the IMU stream is missing or behind the image stream.
- Left/right image mismatch:
  - Check `left_image_topic` and `right_image_topic`.
  - VINS stereo sync tolerance is tight; large timestamp skew will keep dropping frames.
- No extrinsic:
  - With `extrinsic_source:=param_and_topic`, planner can still start from `cam_t* / cam_q*`.
  - With `extrinsic_source:=topic`, planner will wait until `/vins_estimator/extrinsic` arrives.
- Depth encoding mismatch:
  - `grid_map` accepts both `16UC1` and `32FC1`.
  - If map output is wrong, re-check `depth_scale` and whether your depth topic is in meters or millimeters.
- Odom/depth timeout:
  - If planner reports timeout, verify both `/vins_estimator/odometry` and the depth topic are publishing continuously with close timestamps.
