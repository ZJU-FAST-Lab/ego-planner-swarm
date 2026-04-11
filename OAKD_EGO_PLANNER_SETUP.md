# OAK-D EGO-Planner Depth + Odom Setup

## Overview
- This setup uses the repository's native `depth + odom` mapping path.
- `grid_map` consumes depth on `~grid_map/depth` and odom on `~grid_map/odom`.
- A temporary static odom publisher is provided for host-side validation.
- The launch now supports either `TEMP` static odom or a real external odom topic.
- The launch also exposes the planner's global waypoint interface directly.

## Build
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Launch
### Host-side validation with TEMP static odom
```bash
roslaunch ego_planner run_oakd_depth_odom.launch \
  depth_topic:=<your_depth_topic> \
  camera_fx:=<fx> camera_fy:=<fy> camera_cx:=<cx> camera_cy:=<cy> \
  depth_scale:=<depth_scale> depth_min:=0.2 depth_max:=5.0 \
  cam_tx:=<tx> cam_ty:=<ty> cam_tz:=<tz> \
  cam_qx:=<qx> cam_qy:=<qy> cam_qz:=<qz> cam_qw:=<qw>
```

### Real odom input
```bash
roslaunch ego_planner run_oakd_depth_odom.launch \
  use_temp_odom:=false \
  real_odom_topic:=/vins_estimator/odometry \
  depth_topic:=oak/stereo/image_raw \
  camera_fx:=<fx> camera_fy:=<fy> camera_cx:=<cx> camera_cy:=<cy> \
  depth_scale:=<depth_scale> \
  cam_tx:=<tx> cam_ty:=<ty> cam_tz:=<tz> \
  cam_qx:=<qx> cam_qy:=<qy> cam_qz:=<qz> cam_qw:=<qw>
```

### FCU / optical-flow odom input
If the flight controller already publishes `nav_msgs/Odometry`, use:
```bash
roslaunch ego_planner run_oakd_fcu_planner.launch \
  depth_topic:=oak/stereo/image_raw \
  fcu_odom_topic:=/mavros/local_position/odom
```

If the flight controller only publishes `geometry_msgs/PoseStamped`, enable the adapter:
```bash
roslaunch ego_planner run_oakd_fcu_planner.launch \
  depth_topic:=oak/stereo/image_raw \
  use_pose_adapter:=true \
  fcu_pose_topic:=/mavros/local_position/pose
```

The adapter republishes a standard odom topic for the planner:
- Input pose: `geometry_msgs/PoseStamped`
- Output odom: `nav_msgs/Odometry`
- Twist: zeroed as a `TEMP` fallback until a real FCU odom topic is available

### Global waypoint interface
```bash
roslaunch ego_planner run_oakd_depth_odom.launch \
  point_num:=3 \
  point0_x:=2.0 point0_y:=0.0 point0_z:=1.0 \
  point1_x:=2.0 point1_y:=2.0 point1_z:=1.0 \
  point2_x:=0.0 point2_y:=2.0 point2_z:=1.0
```

Optional:
```bash
flight_type:=2
```

## TEMP Items
- `TEMP:` `static_odom_*` parameters are only for host-side validation.
- `TEMP:` `temp_odom_topic` is only used when `use_temp_odom:=true`.
- `TEMP:` `cam_t*` and `cam_q*` are manual camera extrinsics and must be replaced by calibrated values later.
- `TEMP:` The launch default quaternion matches the repository's original optical-frame assumption, not your final calibrated OAK-D mounting.
- `TEMP:` `camera_f*`, `camera_c*`, and `depth_scale` are manually filled and should later come from calibration or `CameraInfo`.

## Interfaces Left for Later
- Real odom interface:
  - `use_temp_odom:=false`
  - `real_odom_topic:=/your/odometry`
- FCU odom interface:
  - `fcu_odom_topic:=/mavros/local_position/odom`
  - `use_pose_adapter:=true`
  - `fcu_pose_topic:=/mavros/local_position/pose`
  - `adapted_odom_topic:=/drone_0_fcu/odom`
- Global waypoint interface:
  - `point_num`
  - `point0_x/y/z` ... `point4_x/y/z`
- Camera extrinsic interface:
  - `cam_tx/ty/tz`
  - `cam_qx/qy/qz/qw`

## Verify
```bash
rostopic list
rostopic info <your_depth_topic>
rostopic hz <your_depth_topic>
rostopic echo -n 1 /drone_0_oakd/temp_odom
rostopic echo -n 1 /drone_0_grid_map/occupancy
rostopic echo -n 1 /drone_0_grid_map/occupancy_inflate
```

## Foxglove Topics
- `/drone_0_grid_map/occupancy`
- `/drone_0_grid_map/occupancy_inflate`
- `/drone_0_planning/bspline`
