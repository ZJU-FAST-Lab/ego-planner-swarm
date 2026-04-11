# OAK-D EGO-Planner Depth + Odom Setup

## Overview
- This setup uses the repository's native `depth + odom` mapping path.
- `grid_map` consumes depth on `~grid_map/depth` and odom on `~grid_map/odom`.
- A temporary static odom publisher is provided for host-side validation.

## Build
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Launch
```bash
roslaunch ego_planner run_oakd_depth_odom.launch \
  depth_topic:=<your_depth_topic> \
  camera_fx:=<fx> camera_fy:=<fy> camera_cx:=<cx> camera_cy:=<cy> \
  depth_scale:=<depth_scale> depth_min:=0.2 depth_max:=5.0 \
  cam_tx:=<tx> cam_ty:=<ty> cam_tz:=<tz> \
  cam_qx:=<qx> cam_qy:=<qy> cam_qz:=<qz> cam_qw:=<qw>
```

## TEMP Items
- `TEMP:` `static_odom_*` parameters are only for host-side validation.
- `TEMP:` `cam_t*` and `cam_q*` are manual camera extrinsics and must be replaced by calibrated values later.
- `TEMP:` The launch default quaternion matches the repository's original optical-frame assumption, not your final calibrated OAK-D mounting.
- `TEMP:` `camera_f*`, `camera_c*`, and `depth_scale` are manually filled and should later come from calibration or `CameraInfo`.

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
