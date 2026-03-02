# MRS Openswarm Utils

ROS package containing utilities for MRS-based UAV operations.

This README section documents only the two nodes currently under active
organization:

- `global_map_provider`
- `map_global_generation`

## Common Service Contract

Both nodes use the same service type:

- `mrs_openswarm_utils/save_map`

File: [`srv/save_map.srv`](./srv/save_map.srv)

```srv
float32 resolution
string destination
---
sensor_msgs/PointCloud2 cloud
```

### Field meaning

- `resolution`:
  - For `global_map_provider`: voxel leaf size in meters.
  - `<= 0`, invalid, or empty-equivalent value means: keep original cloud resolution.
- `destination`:
  - Used by `map_global_generation` for optional local PCD save path.
  - Ignored by `global_map_provider`.
- `cloud`:
  - Returned point cloud (local map in provider, merged map in aggregator).

## global_map_provider

### Objective

Provide a SLAM-agnostic interface to request the latest global map from one UAV.
This decouples map retrieval from any specific SLAM package.

### Expected deployment

Run on **every UAV**. This supports leader changes during tests without changing
map retrieval architecture.

### ROS interfaces

- Subscribes:
  - `global_map_topic` (param-based, default: `global_map`)
- Service server:
  - `~get_global_map` (`mrs_openswarm_utils/save_map`)

When launched with namespace `uav1` and node name `global_map_provider`:
- service path is `/uav1/global_map_provider/get_global_map`

### Parameters

- `~global_map_topic` (`string`, default: `"global_map"`)
  - Source topic containing the latest global map from the selected SLAM stack.
- `~allow_empty_response` (`bool`, default: `false`)
  - If `false`: returns service failure when no map has been received yet.
  - If `true`: returns success with an empty cloud when no map is available.
- `~target_frame` (`string`, default: `""`)
  - If non-empty, transforms the returned cloud from source frame (e.g. `/uavX/odom`)
    into this frame before optional downsampling.
  - If empty, returns cloud in its original frame.
- `~tf_timeout` (`double`, default: `0.2`)
  - TF lookup timeout in seconds for transforming source frame to `~target_frame`.

### Behavior summary

1. Cache latest `sensor_msgs/PointCloud2` from `~global_map_topic`.
2. On service request:
   - If no cached map:
     - fail, or return empty cloud according to `~allow_empty_response`.
   - If `~target_frame` is configured and differs from source frame:
     - transform cloud to `~target_frame` via TF (timestamped lookup with latest-transform fallback).
     - if transform fails, service call fails.
   - If `resolution` is valid and positive:
     - apply voxel downsampling.
   - Otherwise:
     - return original cloud unchanged.

### Launch

File: [`launch/global_map_provider.launch`](./launch/global_map_provider.launch)

Example:

```bash
roslaunch mrs_openswarm_utils global_map_provider.launch UAV_NAME:=uav1 global_map_topic:=global_map

# Example with frame conversion
roslaunch mrs_openswarm_utils global_map_provider.launch UAV_NAME:=uav1 target_frame:=common_origin
```

## map_global_generation

### Objective

Request global maps from all UAVs in `uav_names`, optionally align maps using ICP
with respect to leader orientation, and provide a merged global map.

### Leader rule

The leader is **always** the first UAV in `uav_names`:

- `leader = uav_names[0]`

If ICP is enabled but leader map is unavailable, node falls back to direct merge.

### ROS interfaces

- Service server:
  - `~save_map` (`mrs_openswarm_utils/save_map`)
- Service clients (one per UAV):
  - `/<uav>/<global_map_service>`
  - default suffix: `global_map_provider/get_global_map`
- Publishes:
  - `~globalMap` (`sensor_msgs/PointCloud2`)

When launched with namespace `uav1` and node name `map_global_generation`:
- service path is `/uav1/map_global_generation/save_map`
- topic path is `/uav1/map_global_generation/globalMap`

### Parameters

- `~uav_names` (`string[]`, default: `[]`)
  - UAV list to query and merge.
- `~frame_output` (`string`, default: `"common_origin"`)
  - Frame id assigned to merged output cloud.
- `~global_map_service` (`string`, default: `"global_map_provider/get_global_map"`)
  - Service suffix called on each UAV.
- `~wait_time` (`double`, default: `5.0`)
  - Timeout for service availability and response wait per UAV.
- `~save_pcd_file` (`bool`, default: `true`)
  - Save merged and per-UAV clouds to disk.
- `~use_icp` (`bool`, default: `true`)
  - Enable/disable ICP alignment against leader map.

### Behavior summary

1. For each UAV in `uav_names`:
   - Wait for service with timeout `wait_time`.
   - Request map with timeout `wait_time`.
   - Keep only successful and non-empty clouds.
2. If no valid clouds are received: service returns failure.
3. Merge clouds:
   - ICP path when enabled and leader map exists.
   - Direct merge otherwise.
4. Apply final voxel filtering.
5. Return merged cloud in service response and publish on `~globalMap`.
6. Optionally save:
   - `GlobalMap.pcd`
   - `Map_uavX.pcd` for successful UAV clouds.

### Launch

File: [`launch/map_global_service.launch`](./launch/map_global_service.launch)

Example:

```bash
roslaunch mrs_openswarm_utils map_global_service.launch UAV_NAME:=uav1
```

### Notes on launch file

Current launch contains parameters `uav_name` and `save_map_topic` that are not
used by the current C++ implementation.

## Quick Test Commands

Provider test:

```bash
rosservice call /uav1/global_map_provider/get_global_map "{resolution: 0.0, destination: ''}"
```

Aggregator test:

```bash
rosservice call /uav1/map_global_generation/save_map "{resolution: 0.2, destination: ''}"
```

## Acknowledgement

Part of the source code in this repository is developed within the frame and for
the purpose of the OpenSwarm project. This project has received funding from the
European Union's Horizon Europe Framework Programme under Grant Agreement
No. 101093046.

![OpenSwarm - Funded by the European Union](logos/ack.png)
