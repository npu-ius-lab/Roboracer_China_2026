# Localization map virtual tracks

本目录保存当前 Point-LIO 二维定位地图，以及基于该地图生成的三套虚拟赛道。
三套赛道均为闭环，标称宽度为 `1.20 m`，并包含左右边界、中线、NMPC
raceline、ROS 地图和可视化结果。

## 输入地图

- `point_lio_map_2d.pgm`：原始二维占据栅格地图。
- `point_lio_map_2d.yaml`：地图分辨率、原点和阈值配置。

当前地图分辨率为 `0.05 m/pixel`。

## 三套赛道

| 目录 | 赛道形状 | 宽度 | 障碍物最小间距 | Raceline 长度 | 规划状态 |
| --- | --- | ---: | ---: | ---: | --- |
| `virtual_track/` | 基础椭圆闭环 | 1.20 m | 0.267 m | 22.235 m | `feasible_converged` |
| `virtual_track_u_turn/` | 两段直道和两个 180 度 U 弯 | 1.20 m | 0.255 m | 22.354 m | `feasible_converged` |
| `virtual_track_s_u/` | S 弯、U 弯和直道 | 1.20 m | 0.251 m | 22.455 m | `feasible_converged` |

三套 raceline 均包含 260 个等弧长参考点。完整浮点指标位于每个目录的
`virtual_track_boundaries_summary.json` 和 `raceline_summary.json`。

总览图：`three_virtual_tracks_overview.png`。

## 每套赛道的输出

```text
virtual_left_boundary.csv
virtual_right_boundary.csv
virtual_track_corridor.csv
centerline.csv
raceline.csv
virtual_track_map.pgm
virtual_track_map.yaml
virtual_track_boundaries_summary.json
raceline_summary.json
virtual_track_boundaries_debug.png
centerline_raceline_debug.png
```

- `virtual_left_boundary.csv`、`virtual_right_boundary.csv`：左右边界世界坐标。
- `virtual_track_corridor.csv`：同一采样位置的中心点、左右宽度和边界点。
- `centerline.csv`：中心线弧长、位姿、曲率、边界宽度和边界坐标。
- `raceline.csv`：经过曲率和平滑约束优化的 NMPC 跟踪参考。
- `virtual_track_map.pgm/.yaml`：绘制了虚拟边界的 ROS 地图。
- `*_debug.png`：边界或 raceline 与原始地图的叠加可视化。

## NMPC raceline 格式

```csv
s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,w_tr_right_m,w_tr_left_m
```

字段含义：

- `s_m`：闭环累计弧长，单位 m。
- `x_m,y_m`：地图世界坐标，单位 m。
- `psi_rad`：参考航向角，单位 rad。
- `kappa_radpm`：参考曲率，单位 rad/m。
- `vx_mps`：参考纵向速度，单位 m/s。
- `ax_mps2`：参考纵向加速度，单位 m/s^2。
- `w_tr_right_m,w_tr_left_m`：raceline 到左右边界的可用宽度，单位 m。

## 生成代码

代码位于 `../../tools/virtual_track/`：

- `generate_track_boundaries.py`：基础椭圆赛道边界。
- `generate_stadium_track.py`：U 弯和直道赛道边界。
- `generate_complex_s_u_track.py`：S 弯、U 弯和直道赛道边界。
- `generate_centerline_raceline.py`：中心线和 NMPC raceline。
- `generate_localization_map_racelines.sh`：从三套既有边界一键重算 raceline。
- `validate_localization_map_racelines.py`：实车接入前静态数值验收。
- `generate_three_track_overview.py`：生成三条线的总览图。
- `virtual_track_common.py`：地图读取、坐标转换和 CSV 公共函数。

## 一键重新生成三条 Raceline

在 `pointlio_lighterbev_ws` 根目录运行：

```bash
src/point_lio_sam_lighterbev/tools/virtual_track/generate_localization_map_racelines.sh
```

实车应让生成器和 NMPC 使用同一份车辆配置：

```bash
MAP_DIR="$(rospack find point_lio_sam_lighterbev)/point_lio/localization_map"
VEHICLE_CONFIG="$(rospack find f1tenth_nmpc_tracker)/config/tianracer_vehicle.yaml"
src/point_lio_sam_lighterbev/tools/virtual_track/generate_localization_map_racelines.sh \
  "$MAP_DIR" "$VEHICLE_CONFIG"
```

生成摘要中的 `source_vehicle_config` 会记录本次使用的配置来源。

这个入口只读取三套 `virtual_track_corridor.csv`，不会重画或覆盖已经确认的
左右边界。它统一重算 `centerline.csv`、`raceline.csv`、速度剖面、摘要和图片，
随后自动执行静态验收。

## 算法流程

1. 按同一索引配对左右边界，中心线取两点中点，并计算左右可用宽度。
2. 对闭环中心线做周期三次样条等弧长重采样，共 260 点。
3. 使用周期样条表示横向偏移 `alpha(s)`，优化曲率平方、偏移变化和平移量。
4. 约束 raceline 始终位于收缩后的赛道走廊内。收缩量包含半车宽、规划
   缓冲、在线跟踪余量和数值保护。
5. 根据曲率、最大转角、最大转角速率生成几何速度上限，再通过闭环前向/
   后向传播施加纵向加减速限制。
6. 验证 CSV schema、有限值、闭环连续性、地图范围、车体边界裕度、横向
   加速度、纵向加减速和转向约束，最后生成总览图。

默认车辆参数为轴距 `0.265 m`、车宽 `0.22 m`、最大速度 `1.85 m/s`、
最大转角 `0.55 rad`。优化后的 raceline 在扣除半车宽和 `0.08 m` 赛道
安全缓冲后，仍要求至少保留 `0.25 m` 的单侧跟踪余量。

上述检查是离线几何和运动学预检查，不等同于实车闭环通过。实车应先使用
`virtual_track/` 在低速完成坐标系、转向符号、急停和边界方向检查，再依次
测试 `virtual_track_u_turn/` 和 `virtual_track_s_u/`。
