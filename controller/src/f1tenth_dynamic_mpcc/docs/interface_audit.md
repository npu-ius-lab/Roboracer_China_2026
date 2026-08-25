# Phase 0：实车接口审计

审计日期：2026-08-12。此文档只记录接口事实；在接口、坐标系和符号确认前，
新控制器禁止连接真实底盘话题。

| 项目 | 审计结果 | 证据/备注 |
|---|---|---|
| Point-LIO 状态话题 | `/localization/odom` | `robust_localizer.cpp` 发布 |
| 消息类型 | `nav_msgs/Odometry` | 源码与实测 bag 一致 |
| pose frame | `header.frame_id=map` | pose 表达 `map -> body` |
| velocity frame | `child_frame_id=body` | 线速度发布前执行 `R_odom_body.transpose() * v_world` |
| yaw-rate 来源 | Point-LIO bias-corrected IMU gyro/filter `omg` | body 系，单位 rad/s |
| 控制器期望输出 | `AckermannDriveStamped` | 新项目默认仅发 debug topic |
| 底盘原生命令 | `/tianracer/ackermann_cmd`，`AckermannDrive` | 驱动订阅后将两个 float 原样打包发串口 |
| steering_angle 单位 | rad | ROS Ackermann 约定，驱动无额外缩放 |
| steering_angle 正方向 | 正值左转、负值右转 | body x 前、y 左、yaw CCW 正；实车日志中正转角对应正 yaw-rate |
| speed 单位 | m/s | 正值沿 body +x 前进 |
| command timestamp | Stamped 控制层使用发布时间；原生驱动消息无 Header | 底盘驱动不使用时间戳，控制器必须自行做 freshness gate |
| 真实转角反馈 | 无 | 驱动只接收命令，未发布 steering encoder |
| wheel odom | `/tianracer/odom`，`nav_msgs/Odometry` | `odom -> base_footprint`；仅记录诊断，不作 MPCC 主状态 |
| raceline 加载者 | 新项目 `PeriodicTrack` | 原始 CSV 字节不修改，哈希记录于 `config/track.yaml` |
| 控制触发频率 | 20 Hz | `controller.yaml`；每周期只使用最新 odom |

## Point-LIO twist 结论

Point-LIO 内部线速度状态最初位于 odom/world 系，发布 `/aft_mapped_to_init` 前转换为：

```text
v_body = R_odom_body^T v_world
```

后端生成 `/localization/odom` 时只将 pose 由 odom 修正到 map，并原样复制同一时刻
的 body 系 twist。因此控制器读取：

```text
linear.x = body vx
linear.y = body vy
angular.z = body yaw rate
```

最近 bag 的 pose 差分与 twist 交叉验证表明纵向速度比例约为 1.000。

## 尚未辨识的接口属性

- `speed command -> Point-LIO vx` 的静态映射、时间常数和纯延迟；
- `steering command -> actual equivalent front-wheel angle` 的映射、时间常数和纯延迟；
- 真实转角状态（当前只能由 actuator model 估计）；
- 轮胎 `Cf/Cr` 与 slip envelope。

这些参数在 `vehicle.yaml` 中保持名义值/assumed 状态，不得宣称为实车辨识结果。
