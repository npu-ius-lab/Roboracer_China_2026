# 转向执行器安全数据采集

本流程只在 `steering-actuator-identification` 分支使用。`stable` 分支与
`stable-20260815-v1` 标签不会被修改。

## 启动前

先启动 TianRacer/MID360 与 Point-LIO 定位，使以下接口在线：

- ROS master：远端小车；
- 控制订阅：`/tianracer/ackermann_cmd`；
- 定位：`/localization/vehicle_odom`，frame 必须是
  `map -> localization_base_link`；
- IMU：优先使用 `/livox/imu`。

不要同时启动 MPCC、Pure Pursuit 或其他控制发布者。采集器检测到控制话题已有
发布者时会拒绝启动。车辆需放在 raceline 附近并朝正向，遥控急停保持在手边。

## 先预览，不发实车命令

```bash
./scripts/start_steering_prbs_collection.sh --plan
./scripts/start_steering_prbs_collection.sh track_prbs_100 --plan
```

## 推荐采集顺序

每条命令都会直接运行实车、自动录包，并在达到设定圈数后平滑停车：

```bash
./scripts/start_steering_prbs_collection.sh track_static_060 static_060_run01
./scripts/start_steering_prbs_collection.sh track_prbs_060 prbs_060_run01
./scripts/start_steering_prbs_collection.sh track_prbs_100 prbs_100_run01
./scripts/start_steering_prbs_collection.sh track_prbs_150 prbs_150_run01
```

四种 profile 分别用于低速静态映射，以及 0.6、1.0、1.5 m/s 的动态响应。
激励只在低曲率、横向/航向误差较小且车身到边界余量足够时启用；进入弯道时
激励时钟暂停并回到闭环赛道跟踪。候选激励每 0.1 s 经过一秒、多执行器模型
安全 rollout。候选余量不足 0.15 m 时只撤销激励；稳定跟踪本身预测不足
0.12 m 或当前余量不足 0.15 m 时才执行停车。

激励区以 `|curvature| < 0.20 rad/m、margin >= 0.32 m` 进入，以
`|curvature| < 0.22 rad/m、margin >= 0.28 m` 保持，避免阈值附近抖动切碎
0.9 s 静态平台。更宽的保持区不取代安全预测；任一方向的预测余量不足仍立即
撤销激励。

安全 rollout 在独立进程中运行，并同时检查零激励、正激励和负激励三个分支，
使用左右方向中更小的边界余量。ROS 主进程继续以 50 Hz 接收 odom 和发布
命令。metadata 会记录安全预测耗时、结果年龄、控制循环最大周期和 deadline
错过次数。预测结果超过 0.40 s 时暂停激励并把速度降到 0.25 m/s；超过 1.0 s
才判定安全预测器失效。定位 odom 的独立断流阈值仍保持 0.35 s，不会被放宽。

数据默认保存到 `lateral_identification/data/`：

- `<run_id>.bag`：完整 ROS bag；
- `<run_id>.metadata.json`：安全余量、有效激励时长、抑制次数与模型包络；
- `<run_id>.identification.json/.md/.png`：单包初步分析。

## 多包联合拟合

完成至少两包后可以离线运行：

```bash
./scripts/analyze_steering_actuator_joint.sh \
  lateral_identification/data/static_060_run01.bag \
  lateral_identification/data/prbs_060_run01.bag \
  lateral_identification/data/prbs_100_run01.bag \
  lateral_identification/data/prbs_150_run01.bag \
  --output lateral_identification/data/steering_actuator_joint_v1.json
```

联合工具按完整 bag 等权拟合，执行留一 bag 验证和按 bag 的 bootstrap。结果只
生成候选报告，不会自动写入控制器。模型通过独立未激励 bag 的多步 rollout
验收后，才进入 MPCC 执行器状态集成。
