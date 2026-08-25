# 2026-08-12 离线验收记录

本记录对应全新独立工作空间，不包含旧 `planning` 控制器结果。所有闭环结果均由
MPCC、Dynamic Bicycle 仿真器和未修改的 `virtual_track/raceline.csv` 组成；没有向
实车底盘发布命令。

## 数值配置

- control period / OCP stage：0.05 s（20 Hz）；
- horizon：25 stages / 1.25 s；
- acados ERK：每 stage 5 个子步，有效积分步长 0.01 s；
- standalone RK4：每个 0.05 s 控制周期 5 个子步，有效积分步长 0.01 s；
- delay compensation：0.01 s 分段，每段 5 个 RK4 子步，有效步长 0.002 s；
- solver：SQP_RTI + PARTIAL_CONDENSING_HPIPM + GAUSS_NEWTON；
- vehicle / raceline：保持既有实车配置和 CSV 字节不变。

## 闭环结果

| 配置 | lap | mean/max speed | max abs contour | min track margin | max alpha f/r | max beta | max track/tire slack | solve mean/p95/max | failures |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tracking baseline, debug 1.0 m/s | 21.90 s | 0.990/1.000 m/s | 0.0272 m | 0.3738 m | 0.1907/0.0468 rad | 0.1369 rad | ~0/0.000031 rad | 2.161/2.499/2.973 ms | 0 |
| V1 approximate MPCC, debug 1.0 m/s | 22.05 s | 0.990/1.000 m/s | 0.0312 m | 0.3517 m | 0.0853/0.0228 rad | 0.1343 rad | ~0/0.000032 rad | 2.051/2.375/3.116 ms | 0 |
| V1 approximate MPCC, offline race 2.0 m/s | 11.15 s | 1.928/2.000 m/s | 0.0636 m | 0.4140 m | 0.1604/0.0689 rad | 0.1479 rad | ~0/0.000031 rad | 2.107/2.503/2.706 ms | 0 |

另外做了两组 1.0 m/s 初始扰动恢复测试：

- `e_c=+0.15 m, e_psi=+0.15 rad`：完成一圈，无 solver failure，最大误差
  0.154 m，最小 track margin 0.352 m；
- `e_c=-0.15 m, e_psi=-0.15 rad`：完成一圈，无 solver failure，最大误差
  0.155 m，最小 track margin 0.287 m。

三组均满足 50 ms solver deadline。2.0 m/s 项只证明名义模型的离线数值闭环，不能
代替 `Iz/Cf/Cr`、执行器与真实转角的实车辨识。

## 录制 Point-LIO bag 审计

输入：`/home/tianbot/localization_main/2026-08-12-00-37-29.bag`

- topic：`/localization/odom`，421 条，29.47 Hz；
- frame：`map -> body`；
- bag receipt time 减 header stamp：mean 0.262 s，p95 0.641 s，max 0.689 s；
- body `vx`：-0.097 到 2.247 m/s；
- bag 只含 odom，不含 steering/speed command，不能用于执行器或模型残差辨识。

因此控制器保留 timestamp + command FIFO 前向传播，0.30 s 后限制命令到 0.50 m/s，
0.75 s 后停车。高延迟下不能把 2.0 m/s 离线结果解释成可上车结果。

## 执行器候选值（未应用）

`2026-08-12-00-42-28.bag` 同时包含原生 Ackermann 命令和 Point-LIO odom。只读
拟合得到：

- speed static gain：0.7321；
- speed time constant：0.1246 s；
- speed dead time：0.19 s；
- speed fit RMSE：0.0545 m/s。

这与此前实车的“命令速度高于定位反馈速度”现象一致，但只有一段 ramp/stop excitation，
尚不足以覆盖多速度双向阶跃，故遵照“参数不变”要求没有写入活动 YAML。

steering 的 kinematic yaw-rate proxy 给出 gain 0.9803、tau 0.0357 s、delay 0.19 s，
但车辆无真实转角反馈，结果混入了 `Cf/Cr` 与侧滑，不能视作 steering identification，
同样没有应用。只读工具为 `identify_actuators.py`，车端原始输出保存在
`log/actuator_candidate_2026-08-12.json`。

## 尚未完成的规范门槛

- speed / steering actuator identification；
- `Cf/Cr/Iz` 实车初步辨识；
- 1.0、1.5、2.0 m/s 分级实车 residual 和闭环验收；
- Point-LIO 长延迟来源修复或实车预测误差验证；
- 3–5 m/s 只允许在前一级全部通过后评估。
