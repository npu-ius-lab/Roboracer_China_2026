# V5 Chaoche（独立候选）

本目录只为 Stable V5 的超车实验提供感知参数和局部参考管理。原 Stable V5
启动脚本、控制节点、配置、赛道和 residual 模型均不修改，并由原脚本的 SHA-256
门禁继续校验。

数据流：

```text
MID360 点云 -> /v5_chaoche/perception/targets_detailed
            -> 原无控制权局部超车规划器
            -> /v5_chaoche/overtake/selected_path
            -> V5 Chaoche 路径锁存与返回确认
            -> /v5_chaoche/local_reference
            -> V5 Chaoche MPCC
```

行为约束：

- PREPARE/PASS 开始时锁定一条局部路径，动作期间不接受新候选替换。
- PREPARE、PASS、RETURN、ABORT 都由 MPCC 跟踪；不会切换到 Pure Pursuit。
- 只有到达局部路径末端、车辆回到全局线 `|ey| <= 0.12 m`，并连续 0.60 s
  未检测到赛道内障碍物，才清空局部参考并恢复全局 MPCC。
- 局部参考管理器失联时保留当前局部参考并把速度上限置零，不会突然跳回全局线。
- 感知/状态消息超过 0.35 s 未更新时进入 ABORT 限速，过期的“无障碍”结果不能触发回全局。
- 参考管理器没有 Ackermann 消息依赖，也不发布任何车辆控制命令。

常用命令：

```bash
cd /home/tianbot/f1tenth_residual_controller_ws
./scripts/start_v5_chaoche.sh --check-only
./scripts/start_v5_chaoche.sh --static-test
./scripts/start_v5_chaoche.sh --shadow
```

不带参数是默认实车模式；`V5_CHAOCHE_ALLOW_REAL_HARDWARE` 未设置时按 `YES`
处理。`--shadow` 会启动实验感知、规划和 MPCC 节点并录 bag，但不启动硬件输出
supervisor。

实车控制链已经接通，但不会默认获得控制权：

```text
/v5_chaoche/local_reference
  -> V5 Chaoche MPCC
  -> /v5_chaoche/mpcc/ackermann_cmd_stamped
  -> automatic_relaunch_supervisor_v5_chaoche
  -> /tianracer/ackermann_cmd
```

`--static-test` 会检查上述话题、使能服务和唯一硬件发布者，整个过程不启动 ROS
节点。默认实车启动命令为：

```bash
./scripts/start_v5_chaoche.sh
```

需要强制禁止实车输出时可设置
`V5_CHAOCHE_ALLOW_REAL_HARDWARE=NO`，或使用 `--shadow`。运行默认命令前仍应确认
车辆位于赛道、物理急停可用且有人负责急停。

硬件 supervisor 会先验证定位、车辆落地、赛道边界和起步位置，再使能 V5
Chaoche MPCC；局部超车参考有效时走局部 MPCC，返回确认完成后走全局 MPCC。

RViz 中红色路径话题为 `/v5_chaoche/local_plan_red`。状态分别发布在
`/v5_chaoche/reference_status`（规划管理）和
`/v5_chaoche/mpcc_reference_status`（控制侧是否使用局部/全局参考）。

## 关联参数 bag 回归

使用 `2026-08-21-23-47-49.bag` 的 423 帧对齐点云回放，最终选择 `gate=0.50`：

| 指标 | 原 0.40 | V5 Chaoche 0.50 |
| --- | ---: | ---: |
| 有目标帧占比 | 42.3% | 48.6% |
| 最长目标间隔 | 21.19 s | 14.99 s |
| 重复目标帧 | 0 | 2 |
| 连续可见期间 ID 切换 | 2 | 3 |

`gate=0.65` 虽然达到 50.6% 有目标帧，但产生了 15 帧重复目标，因此未采用。
这版仍不把 ID 连续性当作超车安全条件；动作开始后锁定局部参考，后续 ID 抖动
不会替换正在执行的超车线。
