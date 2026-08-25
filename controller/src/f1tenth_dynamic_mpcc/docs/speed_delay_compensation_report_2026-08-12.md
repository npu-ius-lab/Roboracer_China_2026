# TianRacer Dynamic MPCC：速度校准、延迟辨识与补偿模型报告

日期：2026-08-12  
车辆：TianRacer Ackermann / F1TENTH  
定位：MID360 + PointLIO，控制使用 `/localization/vehicle_odom`  
底盘命令：`/tianracer/ackermann_cmd` (`ackermann_msgs/AckermannDrive`)

## 1. 当前结论

1. 原始底盘速度反馈相对 PointLIO 真实速度约大 `38.7%`。底盘固件参数
   `motor_reduction` 已从 `8.8` 校准到 `12.2`，执行 `param save`、控制器复位后
   回读确认生效。
2. 校准后的 1.0、1.5、2.0、2.5 m/s 绕圈数据中，命令、底盘 odom 和 PointLIO
   稳态速度已经基本一致，误差约为 `-1.8% ~ +0.5%`。因此不应再启用原来的
   `1.387` 固定比例补偿，上层静态速度增益保持 `1.0`。
3. 当前主要剩余误差是动态滞后，而不是静态比例误差：
   - 底盘起步纯滞后约 `0.13 s`；
   - 纵向一阶时间常数约 `0.20 s`；
   - 达到 90% 稳态速度约需 `0.60 ~ 0.76 s`；
   - PointLIO 车辆中心 odom 到达控制器时，时间戳年龄中位数约 `0.11 s`。
4. 当前代码已经按 odom 时间戳把旧状态外推至当前时刻，但纵向模型只使用
   `speed_time_constant_s`；配置项 `speed_dead_time_s` 尚未接入纵向动力学或命令
   FIFO。因此，延迟参数已辨识，但尚不能仅靠修改 YAML 完成全部补偿。

## 2. 已确认的车辆和固件参数

### 2.1 底盘固件

| 参数 | 修改前 | 当前值 | 状态 |
|---|---:|---:|---|
| `base_type` | ackermann | ackermann | 已确认 |
| `wheel_r` | 0.055 m | 0.055 m | 未修改 |
| `motor_reduction` | 8.8 | 12.2 | 已保存并经复位回读 |
| `ticks_per_lap` | 8192 | 8192 | 未修改 |
| `ctrl_period` | 5 ms | 5 ms | 约 200 Hz |
| `feedback_period` | 20 ms | 20 ms | odom 约 50 Hz |
| `max_speed` | 10 m/s | 10 m/s | 固件上限；MPCC 另限 3 m/s |

固件读取命令：

```bash
rosservice call /tianracer/tianracer/debug_cmd_srv "cmd: 'param get'"
```

固件中的 `base_a/base_b` 对 `param set` 返回通用成功信息，但立即回读仍为
`0.125/0.163`，说明当前 RoboRacer 固件不允许通过该接口在线更新这两个值。
未强行覆盖，也没有把未生效的几何参数作为成功修改记录。

### 2.2 MPCC 几何参数

| 参数 | 当前值 | 来源 |
|---|---:|---|
| 轴距 `L` | 0.320 m | 用户实测 |
| 前轴到质心 `lf` | 0.195 m | 用户实测（2026-08-13 更新） |
| 后轴到质心 `lr` | 0.125 m | `L-lf` |
| 车宽 | 0.240 m | 用户实测 |

满足：

```text
lf + lr = 0.195 + 0.125 = 0.320 m
```

这些值已写入 `config/vehicle.yaml`，并通过 `validate_vehicle_model.py` 校验。

MID360/IMU 原点位于车辆中心 `localization_base_link` 前方约 `0.135 m`。
控制器使用已经移到车辆中心的 `/localization/vehicle_odom`，避免直接把雷达原点
当作车辆质心。

## 3. 速度反馈来源

Linux 端 `tianbot_core` 的数据路径为：

```text
/tianracer/ackermann_cmd
        -> ROS 驱动打包 PACK_TYPE_ACKMAN_VEL
        -> 串口发送到底盘 MCU
        -> MCU 回传 PACK_TYPE_ODOM_RESPONSE
        -> /tianracer/odom
```

`/tianracer/odom.twist.twist.linear.x` 来自 MCU 串口回传，不是 ROS 节点直接复制
速度命令。阶跃数据也证明它具有起步延迟、衰减和超调，不是命令回显。

控制所用的真实车辆中心运动状态为：

```text
/localization/vehicle_odom
  pose:  map -> localization_base_link
  twist: 车辆 body 坐标系下的 vx、vy、yaw rate
```

## 4. 静态比例校准

### 4.1 校准前

原始直线定速数据在 0.5~3.5 m/s 范围内给出：

```text
median(底盘 wheel odom / PointLIO vx) = 1.387
```

在保留实际轮径 `wheel_r=0.055 m` 的条件下，按照速度换算比例更新减速比：

```text
motor_reduction_new
  = motor_reduction_old * 1.387
  = 8.8 * 1.387
  = 12.2056
  ~= 12.2
```

### 4.2 校准后

| 命令速度 | 转角 | 底盘 odom 稳态 | PointLIO `vx` 稳态 | PointLIO 相对命令 |
|---:|---:|---:|---:|---:|
| 1.0 | -0.25 rad | 1.001 | 0.982 | -1.8% |
| 1.5 | -0.20 rad | 1.498 | 1.488 | -0.8% |
| 2.0 | -0.20 rad | 2.003 | 2.008 | +0.4% |
| 2.5 | -0.15 rad | 2.498 | 2.495 | -0.2% |

结论：固定比例误差已经由底盘参数校准消除。上层再次乘以 `1.387` 会造成重复
补偿和明显超速。

## 5. Bag 有效性审查

目录：

```text
/home/tianbot/f1tenth_mpcc/speed_identification/data/validation/
```

### 5.1 有效 bag

| Bag | 命令 | 有效恒速段 |
|---|---|---:|
| `circle_20260812_173924_20260812_173925.bag` | 1.0 m/s, -0.25 rad | 9.70 s |
| `circle_20260812_174048_20260812_174048.bag` | 1.5 m/s, -0.20 rad | 7.80 s |
| `circle_20260812_174021_20260812_174022.bag` | 2.0 m/s, -0.20 rad | 6.70 s |
| `circle_20260812_174130_20260812_174131.bag` | 2.5 m/s, -0.15 rad | 6.80 s |

### 5.2 无效 bag

| Bag | 剔除原因 |
|---|---|
| `circle_20260812_173707_20260812_173708.bag` | 总长 0.915 s，全程零命令 |
| `circle_20260812_174011_20260812_174012.bag` | 非零命令约 0.10 s，无可辨识恒速段 |

无效文件未删除。

## 6. 延迟辨识模型和方法

### 6.1 一阶惯性加纯滞后模型

采用 FOPDT（First-Order Plus Dead Time）描述纵向执行器：

```text
                 K_v
G_v(s) = -------------------- * exp(-L_v s)
              tau_v s + 1
```

阶跃上升的时域形式为：

```text
v(t) = v_0,                                      t <= t_cmd + L_v
v(t) = v_0 + K_v (v_cmd-v_0)
              * [1-exp(-(t-t_cmd-L_v)/tau_v)],  t > t_cmd + L_v
```

其中：

- `K_v`：静态增益；校准后约为 1；
- `L_v`：纯滞后；包括 ROS/串口传输、MCU 控制、死区和电机开始运动前的等效滞后；
- `tau_v`：速度建立的一阶时间常数。

命令阶跃时刻使用 bag 中 `/tianracer/ackermann_cmd` 的接收时间；速度分别使用
`/tianracer/odom` 和 `/localization/vehicle_odom`。每个有效 bag 独立拟合上升与停车
过程，并同时计算 10%、50%、90% 交叉时间。

### 6.2 各档加速结果

| 命令 | 底盘纯滞后 `L` | 底盘 `tau` | 底盘 10%/50%/90% | PointLIO 纯滞后 | PointLIO `tau` |
|---:|---:|---:|---:|---:|---:|
| 1.0 | 218 ms | 262 ms | 141/437/763 ms | 298 ms | 212 ms |
| 1.5 | 163 ms | 207 ms | 80/339/601 ms | 196 ms | 183 ms |
| 2.0 | 208 ms | 205 ms | 113/396/644 ms | 238 ms | 188 ms |
| 2.5 | 205 ms | 193 ms | 176/368/634 ms | 224 ms | 199 ms |

FOPDT 的 `L` 与首次 10% 交叉时刻定义不同：拟合的 `L` 会同时受低速死区、噪声和
一阶曲线形状影响。用于实时补偿时，采用较保守且不会过度前馈的汇总值：

```text
K_v   = 1.00
L_v   ~= 0.13 s   （按首次形成可测运动的中位量级）
tau_v ~= 0.20 s
```

若希望严格复现 FOPDT 最小二乘曲线，也可以使用约 `L_v=0.20 s`。实车闭环建议从
`0.13 s` 开始，逐步验证，避免过度预测。

### 6.3 停车结果

| 命令 | 底盘停车纯滞后 | 底盘停车 `tau` | PointLIO 停车纯滞后 | PointLIO停车 `tau` |
|---:|---:|---:|---:|---:|
| 1.0 | 110 ms | 142 ms | 200 ms | 82 ms |
| 1.5 | 117 ms | 163 ms | 193 ms | 114 ms |
| 2.0 | 157 ms | 186 ms | 277 ms | 112 ms |
| 2.5 | 116 ms | 225 ms | 284 ms | 114 ms |

加速和停车并不完全对称。当前模型使用一个统一 `tau_v` 是合理的一阶近似；若后续
高速制动误差明显，可扩展为 `tau_accel`、`tau_decel` 两套参数。

### 6.4 消息时间戳年龄

四个有效 bag 的完整时间跨度统计如下。频率按 `(消息数-1)/(末消息时间-首消息时间)`
计算；不能简单使用相邻消息间隔中位数的倒数，因为轮式 odom 和 PointLIO 可能成组
到达，间隔存在明显抖动。

| 信号 | 实测频率 | 标称周期/来源 | bag 接收时间减 header stamp |
|---|---:|---|---:|
| `/tianracer/odom` | 49.9~50.0 Hz | 约 20 ms，底盘轮式里程计 | 中位 18~19 ms，p95 约 40 ms |
| `/tianracer/imu` | 约 50.0 Hz | 约 20 ms，底盘 IMU | 中位约 0.1~0.2 ms* |
| `/livox/imu` | 约 200 Hz | 约 5 ms，MID360 内置 IMU | 中位约 0.1~0.2 ms* |
| `/localization/vehicle_odom` | 32.6~35.2 Hz | 约 29~31 ms，PointLIO输出 | 中位 107~119 ms，p95 134~151 ms |
| `/tianracer/ackermann_cmd` | 约 20.0 Hz | 50 ms，本次测试命令 | 无 header |

`*` IMU 的这个数值仅表示 ROS 消息 header 与 bag 接收时间接近，不自动证明硬件采样
到主机的总延迟也只有 0.1~0.2 ms；必须结合驱动如何赋值时间戳解释。

#### 6.4.1 命令响应延迟的参考时刻

命令响应分析以 bag 中 `/tianracer/ackermann_cmd.speed` 第一次从 0 变为目标速度的
记录时刻为 `t_cmd`，再检测速度达到最终稳态值 10%、50%、90% 的时刻：

```text
T10 = t(v >= v0 + 0.10*(v_final-v0)) - t_cmd
T50 = t(v >= v0 + 0.50*(v_final-v0)) - t_cmd
T90 = t(v >= v0 + 0.90*(v_final-v0)) - t_cmd
```

轮式里程计四档实测范围：

```text
T10 =  80~176 ms，中位约 127 ms
T50 = 339~437 ms
T90 = 601~763 ms
```

因此本报告推荐的约 `0.13 s` 起步滞后，准确含义是：

> 从 bag 首次观察到非零速度命令，到轮式里程计达到最终稳态速度约 10% 的时间。

它综合包含 ROS 命令传递、驱动串口发送、MCU 处理、电机 PID、机械死区、车辆开始
运动、编码器检测以及 odom 回传。它不是单独某一通信链路的延迟。

FOPDT 最小二乘拟合的 `L=0.16~0.22 s` 来自整条响应曲线，而不是 10% 阈值交叉，
所以数值比 `T10` 大并不矛盾。实时补偿从较保守的 0.13 s 开始，是为了避免模型
前推过量。

#### 6.4.2 ROS 消息时间戳年龄的参考时刻

消息年龄定义为：

```text
message_age = bag_record_time - message.header.stamp
```

但该数值是否代表真实传感器延迟，取决于 header stamp 在驱动链路的哪个环节生成。

`tianbot_core` 在主机收到 MCU 串口 odom 包后，使用 `ros::Time::now()` 给
`/tianracer/odom` 填写时间戳。因此测得的 18~19 ms 主要表示：

```text
主机驱动收到串口包 -> ROS发布/调度 -> rosbag订阅记录
```

它没有覆盖 MCU 内部编码器采样、速度计算、等待反馈周期和串口传输之前的全部时间。
所以不能根据这个数值断言轮式里程计从物理采样到控制器的总延迟只有 18~19 ms。
现有协议没有携带 MCU 硬件采样时间戳，若要精确测量完整轮速链路延迟，需要修改
固件协议或使用外部同步测量。

PointLIO 的 header stamp 对应被处理的雷达/IMU状态时刻，因此
`now-header.stamp ~= 0.11 s` 可以作为控制器收到状态时的实际状态年龄，并由当前
`DelayCompensator` 使用真实时间戳逐帧外推。

#### 6.4.3 更新频率不等于响应延迟

轮式 odom 为 50 Hz，只表示平均约每 20 ms产生一条新反馈，不表示车辆会在命令发出
20 ms 后达到目标速度。当前实车同时存在：

```text
轮式 odom 更新周期             ~= 20 ms
命令到可测运动 T10             ~= 130 ms
纵向一阶时间常数 tau_v         ~= 200 ms
命令到 90% 稳态速度 T90       ~= 600~760 ms
PointLIO 状态到达时的年龄       ~= 110 ms
```

这些量分别描述采样更新、执行器/车辆动力学和定位处理，建模时必须分开。

因此必须区分：

1. **执行器滞后**：命令发出后车辆还没有立即建立速度；
2. **测量滞后**：PointLIO 消息到达时，状态本身已经约旧了 0.11 s。

不能简单把 `0.13 + 0.11` 同时写入一个执行器延迟参数，否则状态时间戳外推和执行器
模型可能对同一段时间重复补偿。

## 7. 绕圈合理性检查

Ackermann 运动学理论半径：

```text
R_kin = L / |tan(delta_cmd)|
```

实测半径使用 `R_meas = |vx/r|`：

| 命令 | 理论半径 | PointLIO 实测半径 |
|---|---:|---:|
| 1.0 m/s, -0.25 rad | 1.253 m | 1.178 m |
| 1.5 m/s, -0.20 rad | 1.579 m | 1.480 m |
| 2.0 m/s, -0.20 rad | 1.579 m | 1.578 m |
| 2.5 m/s, -0.15 rad | 2.117 m | 1.953 m |

误差约 0~8%，表明四个有效 bag 的确是稳定绕圈，而不是定位静止、速度伪反馈或
明显跳变数据。

## 8. 当前 Dynamic MPCC 算法原理

### 8.1 状态和控制量

状态向量共 9 维：

```text
x = [X, Y, psi, vx, vy, r, delta, delta_c, theta]
```

- `X,Y,psi`：车辆中心在 map 中的位置和航向；
- `vx,vy,r`：body 坐标系纵向速度、侧向速度和横摆角速度；
- `delta`：估计的实际前轮转角；
- `delta_c`：舵机命令状态；
- `theta`：沿周期赛道的连续进度。

控制量共 3 维：

```text
u = [v_cmd, delta_c_dot, v_theta]
```

- `v_cmd`：底盘速度命令；
- `delta_c_dot`：舵机命令变化率；
- `v_theta`：虚拟赛道进度速度。

### 8.2 动态自行车模型

前后轮侧偏角：

```text
alpha_f = delta - atan2(vy + lf*r, vx_safe)
alpha_r =       - atan2(vy - lr*r, vx_safe)
```

线性轮胎侧向力：

```text
F_yf = C_f * alpha_f
F_yr = C_r * alpha_r
```

车体动力学：

```text
X_dot   = vx*cos(psi) - vy*sin(psi)
Y_dot   = vx*sin(psi) + vy*cos(psi)
psi_dot = r
vy_dot  = (F_yf + F_yr)/m - vx*r
r_dot   = (lf*F_yf - lr*F_yr)/I_z
```

纵向速度执行器目前实现为：

```text
vx_dot = clip((K_v*v_cmd - vx)/tau_v, -max_decel, max_accel)
```

转向执行器为：

```text
delta_target = clip(K_delta*delta_c + bias)
delta_dot    = clip((delta_target-delta)/tau_delta, +/-max_steer_rate)
```

低于约 0.45 m/s 时，模型在运动学横摆响应和动态轮胎模型之间平滑过渡，避免低速
时侧偏角分母过小造成刚性和数值不稳定。

### 8.3 赛道模型与循环运行

赛道 CSV 在内存中构造 C2 周期三次样条，末点到首点的闭合段也计入真实环长。
`theta` 使用非取模连续进度，查询几何时再 wrap 到一圈内，因此车辆可以持续多圈，
不会在一圈结束后停止。

在线速度规划同时考虑：

- 3.0 m/s 全局上限；
- 曲率对应的横向加速度上限；
- 最大转角；
- 转角变化率；
- 前向/后向纵向加减速连续性。

### 8.4 MPCC 代价和约束

轨迹误差分为：

```text
e_contour：垂直于参考线的轮廓误差
e_lag：沿参考切线方向的滞后误差
```

代价函数还包括航向、侧滑、横摆率、速度先验、控制量、控制变化率、进度一致性和
赛道前进奖励。主要硬/软约束包括：

- 左右赛道边界与车体半宽安全余量；
- 最大速度、转角和转角速率；
- 轮胎侧偏软约束；
- 最大横向加速度；
- 赛道 slack 和轮胎 slack 惩罚。

求解器使用 acados `SQP_RTI + HPIPM`，预测时域为 25 步、每步 0.05 s，总时域
1.25 s。每个 OCP 步内使用 5 个 ERK 子步，即 10 ms 数值积分步长。

### 8.5 无转角传感器观测器

车辆没有前轮转角传感器，因此 `delta` 由三层逻辑估计：

1. 低速：仅用舵机一阶执行器模型；
2. 中速：用 `atan(L*r/vx)` 形成运动学伪测量；
3. 高速：结合 `vx, vy, r, vy_dot, r_dot` 和前轮侧向力反推动态伪转角。

伪测量经过低通、创新限幅和小增益修正，避免 PointLIO 噪声直接驱动舵机状态。

## 9. 当前延迟补偿实现

当前数据流：

```text
PointLIO历史状态 x(t_m)
        |
        | 根据 header stamp 计算 age = now-t_m
        v
DelayCompensator + 已发布命令FIFO
        |
        | RK4 外推，基础步长10 ms，模型内部5子步
        v
预测状态 x(now + actuation_prediction)
        |
        v
赛道投影 -> acados MPCC -> 命令管理器 -> 底盘
```

已经实现：

- PointLIO 时间戳年龄检查；
- 依据已发布命令历史把旧状态预测到当前时刻；
- 最大预测时长保护；
- 高延迟时限速；
- 命令以严格 20 Hz 独立发布，不受求解器抖动直接影响；
- 速度和转角 slew rate 限制。

当前配置仍为：

```yaml
actuator:
  speed_static_gain: 1.0
  speed_time_constant_s: 0.15
  speed_dead_time_s: 0.0

timing:
  actuation_prediction_s: 0.03
```

其中 `speed_dead_time_s` 当前只存在于配置文件，`VehicleParameters` 和
`DynamicBicycleModel` 没有读取或使用它。不能把“配置项存在”理解为“纯延迟已经
补偿”。

## 10. 推荐补偿结构

### 10.1 静态增益

```text
K_v = 1.0
```

由固件 `motor_reduction=12.2` 负责静态标定。暂不需要上层 PID 或查表放大。

### 10.2 测量延迟补偿

继续使用消息真实 `header.stamp`：

```text
x_now = Integrate[x(t_m), u_history, t_m -> now]
```

PointLIO 的约 0.11 s 延迟不应写成固定常量，因为每帧 age 都可直接测量，且 p95
会变化。固定常量只适合作为监控基线。

### 10.3 执行器动态补偿

建议辨识值：

```yaml
speed_static_gain: 1.0
speed_time_constant_s: 0.20
speed_dead_time_s: 0.13
```

正确接入纯延迟有两种方案：

1. **命令 FIFO + 输入时间偏移（推荐）**：车辆模型在时刻 `t` 使用
   `v_cmd(t-L_v)`；控制初始状态先传播到 `now+L_v`，传播区间使用已经发出、无法
   撤回的命令，再从该时刻开始优化新命令。
2. **延迟链增广状态**：用多个一阶 lag 状态近似固定纯延迟。该方法更适合完全放进
   acados，但会增加状态维数和调参复杂度。

不能仅把 `actuation_prediction_s` 从 0.03 改成 0.13 后就认为完整实现了输入延迟；
现有外推器仍把命令视为立即作用，需要同时按 `L_v` 平移命令历史。

### 10.4 是否增加外环 PID

当前四档稳态误差已经很小，不建议立即增加高速 PID。外环积分会与 MPCC 的速度
状态、底盘电机 PID 和延迟补偿叠加，可能引入振荡。只有在不同电池电压、载荷和
地面条件下再次出现持续稳态偏差时，才考虑低带宽 trim：

```text
v_cmd_final = v_cmd_mpcc + clip(Kp*e_v + Ki*integral(e_v), small_limit)
```

并必须具备抗积分饱和、低速禁用、转弯限幅和定位失效冻结。

## 11. 推荐实施顺序

1. 将纵向时间常数从假设值 0.15 更新为辨识值 0.20，并运行离线测试。
2. 在模型参数中正式加入 `speed_delay`，实现按延迟偏移的命令 FIFO。
3. 增加单元测试：阶跃在 `L_v` 前不得响应，`L_v+tau_v` 时应达到约 63.2%。
4. 用四个有效 bag 做离线回放，比较无补偿/仅时间戳补偿/完整补偿的状态预测误差。
5. 实车先限速 1.0 m/s，再测试 1.5、2.0 m/s；每档同时做加速和停车。
6. 补录左转与右转数据，验证纵向动态是否受方向、侧滑影响。
7. 通过后再开放 2.5~3.0 m/s，不直接从当前数据批准高速闭环。

建议验收指标：

- 稳态速度误差绝对值 `< 3%`；
- 0~2 m/s 预测速度 RMSE `< 0.15 m/s`；
- 补偿后预测位置误差相对未补偿下降 `> 30%`；
- 不出现速度命令持续超调或转向振荡；
- PointLIO age 超过保护阈值时能够限速或安全减速。

## 12. 尚未完成的辨识

- 转向静态增益、偏置、纯延迟和时间常数尚未用转角传感器直接辨识；
- 质量、横摆惯量和轮胎侧偏刚度仍主要来自名义 F1TENTH 参数；
- 当前绕圈数据可以验证等效半径，但不足以独立辨识轮胎模型；
- 电池电压、地面附着和负载变化下的纵向模型尚未验证；
- 当前 `high_speed_closed_loop_approved` 应继续保持 false。

## 13. 复现实验命令

固定速度绕圈并自动录包：

```bash
cd /home/tianbot/f1tenth_mpcc

# 左转
./scripts/record_constant_circle.sh 1.0 0.25 0 circle_left_1mps

# 右转
./scripts/record_constant_circle.sh 1.0 -0.25 0 circle_right_1mps
```

参数依次为：

```text
速度(m/s)  转角(rad)  持续时间(s，0=直到Ctrl-C)  RUN_ID
```

按一次 `Ctrl-C` 后，脚本会发送 3 秒零速并正常关闭 bag。实验时必须停止 MPCC 的
命令发布器，保持遥控急停可用。
