# V2 无舵角传感器转向状态估计

本次迁移保持 8 状态、3 输入结构不变，并将 `X[6]` 的语义固定为当前实际
等效前轮转角估计 `delta_hat`。`delta_cmd` 只进入静态 steering map 和一阶执行器
动态；前轮侧偏角始终使用状态 `delta`。

## 实现

- `state_layout.py`：统一状态/输入索引；
- `steering_actuator_model.py`：线性静态 map 和基于真实 dt 的解析一阶传播；
- `steering_state_observer.py`：命令队列预测，以及基于 Point-LIO `vx/vy/r` 的低带宽
  伪转角纠偏；
- `mpcc_node.py`：observer 在 odom callback 更新，MPCC `x0[6]` 使用 `delta_hat`；
- `run_closed_loop_sim.py`：真实 `delta_true` 仅属于 plant，controller 不可读取；
- `fit_tire_stiffness.py`：输出 `delta_cmd` 对照和 `delta_hat` 主拟合候选，不自动改配置。

低于 0.5 m/s 时 observer 处于 `MODEL_ONLY`，因为静止附近车辆运动不包含足够舵角
信息。短时 Point-LIO 间隔过大时保留并传播 `delta_hat`，不做噪声较大的车辆动态
纠偏。非有限观测进入 `INVALID`。当前 debug/hardware 配置分别将 `MODEL_ONLY` 和
`INVALID` 的速度限制为 1.0 m/s 和 0.35 m/s。

## 尚未辨识

当前 steering map gain/bias、时间常数、纯延迟、observer correction gain/cutoff 都是
保守初值，不是实车辨识结果。`Cf/Cr` 也尚未用独立实车数据和 `delta_hat` 重新验证，
因此不允许据此提高实车速度上限。

## 验收

- 本地和车端 22 个测试通过；
- 三圈无舵角传感器闭环：0 solver failure；
- `delta_hat` 对隐藏 `delta_true` RMSE：0.00657 rad；
- solver mean/P95/max：7.58/12.09/20.29 ms；
- 最大 contour error：0.0271 m；track slack 约 0。
