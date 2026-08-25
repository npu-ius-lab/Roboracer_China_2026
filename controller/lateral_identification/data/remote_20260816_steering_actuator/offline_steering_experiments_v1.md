# TianRacer 转向执行器本地离线实验

生成时间：`2026-08-16T02:16:52`

状态：`offline_stage_5_all_local_data_complete_candidate_low_speed_only_high_speed_gate_failed_not_applied`

本报告完成当前本地 bag 能支持的辨识和影子回放，只生成候选；没有修改 stable、vehicle.yaml、controller.yaml 或实车配置。

## 数据划分

- 4 个 8 月 16 日 completed/最佳里程 bag 作为 primary，只用于拟合与模型选择。
- 5 个 bag 作为 holdout：1 个 8 月 16 日暂停 bag，加 4 个外部历史双向转向 bag；均未参与当前候选拟合。
- 暂停区间由连续运动窗口过滤，不会作为车辆响应样本。

## 数据审计

- `track_static_060_20260816_005542.bag`：completed=True，有效运动 55.00 s，最长暂停 2.89 s，cmd/IMU/vehicle_odom=49.8/200.0/30.3 Hz，Point-LIO age P95=0.137 s，位移/航向跳变=0/0
- `track_prbs_060_20260816_005954.bag`：completed=False，有效运动 36.01 s，最长暂停 0.89 s，cmd/IMU/vehicle_odom=49.9/200.0/29.9 Hz，Point-LIO age P95=0.135 s，位移/航向跳变=0/0
- `track_prbs_100_20260816_010820.bag`：completed=False，有效运动 41.70 s，最长暂停 17.24 s，cmd/IMU/vehicle_odom=48.8/200.0/29.7 Hz，Point-LIO age P95=0.139 s，位移/航向跳变=0/0
- `track_prbs_150_20260816_011504.bag`：completed=True，有效运动 33.48 s，最长暂停 6.62 s，cmd/IMU/vehicle_odom=49.6/200.0/30.6 Hz，Point-LIO age P95=0.136 s，位移/航向跳变=0/0
- `track_prbs_150_20260816_011147.bag`：completed=False，有效运动 22.70 s，最长暂停 28.71 s，cmd/IMU/vehicle_odom=48.5/200.0/29.2 Hz，Point-LIO age P95=0.142 s，位移/航向跳变=0/0
- `actuator_run01.bag`：completed=True，有效运动 36.95 s，最长暂停 2.90 s，cmd/IMU/vehicle_odom=50.0/200.0/31.5 Hz，Point-LIO age P95=0.145 s，位移/航向跳变=0/0
- `actuator_run02.bag`：completed=True，有效运动 36.98 s，最长暂停 2.88 s，cmd/IMU/vehicle_odom=50.0/200.0/32.2 Hz，Point-LIO age P95=0.146 s，位移/航向跳变=0/0
- `lateral_mpcc_20260814_185646_0.bag`：completed=False，有效运动 101.74 s，最长暂停 0.00 s，cmd/IMU/vehicle_odom=20.0/50.0/32.1 Hz，Point-LIO age P95=0.144 s，位移/航向跳变=0/0
- `mpcc_40hz_td0_20260814_193817.bag`：completed=False，有效运动 97.70 s，最长暂停 0.62 s，cmd/IMU/vehicle_odom=50.0/200.0/32.9 Hz，Point-LIO age P95=0.144 s，位移/航向跳变=0/0

## 传感器时间对齐（header stamp 域）

- Point-LIO yaw-rate 相对 IMU 的跨 bag 中位信号滞后：`0.010 s`。
- Point-LIO vx 相对轮式里程计的跨 bag 中位信号滞后：`-0.075 s`。
- 这是去除仿射比例后的信号形状对齐，不是 ROS 接收 age；不能与舵机 Td 或 Point-LIO 网络 age 直接相加。转向辨识使用 IMU yaw-rate 的 header stamp，因此没有把约 0.14 s 的 Point-LIO 接收 age 当成舵机延迟。

## 静态映射

- 平台数：`33`
- 命令覆盖：`0.0162 .. 0.2954 rad`
- 推荐映射：`linear`
- 方向覆盖：`one_sided_total_command_only`
- 回差代理：`0.02773 rad`
- 结论：本批总转角命令几乎只有正向平台，因此不能据此辨识左右不对称、死区或真实机械回差。

## 历史固定圆负向转向检查

- `circle_20260812_173924_20260812_173925`：vx=`0.98 m/s`，cmd=`-0.250 rad`，等效角=`-0.265 rad`，半径=`1.179 m`；stable/candidate RMSE=`0.0254/0.0302 rad`，split=`0.0279 rad`
- `circle_20260812_174021_20260812_174022`：vx=`2.01 m/s`，cmd=`-0.200 rad`，等效角=`-0.202 rad`，半径=`1.563 m`；stable/candidate RMSE=`0.0118/0.0348 rad`，split=`0.0334 rad`
- `circle_20260812_174048_20260812_174048`：vx=`1.50 m/s`，cmd=`-0.200 rad`，等效角=`-0.212 rad`，半径=`1.489 m`；stable/candidate RMSE=`0.0192/0.0274 rad`，split=`0.0262 rad`
- `circle_20260812_174130_20260812_174131`：vx=`2.51 m/s`，cmd=`-0.150 rad`，等效角=`-0.163 rad`，半径=`1.940 m`；stable/candidate RMSE=`0.0153/0.0166 rad`，split=`0.0168 rad`
- 固定圆中 candidate 优于 stable 的数量：`0/4`。
- 固定圆中 exploratory split 优于 stable 的数量：`0/4`。
- 固定圆无持续动态激励，且等效角含轮胎侧偏；它不能估计 Td/tau，但能暴露当前单一正负对称仿射增益在负向稳态上的风险。

## 探索性正负分段增益

- K_neg/K_pos：`1.08002/1.13377`，bias=`-0.01637 rad`，tau=`0.07215 s`。
- primary/holdout 平均动态 RMSE 改善：`2.05%/3.37%`。
- 这是决定下一轮数据采集是否值得做的探索，不可部署：仍缺当前配置下独立的正负静态平台和真实前轮角。

## 单步联合 FOPDT

- K：`1.107931`
- b：`-0.012963 rad`
- Td：`0.105000 s`
- tau：`0.061888 s`
- Td + tau：`0.166888 s`
- 动态 RMSE/R2：`0.020960 rad / 0.97522`
- 4 个 primary 的 K、b、Td、tau、Td+tau 跨 bag 一致性门槛全部通过；严格 leave-one-bag RMSE <0.020 rad 仅差约 0.0004–0.0018 rad，未严格通过。
- `Td` 与 `tau` 存在明显等效脊，最可靠的是总响应时间约 0.16–0.17 s，不是二者唯一拆分。

## 多时域输出误差候选（推荐继续验证）

- K：`1.113230`
- b：`-0.011165 rad`
- Td：`0.095000 s`
- tau：`0.066662 s`
- Td + tau：`0.161662 s`
- 该候选只由 4 个 primary 的 0.1/0.2/0.5/1.0 s yaw-rate、heading、lateral 输出误差拟合；holdout 未参与。

### 舵机隔离 rollout 的 holdout 结果

- `stable_nominal` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate 相对 stable 改善 `0.0%`，0.5 s lateral P95=`0.0813 m`
- `historical_candidate` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate 相对 stable 改善 `61.2%`，0.5 s lateral P95=`0.0601 m`
- `joint_fopdt` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate 相对 stable 改善 `59.7%`，0.5 s lateral P95=`0.0635 m`
- `two_stage_050` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate 相对 stable 改善 `45.3%`，0.5 s lateral P95=`0.0663 m`
- `multistep_fopdt` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate 相对 stable 改善 `61.3%`，0.5 s lateral P95=`0.0622 m`
- `stable_nominal` on `actuator_run01`：0.2 s yaw-rate 相对 stable 改善 `0.0%`，0.5 s lateral P95=`0.0368 m`
- `historical_candidate` on `actuator_run01`：0.2 s yaw-rate 相对 stable 改善 `48.1%`，0.5 s lateral P95=`0.0352 m`
- `joint_fopdt` on `actuator_run01`：0.2 s yaw-rate 相对 stable 改善 `42.3%`，0.5 s lateral P95=`0.0363 m`
- `two_stage_050` on `actuator_run01`：0.2 s yaw-rate 相对 stable 改善 `28.4%`，0.5 s lateral P95=`0.0362 m`
- `multistep_fopdt` on `actuator_run01`：0.2 s yaw-rate 相对 stable 改善 `45.9%`，0.5 s lateral P95=`0.0358 m`
- `stable_nominal` on `actuator_run02`：0.2 s yaw-rate 相对 stable 改善 `0.0%`，0.5 s lateral P95=`0.0387 m`
- `historical_candidate` on `actuator_run02`：0.2 s yaw-rate 相对 stable 改善 `55.9%`，0.5 s lateral P95=`0.0360 m`
- `joint_fopdt` on `actuator_run02`：0.2 s yaw-rate 相对 stable 改善 `55.5%`，0.5 s lateral P95=`0.0368 m`
- `two_stage_050` on `actuator_run02`：0.2 s yaw-rate 相对 stable 改善 `44.2%`，0.5 s lateral P95=`0.0368 m`
- `multistep_fopdt` on `actuator_run02`：0.2 s yaw-rate 相对 stable 改善 `55.2%`，0.5 s lateral P95=`0.0366 m`
- `stable_nominal` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate 相对 stable 改善 `0.0%`，0.5 s lateral P95=`0.0525 m`
- `historical_candidate` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate 相对 stable 改善 `19.4%`，0.5 s lateral P95=`0.0453 m`
- `joint_fopdt` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate 相对 stable 改善 `23.1%`，0.5 s lateral P95=`0.0442 m`
- `two_stage_050` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate 相对 stable 改善 `24.6%`，0.5 s lateral P95=`0.0438 m`
- `multistep_fopdt` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate 相对 stable 改善 `17.0%`，0.5 s lateral P95=`0.0461 m`
- `stable_nominal` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate 相对 stable 改善 `0.0%`，0.5 s lateral P95=`0.0587 m`
- `historical_candidate` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate 相对 stable 改善 `17.1%`，0.5 s lateral P95=`0.0495 m`
- `joint_fopdt` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate 相对 stable 改善 `24.3%`，0.5 s lateral P95=`0.0517 m`
- `two_stage_050` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate 相对 stable 改善 `27.0%`，0.5 s lateral P95=`0.0522 m`
- `multistep_fopdt` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate 相对 stable 改善 `17.7%`，0.5 s lateral P95=`0.0513 m`
- multistep 所有 holdout 门槛通过：`False`；高速度闭环 bag 未达到 yaw-rate 改善 ≥30% 门槛。

## 无转角传感器观测器影子回放

- `stable_nominal` on `track_prbs_150_20260816_011147`：delta_hat RMSE=`0.02479 rad`，相对 stable 改善=`0.0%`
- `historical_candidate` on `track_prbs_150_20260816_011147`：delta_hat RMSE=`0.01655 rad`，相对 stable 改善=`33.2%`
- `multistep_fopdt` on `track_prbs_150_20260816_011147`：delta_hat RMSE=`0.01616 rad`，相对 stable 改善=`34.8%`
- `stable_nominal` on `actuator_run01`：delta_hat RMSE=`0.03445 rad`，相对 stable 改善=`0.0%`
- `historical_candidate` on `actuator_run01`：delta_hat RMSE=`0.01938 rad`，相对 stable 改善=`43.7%`
- `multistep_fopdt` on `actuator_run01`：delta_hat RMSE=`0.01994 rad`，相对 stable 改善=`42.1%`
- `stable_nominal` on `actuator_run02`：delta_hat RMSE=`0.03938 rad`，相对 stable 改善=`0.0%`
- `historical_candidate` on `actuator_run02`：delta_hat RMSE=`0.01830 rad`，相对 stable 改善=`53.5%`
- `multistep_fopdt` on `actuator_run02`：delta_hat RMSE=`0.01871 rad`，相对 stable 改善=`52.5%`
- `stable_nominal` on `lateral_mpcc_20260814_185646_0`：delta_hat RMSE=`0.01930 rad`，相对 stable 改善=`0.0%`
- `historical_candidate` on `lateral_mpcc_20260814_185646_0`：delta_hat RMSE=`0.01654 rad`，相对 stable 改善=`14.3%`
- `multistep_fopdt` on `lateral_mpcc_20260814_185646_0`：delta_hat RMSE=`0.01705 rad`，相对 stable 改善=`11.7%`
- `stable_nominal` on `mpcc_40hz_td0_20260814_193817`：delta_hat RMSE=`0.02163 rad`，相对 stable 改善=`0.0%`
- `historical_candidate` on `mpcc_40hz_td0_20260814_193817`：delta_hat RMSE=`0.02034 rad`，相对 stable 改善=`6.0%`
- `multistep_fopdt` on `mpcc_40hz_td0_20260814_193817`：delta_hat RMSE=`0.01997 rad`，相对 stable 改善=`7.7%`
- 只按 primary 选择的增益网格结果：kinematic=`0.15`，dynamic=`0.00`；其 holdout RMSE=`0.02180 rad`。
- 当前修正对新候选只带来约 1% 量级变化；1.5 m/s completed bag 上动态修正还略微恶化。因此不能把运动学伪测量当作真实转角反馈。

## 整车动力学开环影子回放

- `stable_physics` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate RMSE=`0.1885 rad/s`，0.5 s lateral P95=`0.0667 m`，1.0 s position P95=`0.1525 m`
- `stable_plus_v1_residual` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate RMSE=`0.1731 rad/s`，0.5 s lateral P95=`0.0635 m`，1.0 s position P95=`0.1164 m`
- `multistep_physics` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate RMSE=`0.0978 rad/s`，0.5 s lateral P95=`0.0521 m`，1.0 s position P95=`0.1128 m`
- `multistep_plus_old_v1_residual` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate RMSE=`0.0896 rad/s`，0.5 s lateral P95=`0.0523 m`，1.0 s position P95=`0.0891 m`
- `historical_physics` on `track_prbs_150_20260816_011147`：0.2 s yaw-rate RMSE=`0.0995 rad/s`，0.5 s lateral P95=`0.0467 m`，1.0 s position P95=`0.1074 m`
- `stable_physics` on `actuator_run01`：0.2 s yaw-rate RMSE=`0.0611 rad/s`，0.5 s lateral P95=`0.0186 m`，1.0 s position P95=`0.0524 m`
- `stable_plus_v1_residual` on `actuator_run01`：0.2 s yaw-rate RMSE=`0.0789 rad/s`，0.5 s lateral P95=`0.0204 m`，1.0 s position P95=`0.2541 m`
- `multistep_physics` on `actuator_run01`：0.2 s yaw-rate RMSE=`0.0411 rad/s`，0.5 s lateral P95=`0.0170 m`，1.0 s position P95=`0.0431 m`
- `multistep_plus_old_v1_residual` on `actuator_run01`：0.2 s yaw-rate RMSE=`0.0583 rad/s`，0.5 s lateral P95=`0.0205 m`，1.0 s position P95=`0.3069 m`
- `historical_physics` on `actuator_run01`：0.2 s yaw-rate RMSE=`0.0419 rad/s`，0.5 s lateral P95=`0.0165 m`，1.0 s position P95=`0.0426 m`
- `stable_physics` on `actuator_run02`：0.2 s yaw-rate RMSE=`0.0790 rad/s`，0.5 s lateral P95=`0.0205 m`，1.0 s position P95=`0.0550 m`
- `stable_plus_v1_residual` on `actuator_run02`：0.2 s yaw-rate RMSE=`0.0951 rad/s`，0.5 s lateral P95=`0.0242 m`，1.0 s position P95=`0.3197 m`
- `multistep_physics` on `actuator_run02`：0.2 s yaw-rate RMSE=`0.0468 rad/s`，0.5 s lateral P95=`0.0174 m`，1.0 s position P95=`0.0454 m`
- `multistep_plus_old_v1_residual` on `actuator_run02`：0.2 s yaw-rate RMSE=`0.0575 rad/s`，0.5 s lateral P95=`0.0249 m`，1.0 s position P95=`0.3431 m`
- `historical_physics` on `actuator_run02`：0.2 s yaw-rate RMSE=`0.0506 rad/s`，0.5 s lateral P95=`0.0165 m`，1.0 s position P95=`0.0464 m`
- `stable_physics` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate RMSE=`0.4000 rad/s`，0.5 s lateral P95=`0.1660 m`，1.0 s position P95=`0.8209 m`
- `stable_plus_v1_residual` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate RMSE=`0.2711 rad/s`，0.5 s lateral P95=`0.1187 m`，1.0 s position P95=`0.5947 m`
- `multistep_physics` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate RMSE=`0.4351 rad/s`，0.5 s lateral P95=`0.1676 m`，1.0 s position P95=`0.8844 m`
- `multistep_plus_old_v1_residual` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate RMSE=`0.3197 rad/s`，0.5 s lateral P95=`0.1287 m`，1.0 s position P95=`0.6715 m`
- `historical_physics` on `lateral_mpcc_20260814_185646_0`：0.2 s yaw-rate RMSE=`0.4357 rad/s`，0.5 s lateral P95=`0.1683 m`，1.0 s position P95=`0.9109 m`
- `stable_physics` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate RMSE=`0.4076 rad/s`，0.5 s lateral P95=`0.1688 m`，1.0 s position P95=`0.8056 m`
- `stable_plus_v1_residual` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate RMSE=`0.2702 rad/s`，0.5 s lateral P95=`0.1211 m`，1.0 s position P95=`0.5829 m`
- `multistep_physics` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate RMSE=`0.4363 rad/s`，0.5 s lateral P95=`0.1789 m`，1.0 s position P95=`0.9596 m`
- `multistep_plus_old_v1_residual` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate RMSE=`0.3284 rad/s`，0.5 s lateral P95=`0.1434 m`，1.0 s position P95=`0.8164 m`
- `historical_physics` on `mpcc_40hz_td0_20260814_193817`：0.2 s yaw-rate RMSE=`0.4380 rad/s`，0.5 s lateral P95=`0.1789 m`，1.0 s position P95=`0.9814 m`
- 3 个专用转向 holdout 上，新舵机模型单独使用时 yaw-rate 改善约 33–48%，lateral 改善约 9–22%。
- 两个约 2–3.15 m/s 的历史闭环 holdout 上，新舵机模型单独使用时 yaw-rate 反而恶化约 7–9%，lateral 恶化约 1–6%；当前候选没有高速泛化证据。
- 旧 V1 残差能缓解高速度影子误差，但在多份 0.6 m/s 数据上把 1 s position P95 放大到约 0.25–0.34 m；说明其速度域兼容性不成立，不能直接与新舵机模型绑定部署。

## 当前 stable C++/acados 闭环基线

- `legacy`：solves=1200，failures=15，status4=15，laps=2.676，minimum margin=0.142 m，solve P95/max=1.484/6.622 ms
- `simplified`：solves=1200，failures=880，status4=25，laps=0.951，minimum margin=-1.127 m，solve P95/max=6.824/8.256 ms
- `cap_guard`：solves=1200，failures=0，status4=0，laps=2.716，minimum margin=0.100 m，solve P95/max=1.482/6.672 ms
- `full`：solves=1200，failures=0，status4=0，laps=3.399，minimum margin=0.103 m，solve P95/max=1.518/10.288 ms
- 该矩阵只验证当前 stable OCP；新候选约 95 ms 的纯延迟尚未写入 OCP，因此 `candidate_delay_model_tested=false`。

## 轮胎侧偏刚度可辨识性

- 通过力平衡基本门槛的 bag：`0`。
- `track_static_060_20260816_005542`：not enough informative tire-slip samples
- `track_prbs_060_20260816_005954`：not enough informative tire-slip samples
- `track_prbs_100_20260816_010820`：stiffness reached/approached the lower bound or axle-force R2 is non-positive；Cf=5.00, Cr=5.39 N/rad，R2f/R2r=-0.108/-0.239
- `track_prbs_150_20260816_011504`：stiffness reached/approached the lower bound or axle-force R2 is non-positive；Cf=12.71, Cr=5.00 N/rad，R2f/R2r=-0.022/-0.171
- `track_prbs_150_20260816_011147`：stiffness reached/approached the lower bound or axle-force R2 is non-positive；Cf=14.85, Cr=20.27 N/rad，R2f/R2r=-0.138/-0.283
- `actuator_run01`：not enough informative tire-slip samples
- `actuator_run02`：not enough informative tire-slip samples
- `lateral_mpcc_20260814_185646_0`：stiffness reached/approached the lower bound or axle-force R2 is non-positive；Cf=21.73, Cr=32.90 N/rad，R2f/R2r=0.114/-0.065
- `mpcc_40hz_td0_20260814_193817`：stiffness reached/approached the lower bound or axle-force R2 is non-positive；Cf=20.64, Cr=36.97 N/rad，R2f/R2r=-0.066/-0.276
- 结论：当前转向辨识 bag 不能独立辨识前后轴侧偏刚度；Cf/Cr 保持不变，后续需采集专用的更高速横向激励，并改善横向状态可观测性。

## 最终判定与部署门槛

- 当前本地证据仅支持把 `K=1.11323, b=-0.01117 rad, Td=0.095 s, tau=0.06666 s` 保留为 0.6–1.5 m/s 的瞬态候选；高速度门槛失败，禁止直接部署到 MPCC。
- 当前证据不支持修改 Cf/Cr，也不支持宣称已辨识 1.5、3.4 或 5.5 rad/s 的机械硬转角速率。
- 不自动部署候选：除 OCP 需因果表示约 95 ms 纯延迟外，还必须先解决高速横向动力学/Cf-Cr 不可辨识和旧残差低速失配。
- 下一轮至少需要：当前配置下独立左右静态平台、2–3 m/s 安全 PRBS、全新未见高速 final-test bag、candidate solver 闭环仿真回归，以及 stable 一键回退。

## 复现实验

```bash
cd /home/ros/f1tenth_residual_controller_ws
./scripts/run_steering_offline_experiments.sh
```
