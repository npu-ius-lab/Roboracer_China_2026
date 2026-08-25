# TianRacer 转向执行器联合辨识

生成时间：2026-08-16T19:35:39

该结果是无舵角传感器条件下的等效命令到前轮角模型，不会自动修改实车配置。

## 联合候选

- 增益 K：`1.094613`
- 偏置 b：`-0.029719 rad`
- 纯延迟 Td：`0.095000 s`
- 时间常数 tau：`0.065548 s`
- 动态 RMSE：`0.014959 rad`
- 动态 R2：`0.73673`

## 每个完整 bag

- `circle_v0.8banjin2.0`: RMSE=0.01662 rad, R2=0.6940, optimistic-P95=0.01798 rad
- `circle_v1.2banjin2.0`: RMSE=0.01439 rad, R2=0.7567, optimistic-P95=0.01896 rad
- `circle_v1.6banjin2.0`: RMSE=0.02027 rad, R2=0.6488, optimistic-P95=0.02955 rad
- `circle_r2p5_2p0`: RMSE=0.01075 rad, R2=0.8063, optimistic-P95=0.02090 rad

## 留一验证

- 留出 `circle_v0.8banjin2.0`: RMSE=0.01732 rad, R2=0.6677
- 留出 `circle_v1.2banjin2.0`: RMSE=0.01508 rad, R2=0.7328
- 留出 `circle_v1.6banjin2.0`: RMSE=0.02053 rad, R2=0.6397
- 留出 `circle_r2p5_2p0`: RMSE=0.01201 rad, R2=0.7584

## 状态

`candidate_needs_new_independent_bags`

需要新的多速度 PRBS bag 和完全未激励 stable 圈作为最终独立验收。
