# TianRacer 转向执行器联合辨识

生成时间：2026-08-16T19:34:20

该结果是无舵角传感器条件下的等效命令到前轮角模型，不会自动修改实车配置。

## 联合候选

- 增益 K：`0.974299`
- 偏置 b：`-0.011499 rad`
- 纯延迟 Td：`0.105000 s`
- 时间常数 tau：`0.062672 s`
- 动态 RMSE：`0.016627 rad`
- 动态 R2：`0.98508`

## 每个完整 bag

- `circle_v0.8banjin2.0`: RMSE=0.01723 rad, R2=0.6711, optimistic-P95=0.01931 rad
- `circle_v1.2banjin2.0`: RMSE=0.01447 rad, R2=0.7541, optimistic-P95=0.01790 rad
- `circle_v1.6banjin2.0`: RMSE=0.02086 rad, R2=0.6280, optimistic-P95=0.02787 rad
- `circle_r2p5_2p0`: RMSE=0.01055 rad, R2=0.8136, optimistic-P95=0.02016 rad
- `circle_v0.8right`: RMSE=0.01996 rad, R2=0.3740, optimistic-P95=0.01978 rad
- `circle_v1.2right`: RMSE=0.01623 rad, R2=0.6175, optimistic-P95=0.02301 rad
- `circle_v1.6right`: RMSE=0.01770 rad, R2=0.5836, optimistic-P95=0.03220 rad
- `circle_v2.0right`: RMSE=0.02039 rad, R2=0.3680, optimistic-P95=0.03782 rad

## 留一验证

- 留出 `circle_v0.8banjin2.0`: RMSE=0.01795 rad, R2=0.6430
- 留出 `circle_v1.2banjin2.0`: RMSE=0.01502 rad, R2=0.7352
- 留出 `circle_v1.6banjin2.0`: RMSE=0.02117 rad, R2=0.6171
- 留出 `circle_r2p5_2p0`: RMSE=0.01155 rad, R2=0.7764
- 留出 `circle_v0.8right`: RMSE=0.02211 rad, R2=0.2321
- 留出 `circle_v1.2right`: RMSE=0.01635 rad, R2=0.6120
- 留出 `circle_v1.6right`: RMSE=0.01852 rad, R2=0.5438
- 留出 `circle_v2.0right`: RMSE=0.02206 rad, R2=0.2607

## 状态

`candidate_needs_new_independent_bags`

需要新的多速度 PRBS bag 和完全未激励 stable 圈作为最终独立验收。
