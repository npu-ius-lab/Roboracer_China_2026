# TianRacer 转向执行器联合辨识

生成时间：2026-08-16T19:36:26

该结果是无舵角传感器条件下的等效命令到前轮角模型，不会自动修改实车配置。

## 联合候选

- 增益 K：`0.894649`
- 偏置 b：`-0.021495 rad`
- 纯延迟 Td：`0.105000 s`
- 时间常数 tau：`0.077907 s`
- 动态 RMSE：`0.018360 rad`
- 动态 R2：`0.54184`

## 每个完整 bag

- `circle_v0.8right`: RMSE=0.01946 rad, R2=0.4052, optimistic-P95=0.01857 rad
- `circle_v1.2right`: RMSE=0.01609 rad, R2=0.6245, optimistic-P95=0.02335 rad
- `circle_v1.6right`: RMSE=0.01754 rad, R2=0.5908, optimistic-P95=0.03180 rad
- `circle_v2.0right`: RMSE=0.02040 rad, R2=0.3677, optimistic-P95=0.03506 rad

## 留一验证

- 留出 `circle_v0.8right`: RMSE=0.02179 rad, R2=0.2540
- 留出 `circle_v1.2right`: RMSE=0.01623 rad, R2=0.6175
- 留出 `circle_v1.6right`: RMSE=0.01822 rad, R2=0.5587
- 留出 `circle_v2.0right`: RMSE=0.02232 rad, R2=0.2431

## 状态

`candidate_needs_new_independent_bags`

需要新的多速度 PRBS bag 和完全未激励 stable 圈作为最终独立验收。
