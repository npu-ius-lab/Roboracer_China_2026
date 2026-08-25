# 不可变基线清单

新项目与旧 `planning` 工作空间相互独立。以下文件于 2026-08-12 从车端当前部署
只读复制，未重新生成：

| 文件 | SHA-256 |
|---|---|
| `virtual_track/raceline.csv` | `f90c6de4e38e00764293ba58f3eadfa34d19f9d5c267180de42e6e18a25a2478` |
| `virtual_track_u_turn/raceline.csv` | `8c5ec2b0e815341189a87b9e3dbf9a5900e8c68d626c61adce75f5ff1a46dbae` |
| `virtual_track_s_u/raceline.csv` | `e7231e9af57a740acfb0b03e92aa1ba65c970d6be660caa5a199e91aa249deb3` |

车辆配置采用车端当前版本：轴距 0.320 m、车体宽度 0.24 m、质量 3.74 kg、
名义横摆惯量 0.04712 kg·m²、`lf=0.153846154 m`、`lr=0.166153846 m`。
这些值在本次重新实现中不修改。
