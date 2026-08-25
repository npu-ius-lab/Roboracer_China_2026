# Delay compensation replay validation

Bags: 4

| Mode | Position RMSE | Yaw RMSE | vx RMSE | vy RMSE | r RMSE |
|---|---:|---:|---:|---:|---:|
| RAW | 0.1535 | 0.0944 | 0.1314 | 0.0503 | 0.1056 |
| MODEL_ONLY | 0.0220 | 0.0843 | 0.0820 | 0.1160 | 0.8218 |
| MODEL_IMU | 0.0174 | 0.0195 | 0.0820 | 0.1753 | 0.0320 |
| MODEL_IMU_WHEEL | 0.0173 | 0.0195 | 0.0827 | 0.1753 | 0.0320 |
| REPROPAGATION | 0.0173 | 0.0195 | 0.0827 | 0.1753 | 0.0320 |

PointLIO age median/p95: 0.1101/0.1435 s
Position improvement vs RAW: 88.7%
REPROPAGATION vx RMSE <0.15 m/s: True
