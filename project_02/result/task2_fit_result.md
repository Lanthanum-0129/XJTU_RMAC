# 任务 2 拟合结果说明

## 1. 视频与检测

- 输入：resources/task_2.mp4
- 分辨率：960x720，帧率：60 FPS，读入帧数：1440
- 旋转中心：已知 (480, 360)；白色点检测值 (479.7688378, 359.7688378)，已用于角度计算
- 青色 HSV 阈值：H[80,100] S[100,255] V[100,255]
- 预处理后端：GPU (OpenCV CUDA)（运行参数 --accel auto）
- 有效样本数：1440，帧范围 [0, 1439]

## 2. 模型与方法

- 模型：omega(t) = b + A sin(Omega t + phi)，时间单位 s，角度单位 rad
- 拟合方式：角度拟合（theta(t) 为 omega 的积分，附加估计 theta0）
- 求解器：Ceres AutoDiff + DENSE_QR；终止状态：CONVERGENCE，解可用：是
- 初始代价 0.002655418668 → 最终代价 0.001375224145
- 初值获取：Omega 网格扫描（0.05–8.0，步长 0.01），每个 Omega 下以 Eigen colPivHouseholderQr 解线性最小二乘，取 SSE 最小者
- 约束：A>0、Omega>0 由参数下界保证；b>A 由重参数化 b = b_extra + A（b_extra>=1e-3）保证

## 3. 估计参数

| 参数 | 值 | 单位 |
|---|---|---|
| A | 0.549993236 | rad/s |
| b | 1.350015478 | rad/s |
| Omega | 1.649925281 | rad/s |
| phi（统一到 [-pi, pi)） | 0.701639157 | rad |
| theta0 | 0.3501396571 | rad |
| 速度变化周期 T=2pi/Omega | 3.808163546 | s |

## 4. 误差指标

- 主指标（角度）RMSE = 0.001382039629 rad，有效样本 1440，帧范围 [0, 1439]
- 参考指标：角度 RMSE = 0.001382039629 rad；角速度 RMSE = 0.06092334599 rad/s

## 5. 输出文件

- tracking_overlay.mp4：叠加中心、目标轮廓、中心连线、ID 与状态
- fit_comparison.png：观测与拟合曲线对比
- angular_velocity.png：估计角速度曲线
- residuals.png：残差曲线
