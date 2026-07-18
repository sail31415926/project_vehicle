import pandas as pd
import matplotlib.pyplot as plt

# 读取 C++ 生成的数据
df = pd.read_csv("mpc_tracking_result.csv")

plt.figure(figsize=(12, 5))

# 绘制轨迹对比
plt.subplot(1, 2, 1)
plt.plot(df['X_ref'], df['Y_ref'], 'r--', label='Reference (Bezier)')
plt.plot(df['X_act'], df['Y_act'], 'b-', linewidth=2, label='Actual Vehicle Path (True)')

# 【新增】：绘制 EKF 估计出的轨迹，使用绿色点划线
# 注意：如果 EKF 效果极好，这条绿线会和上面的蓝线完全重合！
if 'X_est' in df.columns and 'Y_est' in df.columns:
    plt.plot(df['X_est'], df['Y_est'], 'g-.', linewidth=1.5, label='EKF Estimated Path')

plt.title("Vehicle Trajectory Tracking")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.grid(True)
plt.axis('equal')

# 绘制横向误差收敛曲线
plt.subplot(1, 2, 2)
plt.plot(df['Step'], df['Lat_Error'], 'g-')
plt.title("Cross-Track Error")
plt.xlabel("Simulation Step")
plt.ylabel("Lateral Error [m]")
plt.grid(True)

plt.tight_layout()
plt.show()