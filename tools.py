import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. 模拟上游规划团队：生成三次贝塞尔曲线并导出为 CSV
def generate_bezier_trajectory(filename="bezier_reference.csv"):
    print("正在生成贝塞尔参考轨迹...")
    # 把路径拉长到 500 米，密集度保持 0.5m 左右
    s = np.linspace(0, 500, 1001) 
    X_ref = s
    # 稍微把正弦曲线的周期拉长一点，避免极短距离内的恐怖曲率
    Y_ref = 10 * np.sin(0.05 * X_ref) + 2 * np.cos(0.02 * X_ref) - 2
    
    # 计算一阶和二阶导数用于求航向角和曲率
    dX = np.gradient(X_ref)
    dY = np.gradient(Y_ref)
    ddX = np.gradient(dX)
    ddY = np.gradient(dY)
    
    Phi_ref = np.arctan2(dY, dX)
    Kappa_ref = (dX * ddY - dY * ddX) / ((dX**2 + dY**2)**(1.5))
    
    # 假设参考速度为恒定 10 m/s
    V_ref = np.full_like(X_ref, 10.0)
    
    df = pd.DataFrame({
        'X_ref': X_ref, 'Y_ref': Y_ref, 
        'Phi_ref': Phi_ref, 'Kappa_ref': Kappa_ref, 'V_ref': V_ref
    })
    df.to_csv(filename, index=False)
    print(f"轨迹已保存至 {filename}")

# 2. 模拟下游数据分析：读取 C++ 跑出来的 CSV 并画图
def plot_mpc_tracking(filename="mpc_tracking_result.csv"):
    if not os.path.exists(filename):
        print(f"找不到文件 {filename}，请先运行 C++ 程序！")
        return
        
    df = pd.read_csv(filename)
    
    plt.rcParams['font.sans-serif'] = ['SimHei'] # 支持中文
    plt.rcParams['axes.unicode_minus'] = False
    
    fig = plt.figure(figsize=(14, 8))
    
    # 子图1：轨迹对比
    ax1 = fig.add_subplot(2, 2, (1, 2))
    ax1.plot(df['X_ref'], df['Y_ref'], 'k--', label='参考轨迹 (贝塞尔)')
    ax1.plot(df['X_act'], df['Y_act'], 'b-', linewidth=2, label='实际轨迹 (动力学)')
    ax1.set_title('无人车动力学 LTV-MPC 轨迹跟踪')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # 子图2：横向误差
    ax2 = fig.add_subplot(2, 2, 3)
    ax2.plot(df['Step'], df['Lat_Error'], 'r-')
    ax2.set_title('横向误差 (Lateral Error)')
    ax2.set_xlabel('控制步数')
    ax2.set_ylabel('误差 [m]')
    ax2.grid(True)
    
    # 子图3：控制指令(前轮转角)
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.plot(df['Step'], np.degrees(df['Delta_f']), 'g-')
    ax3.set_title('前轮转角控制指令')
    ax3.set_xlabel('控制步数')
    ax3.set_ylabel('转角 [deg]')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 执行流程：1. 解开注释生成轨迹 -> 2. 跑 C++ -> 3. 解开注释画图
    generate_bezier_trajectory() 
    # plot_mpc_tracking()