#pragma once
#include <cmath>

class CarParam {
   public:
    // 算法参数
    const double dt = 0.05;  // 控制周期 (20Hz)
    const int Np = 30;       // 预测时域
    const int Nc = 5;        // 控制时域
    const double ds = 0.5;   // 轨迹点间距 (m)

    // 车辆物理参数 (4-DOF 动力学模型)
    const double m = 1520.0;      // 车辆质量 kg
    const double Iz = 2625.0;     // 绕Z轴转动惯量 kg*m^2
    const double lf = 1.19;       // 质心到前轴距离 m
    const double lr = 1.37;       // 质心到后轴距离 m
    const double Cf = -155495.0;  // 前轮总侧偏刚度 (负刚度体系)
    const double Cr = -155495.0;  // 后轮总侧偏刚度 (负刚度体系)

    // 物理约束
    const double delta_max = 30.0 * M_PI / 180.0;       // 最大前轮偏角 (30度)
    const double delta_rate_max = 10.0 * M_PI / 180.0;  // 每周期最大转角增量
    const double mu = 0.8;                              // 路面附着系数
    const double g = 9.81;                              // 重力加速度
};

CarParam get_CarParam(void);