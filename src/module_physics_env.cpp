// ==== module_physics_env.cpp ====
#include "module_physics_env.h"

#include <algorithm>

// ==== module_physics_env.cpp ====
#include <algorithm>

#include "module_physics_env.h"

VehicleState module_physics_env(const CarParam& car, const VehicleState& state,
                                double a_x_cmd, double delta_cmd) {
    VehicleState next = state;

    // 1. 纵向物理更新 (假设底盘能完美跟踪加速度，增加纵向积分)
    next.vx = state.vx + a_x_cmd * car.dt;
    // 限制车辆不能倒车，且不超过物理极速
    next.vx = std::max(
        0.1, std::min(30.0, next.vx));  // 最低保留 0.1 m/s 防止矩阵分母除零奇异

    // 2. 横向执行器延时与限幅模拟
    double delta_error = delta_cmd - state.delta;
    double delta_step = std::max(-car.delta_rate_max,
                                 std::min(car.delta_rate_max, delta_error));
    next.delta = std::max(-car.delta_max,
                          std::min(car.delta_max, state.delta + delta_step));

    // 3. 侧偏角与侧向力计算 (使用最新的 next.vx 参与计算)
    double alpha_f = (state.vy + car.lf * state.phi_dot) / next.vx - next.delta;
    double alpha_r = (state.vy - car.lr * state.phi_dot) / next.vx;

    double Fyf = car.Cf * alpha_f;
    double Fyr = car.Cr * alpha_r;

    // 4. 动力学微分方程 (纵向速度是变参)
    double vy_dot = (Fyf + Fyr) / car.m - next.vx * state.phi_dot;  // 纵向
    double phi_ddot = (car.lf * Fyf - car.lr * Fyr) / car.Iz;       // 横摆

    // 欧拉积分更新本体状态
    next.vy = state.vy + vy_dot * car.dt;
    next.phi_dot = state.phi_dot + phi_ddot * car.dt;

    // 坐标系转换更新世界坐标
    next.x = state.x +
             (next.vx * std::cos(state.phi) - state.vy * std::sin(state.phi)) *
                 car.dt;
    next.y = state.y +
             (next.vx * std::sin(state.phi) + state.vy * std::cos(state.phi)) *
                 car.dt;
    next.phi = state.phi + state.phi_dot * car.dt;

    // 规范化航向角
    next.phi = std::atan2(std::sin(next.phi), std::cos(next.phi));

    return next;
}