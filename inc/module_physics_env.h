// ==== module_physics_env.h ====
#pragma once
#include "get_car_param.h"

class VehicleState {
public:
    double x, y, phi;         // 全局坐标与航向角
    double vx, vy, phi_dot;   // 纵向速度，横向速度，横摆角速度
    double delta;             // 前轮偏角

    VehicleState() : x(0), y(0), phi(0), vx(10), vy(0), phi_dot(0), delta(0) {}
};

VehicleState module_physics_env(const CarParam& car, const VehicleState& state, double a_x_cmd, double delta_cmd);