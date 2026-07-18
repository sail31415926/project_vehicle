// ==== module_lon_control.cpp ====
// 纵向 PI 控制器实现
#include "module_lon_control.h"
#include <algorithm>

double LonController::compute(double v_ref, double v_real, double dt) {
    // 1. 计算速度误差
    double error = v_ref - v_real;

    // 2. 积分项累加与限幅
    integral_error_ += error * dt;
    integral_error_ = std::max(-max_integral_, std::min(max_integral_, integral_error_));

    // 3. PI 控制律计算加速度指令
    double a_x = kp_ * error + ki_ * integral_error_;

    // 4. 输出物理限幅
    a_x = std::max(min_acc_, std::min(max_acc_, a_x));

    return a_x;
}