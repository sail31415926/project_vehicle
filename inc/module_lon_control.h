// ==== module_lon_control.h ====
// 纵向 PI 控制器实现
#pragma once

class LonController {
private:
    double kp_;
    double ki_;
    double integral_error_;
    double max_integral_; // 积分限幅
    double max_acc_;      // 最大加速度
    double min_acc_;      // 最大减速度 (负数)

public:
    LonController(double kp, double ki, double max_acc, double min_acc) 
        : kp_(kp), ki_(ki), integral_error_(0.0), max_integral_(2.0), max_acc_(max_acc), min_acc_(min_acc) {}

    // 计算期望加速度 a_x
    double compute(double v_ref, double v_real, double dt);
    
    // 重置积分器 (例如在自动驾驶退出重进时调用)
    void reset() { integral_error_ = 0.0; }
};