// ==== module_state_estimator.h ====
// 基于扩展卡尔曼滤波器 (EKF) 的车辆状态估计器
#pragma once
#include <Eigen/Dense>
#include "module_physics_env.h"
#include "get_car_param.h"

class EKF_Estimator {
private:
    CarParam car_;
    Eigen::VectorXd X_; // 状态向量 [x, y, phi, vx, vy, phi_dot]^T (6x1)
    Eigen::MatrixXd P_; // 状态协方差矩阵 (6x6)
    Eigen::MatrixXd Q_; // 过程噪声协方差 (6x6)
    Eigen::MatrixXd R_; // 测量噪声协方差 (5x5)
    Eigen::MatrixXd H_; // 观测矩阵 (5x6)
    Eigen::MatrixXd I_; // 单位阵

    // 辅助函数：状态体与 Eigen 向量互转
    Eigen::VectorXd stateToVector(const VehicleState& state);
    VehicleState vectorToState(const Eigen::VectorXd& vec, double delta);

    // 核心：基于微小扰动的数值雅可比计算
    Eigen::MatrixXd computeNumericalJacobian(const VehicleState& state, double a_x, double delta);

public:
    EKF_Estimator(const CarParam& car);

    // 初始化滤波器状态
    void initialize(const VehicleState& init_state);

    // EKF 预测步 (Prior)
    void predict(double a_x, double delta);

    // EKF 更新步 (Posterior)
    // 观测向量 z = [x, y, phi, vx, phi_dot]^T
    void update(const Eigen::VectorXd& z_meas);

    // 获取当前最优估计状态
    VehicleState getEstimatedState(double current_delta);
};