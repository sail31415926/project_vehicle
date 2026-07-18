// ==== module_state_estimator.cpp ====
// 基于扩展卡尔曼滤波器 (EKF) 的车辆状态估计器
#include "module_state_estimator.h"

EKF_Estimator::EKF_Estimator(const CarParam& car) : car_(car) {
    X_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;

    // Q: 过程噪声 (表示我们对动力学模型的不信任程度)
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_.diagonal() << 0.1, 0.1, 0.05, 0.1, 0.1, 0.05;

    // R: 测量噪声 (表示我们对传感器的不信任程度)
    /* 这里将 $x$ 和 $y$ 的方差设为 0.5 ，意味着标准差 $\sigma = \sqrt{0.5}
     * \approx 0.7\text{米}$，这非常符合普通民用 GPS 的定位波动范围。将角度
     * $\phi$ 和角速度 $\dot{\phi}$ 的方差设为 0.1 ，是因为车辆搭载的
     * IMU（惯导）或者车载轮速计在测角度和速度时往往比 GPS 更加精准、稳定。 */
    R_ = Eigen::MatrixXd::Identity(5, 5);
    R_.diagonal() << 0.5, 0.5, 0.1, 0.5,
        0.1;  // [x, y, phi, vx, phi_dot] 的噪声方差

    // H: 观测矩阵，传感器能测到 [x, y, phi, vx, phi_dot]，测不到 vy (索引为4)
    H_ = Eigen::MatrixXd::Zero(5, 6);
    H_(0, 0) = 1.0;  // x
    H_(1, 1) = 1.0;  // y
    H_(2, 2) = 1.0;  // phi
    H_(3, 3) = 1.0;  // vx
    H_(4, 5) = 1.0;  // phi_dot

    I_ = Eigen::MatrixXd::Identity(6, 6);
}

Eigen::VectorXd EKF_Estimator::stateToVector(const VehicleState& state) {
    Eigen::VectorXd vec(6);
    vec << state.x, state.y, state.phi, state.vx, state.vy, state.phi_dot;
    return vec;
}

VehicleState EKF_Estimator::vectorToState(const Eigen::VectorXd& vec,
                                          double delta) {
    VehicleState state;
    state.x = vec(0);
    state.y = vec(1);
    state.phi = vec(2);
    state.vx = vec(3);
    state.vy = vec(4);
    state.phi_dot = vec(5);
    state.delta = delta;
    return state;
}

void EKF_Estimator::initialize(const VehicleState& init_state) {
    X_ = stateToVector(init_state);
}

Eigen::MatrixXd EKF_Estimator::computeNumericalJacobian(
    const VehicleState& state, double a_x, double delta) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
    double eps = 1e-5;
    // state是上一次滤波后的最优状态即上一次后验状态\hat{x}_{k-1}^{+}
    // 不注入扰动的情况下的下一状态,即f(x)_{k-1} | x={\hat{x}_{k-1}}^{+}
    VehicleState state_next_nom = module_physics_env(car_, state, a_x, delta);
    Eigen::VectorXd x_next_nom = stateToVector(state_next_nom);

    // 对 6 个状态逐一注入微小扰动求偏导
    for (int i = 0; i < 6; ++i) {
        Eigen::VectorXd x_plus = stateToVector(state);
        x_plus(i) += eps;
        VehicleState state_plus =
            vectorToState(x_plus, delta);  // 加入扰动后的状态

        // 预测在扰动下的下一状态,即 f(x)_{k-1} | x={\hat{x}_{k-1}}^{+} + eps}
        VehicleState state_next_plus =
            module_physics_env(car_, state_plus, a_x, delta);
        Eigen::VectorXd x_next_plus = stateToVector(state_next_plus);

        F.col(i) =
            (x_next_plus - x_next_nom) / eps;  // [f(x + eps) - f(x)] / eps
    }
    return F;
}

void EKF_Estimator::predict(double a_x, double delta) {
    // 1. 状态预测: 直接复用高精度非线性物理引擎 x_{k|k-1} = f(x_{k-1}, u_{k-1})
    /* current_state是上一次滤波后的最优状态即上一次后验状态\hat{x}_{k-1}^{+}，next_state_pred是预测的下一状态的前验状态\hat{x}_k^{-}
     */
    VehicleState current_state = vectorToState(X_, delta);

    // 下一时刻的前验状态预测 \hat{x}_k^{-} = f(\hat{x}_{k-1}^{+}, u_{k-1})
    VehicleState next_state_pred =
        module_physics_env(car_, current_state, a_x, delta);
    Eigen::VectorXd X_pred = stateToVector(next_state_pred);

    // 2. 计算雅可比矩阵 F_k
    Eigen::MatrixXd F = computeNumericalJacobian(current_state, a_x, delta);

    // 3. 协方差预测 P_{k|k-1} = F * P_{k-1} * F^T + Q
    P_ = F * P_ * F.transpose() + Q_;
    X_ = X_pred;  // k步的前验状态更新
}

void EKF_Estimator::update(const Eigen::VectorXd& z_meas) {
    // 1. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // 2. 计算残差 y = z - H * X
    Eigen::VectorXd y =
        z_meas - H_ * X_;  // z_meas是传感器测量值，H * X是预测的观测值
    // 处理航向角跳变问题 (保持在 -pi 到 pi)
    y(2) = std::remainder(y(2), 2.0 * M_PI);

    // 3. 状态更新 X = X + K * y
    X_ = X_ + K * y;

    // 4. 协方差更新 P = (I - K * H) * P
    P_ = (I_ - K * H_) * P_;
}

VehicleState EKF_Estimator::getEstimatedState(double current_delta) {
    return vectorToState(X_, current_delta);
}