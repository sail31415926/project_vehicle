// ==== module_mpc_control.cpp ====
#include "module_mpc_control.h"

#include <iostream>

double MpcSolver::solve(const Eigen::Vector4d& xi_k, double vx,
                        double kappa_ref, double current_delta,
                        double& current_ax) {
    int n = 4;  // 状态：[ed, ed_dot, e_phi, e_phi_dot], 这里的导数是相对于 s (弧长)的导数
    int m = 1;  // 控制：[delta]
    int Np = car.Np;
    int Nc = car.Nc;

    // 1. 连续时间动力学矩阵 A_c, B_c, C_c (负刚度体系修正)
    double a22 = (car.Cf + car.Cr) / (car.m * vx * vx);
    double a23 = -(car.Cf + car.Cr) / (car.m * vx);
    double a24 = (car.lf * car.Cf - car.lr * car.Cr) / (car.m * vx * vx);
    double a42 = (car.lf * car.Cf - car.lr * car.Cr) / (car.Iz * vx * vx);
    double a43 = -(car.lf * car.Cf - car.lr * car.Cr) / (car.Iz * vx);
    double a44 =
        (car.lf * car.lf * car.Cf + car.lr * car.lr * car.Cr) / (car.Iz * vx * vx);

    Eigen::Matrix4d A_c = Eigen::Matrix4d::Zero();
    A_c(0, 1) = 1.0 / vx;
    A_c(1, 1) = a22;
    A_c(1, 2) = a23;
    A_c(1, 3) = a24;
    A_c(2, 3) = 1.0 / vx;
    A_c(3, 1) = a42;
    A_c(3, 2) = a43;
    A_c(3, 3) = a44;

    Eigen::Vector4d B_c = Eigen::Vector4d::Zero();
    B_c(1) = -car.Cf / (car.m * vx);
    B_c(3) = -car.lf * car.Cf / (car.Iz * vx);

    Eigen::Vector4d C_c = Eigen::Vector4d::Zero();
    C_c(1) = (car.lf * car.Cf - car.lr * car.Cr) / (car.m * vx) - vx;
    C_c(3) = (car.lf * car.lf * car.Cf + car.lr * car.lr * car.Cr) / (car.Iz * vx);

    // 2. 离散化 (欧拉近似法)
    // ==============================================================
    // 离散化：使用前向欧拉法，步长为空间步长 ds
    // =============================================================
    Eigen::Matrix4d A_k = Eigen::Matrix4d::Identity() + A_c * car.ds;
    Eigen::Vector4d B_k = B_c * car.ds;
    Eigen::Vector4d C_k = C_c * kappa_ref * car.ds;

    // 3. 构建预测堆叠矩阵(Phi*xi + Theta*U + W*kappa_ref)
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(n * Np, n);
    Eigen::MatrixXd Theta = Eigen::MatrixXd::Zero(n * Np, m * Nc);
    Eigen::VectorXd W = Eigen::VectorXd::Zero(n * Np);

    Eigen::Matrix4d A_pow = A_k;
    for (int i = 0; i < Np; ++i) {
        Phi.block(i * n, 0, n, n) = A_pow;

        Eigen::Vector4d w_temp = Eigen::Vector4d::Zero();
        Eigen::Matrix4d A_j = Eigen::Matrix4d::Identity();  // A^j
        for (int j = 0; j <= i; ++j) {
            w_temp += A_j * C_k;
            A_j *= A_k;
        }
        W.segment(i * n, n) = w_temp;

        for (int j = 0; j < Nc; ++j) {
            if (i >= j) {
                if (j < Nc - 1) {
                    Eigen::Matrix4d A_ij = Eigen::Matrix4d::Identity();
                    for (int k = 0; k < i - j; k++) A_ij *= A_k;
                    Theta.block(i * n, j * m, n, m) = A_ij * B_k;
                } else {
                    Eigen::Vector4d theta_sum = Eigen::Vector4d::Zero();
                    Eigen::Matrix4d A_ij = Eigen::Matrix4d::Identity();
                    for (int k = 0; k <= i - Nc; ++k) {
                        theta_sum += A_ij * B_k;
                        A_ij *= A_k;
                    }
                    Theta.block(i * n, Nc - 1, n, m) = theta_sum;
                }
            }
        }
        A_pow *= A_k;
    }

    // 4. 代价函数权重
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << 100, 1, 10, 1;
    double R = 10.0;

    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(n * Np, n * Np);
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(m * Nc, m * Nc);
    for (int i = 0; i < Np; ++i) Q_bar.block(i * n, i * n, n, n) = Q;
    for (int i = 0; i < Nc; ++i) R_bar(i, i) = R;

    // H = 2 * (Theta^T * Q * Theta + R)
    Eigen::MatrixXd H = 2 * (Theta.transpose() * Q_bar * Theta + R_bar);
    H = (H + H.transpose()) / 2.0;  // 保证严格对称

    // f = 2 * Theta^T * Q * (Phi * xi + W)
    Eigen::VectorXd f = 2 * Theta.transpose() * Q_bar * (Phi * xi_k + W);

    // 5. 约束构建 (包含附着力软约束转换)
    int num_vars = Nc;             // 决策变量数量：转角序列 U_k
    int num_cons = Nc + (Nc - 1);  // 约束数量：转角约束 + 转角速率约束
    Eigen::MatrixXd A_cons = Eigen::MatrixXd::Zero(num_cons, num_vars);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(num_cons);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(num_cons);

    // --- 车辆附着条件约束近似化 ---
    // 1. 计算理论上的最大加速度平方
    double max_acc_sq = (car.mu * car.g) * (car.mu * car.g);
    double ax_sq = current_ax * current_ax;
    
    // 2. 计算留给横向的可用加速度 (防止浮点误差导致根号内为负)
    double available_ay_sq = std::max(0.0, max_acc_sq - ax_sq);
    double available_ay = std::sqrt(available_ay_sq);
    
    // 3. 计算动态转角极限
    double dynamic_delta_max = (available_ay * (car.lf + car.lr)) / (vx * vx + 1e-6);
    double actual_delta_max = std::min(car.delta_max, dynamic_delta_max);

    // 【新增】：将时间域角速度限制(rad/s)转换为空间域每一步的角度增量限制
    // 计算车辆走过 1 个 ds 需要多长时间 (dt_spatial = ds / vx)
    double spatial_delta_rate_max = car.delta_rate_max * (car.ds / (vx * car.dt));

    // --- [0, Nc-1] 行：转角极值约束 (U_k) ---
    A_cons.block(0, 0, Nc, Nc) = Eigen::MatrixXd::Identity(Nc, Nc);
    for (int i = 0; i < Nc; ++i) {
        lb(i) = -actual_delta_max;
        ub(i) = actual_delta_max;
        if (i == 0) {
            lb(i) = std::max(lb(i), current_delta - spatial_delta_rate_max);
            ub(i) = std::min(ub(i), current_delta + spatial_delta_rate_max);
        }
    }
    // --- [Nc, 2Nc-2] 行：转角速率约束 (U_k - U_{k-1}) ---
    for (int i = 0; i < Nc - 1; ++i) {
        A_cons(Nc + i, i + 1) = 1.0;
        A_cons(Nc + i, i) = -1.0;
        lb(Nc + i) = -spatial_delta_rate_max;
        ub(Nc + i) = spatial_delta_rate_max;
    }

    // 6. OSQP 求解过程
    std::vector<OSQPFloat> P_data, A_data;
    std::vector<OSQPInt> P_ri, P_cp, A_ri, A_cp;
    eigenToCscMatrix(H, P_data, P_ri, P_cp, true);
    eigenToCscMatrix(A_cons, A_data, A_ri, A_cp, false);

    // 构建 P 矩阵的 CSC 结构体
    OSQPCscMatrix P_csc;
    P_csc.m = H.rows();
    P_csc.n = H.cols();
    P_csc.nzmax = static_cast<OSQPInt>(P_data.size());
    P_csc.x = P_data.data();  // 数值
    P_csc.i = P_ri.data();    // 行索引
    P_csc.p = P_cp.data();    // 列指针
    P_csc.nz = -1;            // -1 表示标准 CSC 格式

    // 构建 A 矩阵的 CSC 结构体
    OSQPCscMatrix A_csc;
    A_csc.m = A_cons.rows();
    A_csc.n = A_cons.cols();
    A_csc.nzmax = static_cast<OSQPInt>(A_data.size());
    A_csc.x = A_data.data();
    A_csc.i = A_ri.data();
    A_csc.p = A_cp.data();
    A_csc.nz = -1;

    std::vector<OSQPFloat> q_vec(f.data(), f.data() + f.size());
    std::vector<OSQPFloat> l_vec(lb.data(), lb.data() + lb.size());
    std::vector<OSQPFloat> u_vec(ub.data(), ub.data() + ub.size());

    OSQPSolver* solver = nullptr;
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_starting = 1;

    double optimal_delta = current_delta;
    if (osqp_setup(&solver, &P_csc, q_vec.data(), &A_csc, l_vec.data(),
                   u_vec.data(), num_cons, num_vars, &settings) == 0) {
        osqp_solve(solver);
        if (solver->info->status_val == OSQP_SOLVED ||
            solver->info->status_val == OSQP_SOLVED_INACCURATE) {
            optimal_delta = solver->solution->x[0];
        }
        osqp_cleanup(solver);
    }
    return optimal_delta;
}

void MpcSolver::eigenToCscMatrix(const Eigen::MatrixXd& M,
                                 std::vector<OSQPFloat>& data,
                                 std::vector<OSQPInt>& row_indices,
                                 std::vector<OSQPInt>& col_ptrs,
                                 bool upper_triangular) {
    int rows = M.rows(), cols = M.cols();
    col_ptrs.assign(cols + 1, 0);
    data.clear();
    row_indices.clear();
    for (int j = 0; j < cols; ++j) {
        int col_nnz = 0;
        for (int i = 0; i < rows; ++i) {
            if (upper_triangular && i > j) continue;
            if (std::abs(M(i, j)) > 1e-9) {
                data.push_back(M(i, j));
                row_indices.push_back(i);
                col_nnz++;
            }
        }
        col_ptrs[j + 1] = col_ptrs[j] + col_nnz;
    }
}