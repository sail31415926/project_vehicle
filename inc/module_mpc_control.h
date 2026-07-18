// ==== module_mpc_control.h ====
#pragma once
#include <Eigen/Dense>
#include "get_car_param.h"
#include "osqp.h"

class MpcSolver {
private:
    CarParam car;
    void eigenToCscMatrix(const Eigen::MatrixXd& M, std::vector<OSQPFloat>& data,
                          std::vector<OSQPInt>& row_indices, std::vector<OSQPInt>& col_ptrs, bool upper_triangular);
public:
    MpcSolver(const CarParam& car_param) : car(car_param) {}
    double solve(const Eigen::Vector4d& xi_k, double vx, double kappa_ref, double current_delta, double& current_ax);
};