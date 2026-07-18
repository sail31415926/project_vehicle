#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include "get_car_param.h"
#include "module_lon_control.h"
#include "module_mpc_control.h"
#include "module_physics_env.h"
#include "module_planning.h"
#include "module_state_estimator.h"

int main() {
    // 1. 初始化
    CarParam car = get_CarParam();
    auto ref_path = module_planning("bezier_reference.csv");
    if (ref_path.empty()) return -1;

    // mpc solver 初始化
    MpcSolver mpc(car);

    // 初始化纵向 PI 控制器: Kp=1.5, Ki=0.2, 最大加速 3.0 m/s^2, 最大减速 -5.0
    // m/s^2
    LonController lon_pid(1.5, 0.2, 3.0, -5.0);

    // 文件初始化代码
    std::ofstream outfile("mpc_tracking_result.csv");
    outfile
        << "Step,X_ref,Y_ref,X_act,Y_act,Lat_Error,Delta_f,Vx,X_est,Y_est\n";

    // 假设定位系统给出初始状态：略有误差
    VehicleState state_current;
    state_current.x = ref_path[0].X_ref;
    state_current.y = ref_path[0].Y_ref + 0.5;  // 初始给 0.5 米偏差
    state_current.phi = ref_path[0].phi_ref;
    // 假设初始车速为 0，让 PI 控制器自己把速度提上去！
    state_current.vx = 10.0;

    /* ---------------- BEGIN: EKF 初始化 --------------------- */
    // 【新增】：初始化随机数引擎，模拟传感器噪声
    std::default_random_engine generator;
    std::normal_distribution<double> noise_xy(0.0, 0.2);  // GPS 位置漂移 0.2 米
    std::normal_distribution<double> noise_phi(0.0, 0.05);  // 航向角测量噪声
    std::normal_distribution<double> noise_vel(0.0, 0.1);   // 速度测量噪声

    // 【新增】：初始化 EKF
    EKF_Estimator ekf(car);
    ekf.initialize(state_current);  // 用初始状态冷启动
    /* ---------------- END: EKF 初始化 --------------------- */

    /* ----------------- BEGIN系统延迟定义 ------------------ */
    // 【1】：定义系统延迟步数 (假设延迟 100ms，即 2 个控制周期)
    int delay_steps = 2;

    // 【2】：给物理底盘建一个“延迟执行队列” (模拟真车的通讯和机械延迟)
    // 队列里存的是 pair<a_x, delta>
    std::deque<std::pair<double, double>>
        chassis_delay_buffer;  // 真车底盘延迟队列
    for (int i = 0; i < delay_steps; ++i) {
        chassis_delay_buffer.push_back(
            {0.0, state_current.delta});  // 初始默认不加速，保持初始转角
    }

    // 【3】：给算法建一个“记忆队列” (记录我们发出去但还没生效的指令)
    std::deque<std::pair<double, double>> algo_memory_buffer =
        chassis_delay_buffer;
    /* --------------- END系统延迟定义 -------------- */

    // 2. 主仿真循环
    int sim_steps = ref_path.size();
    int closest_idx = 0;  // 重点：将最近点索引声明在循环外部，持续跟踪
    // 主仿真循环 (上限设为足够大的数，依靠到达终点自动 break)
    for (int step = 0; step < 10000; ++step) {
        // ==========================================
        // 【新增】：1. 上帝视角的真实车在跑 (执行上一时刻的指令)
        // ==========================================
        std::pair<double, double> exec_cmd = chassis_delay_buffer.front();
        chassis_delay_buffer.pop_front();
        state_current = module_physics_env(car, state_current, exec_cmd.first,
                                           exec_cmd.second);

        // ==========================================
        // 【新增】：2. 传感器采样并注入噪声 -> 得到测量值 z_meas
        // ==========================================
        Eigen::VectorXd z_meas(5);
        z_meas << state_current.x + noise_xy(generator),
            state_current.y + noise_xy(generator),
            state_current.phi + noise_phi(generator),
            state_current.vx + noise_vel(generator),
            state_current.phi_dot + noise_phi(generator);

        // ==========================================
        // 【新增】：3. EKF 滤波，洗出干净的状态 state_est
        // ==========================================
        // 预测步：算法知道底盘刚刚执行了 exec_cmd，所以拿它做预测
        ekf.predict(exec_cmd.first, exec_cmd.second);
        // 更新步：融合带噪声的传感器数据
        ekf.update(z_meas);

        // 提取出滤波后的最优估计状态
        VehicleState state_est = ekf.getEstimatedState(state_current.delta);

        // ==========================================
        // 4. 前向预测补偿 (Forward Prediction)
        // ==========================================
        VehicleState state_pred = state_est;

        // 计算 delay_steps 步后的状态，得到 state_pred
        for (const auto& cmd : algo_memory_buffer) {
            state_pred =
                module_physics_env(car, state_pred, cmd.first, cmd.second);
        }

        // ==========================================
        // 5. 寻找最近投影点 (注意：这里必须用推演后的 state_pred 找！)
        // ==========================================
        double min_dist = 1e9;
        int search_start = closest_idx;

        // 往后搜索100个点，不用全局搜索以免太慢了
        for (int i = search_start; i < std::min(search_start + 100, sim_steps);
             ++i) {
            // 使用 state_pred 寻找未来的投影点！
            double dist = std::hypot(state_pred.x - ref_path[i].X_ref,
                                     state_pred.y - ref_path[i].Y_ref);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 终点退出条件：如果剩下的参考点不够预测时域 Np，说明跑完了
        if (closest_idx >= sim_steps - car.Np - 1) {
            std::cout << "车辆已接近参考轨迹终点，正常结束仿真。" << std::endl;
            break;
        }

        // 提取参考状态
        double x_r = ref_path[closest_idx].X_ref;
        double y_r = ref_path[closest_idx].Y_ref;
        double phi_r = ref_path[closest_idx].phi_ref;
        double kappa_r = ref_path[closest_idx].kappa_ref;
        double v_ref =
            ref_path[closest_idx].v_ref;  // 工业规划器每个点都带有参考速度

        // 6. 构建增广误差状态 xi
        // 注意：所有的误差全都基于 state_pred 计算
        Eigen::Vector2d vec_t(std::cos(phi_r), std::sin(phi_r));
        Eigen::Vector2d vec_d(state_pred.x - x_r, state_pred.y - y_r);
        double ed = vec_d.y() * vec_t.x() - vec_d.x() * vec_t.y();

        double e_phi = state_pred.phi - phi_r;
        e_phi = std::remainder(e_phi, 2.0 * M_PI);

        // 使用 state_pred 的速度和状态
        double ed_dot = state_pred.vy + state_pred.vx * std::sin(e_phi);
        double e_phi_dot = state_pred.phi_dot - state_pred.vx * kappa_r;

        Eigen::Vector4d xi_k(ed, ed_dot, e_phi, e_phi_dot);  // 增广误差状态

        // 7. 计算100ms后最优控制策略 optimal_ax 和 optimal_delta 并压入栈中
        // --- C1. 纵向控制 (基于推演后的速度) ---
        double optimal_ax = lon_pid.compute(v_ref, state_pred.vx, car.dt);

        // --- C2. 横向控制 (基于推演后的速度) ---
        double optimal_delta = mpc.solve(xi_k, state_pred.vx, kappa_r,
                                         state_pred.delta, optimal_ax);

        // --- 将新鲜出炉的指令塞入算法记忆队列和底盘延迟队列 ---
        algo_memory_buffer.push_back({optimal_ax, optimal_delta});
        algo_memory_buffer.pop_front();  // 算法知道最旧的指令已经被执行了
        chassis_delay_buffer.push_back({optimal_ax, optimal_delta});

        // 7. 记录并输出
        outfile << step << "," << x_r << "," << y_r << "," << state_current.x
                << "," << state_current.y << "," << ed << ","
                << state_current.delta << "," << state_current.vx << ","
                << state_est.x << "," << state_est.y << "\n";

        if (step % 10 == 0) {
            std::cout << "Step: " << step << " | Lat Error: " << ed
                      << " m | Delta: " << optimal_delta * 180.0 / M_PI
                      << " deg\n";
        }
    }

    outfile.close();
    std::cout << "仿真结束，结果已写入 mpc_tracking_result.csv" << std::endl;
    return 0;
}