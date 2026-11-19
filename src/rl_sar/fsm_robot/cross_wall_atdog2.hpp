#pragma once

#include "cross_wall.hpp"
#include <atomic>

class CrossWallStateAtdog2 {
public:
    CrossWallStateAtdog2(const std::string& urdf_file_path);
    void enter();
    RobotTarget update();
    std::tuple<Eigen::Vector3d, double> get_robot_mass_info(
        const Eigen::Vector3d& lf_joint_pos, const Eigen::Vector3d& rf_joint_pos,
        const Eigen::Vector3d& lb_joint_pos, const Eigen::Vector3d& rb_joint_pos);

    // 由服务通信写入：0 = 正常执行当前动作，1 = 暂停并保持当前输出
    std::atomic<int> action_pause{0};

    // 注入 RL 配置中的站立关节角（dof 顺序 0=RF, 1=LF, 2=RB, 3=LB，每腿 3 个关节）
    void setStandJointPos(const std::vector<float>& dof_pos);

    std::shared_ptr<Robot_t> robot;
    int cross_wall_stage{-1};
    std::chrono::steady_clock::time_point cross_wall_stage_time;

    Eigen::Vector3d wall_lf_foot_pos{0, 0, 0}, wall_rf_foot_pos{0, 0, 0}, wall_lb_foot_pos{0, 0, 0}, wall_rb_foot_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_pos{0, 0, 0}, rf_foot_exp_pos{0, 0, 0}, lb_foot_exp_pos{0, 0, 0}, rb_foot_exp_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_force{0, 0, 0}, rf_foot_exp_force{0, 0, 0}, lb_foot_exp_force{0, 0, 0}, rb_foot_exp_force{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_vel{0, 0, 0}, rf_foot_exp_vel{0, 0, 0}, lb_foot_exp_vel{0, 0, 0}, rb_foot_exp_vel{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_acc{0, 0, 0}, rf_foot_exp_acc{0, 0, 0}, lb_foot_exp_acc{0, 0, 0}, rb_foot_exp_acc{0, 0, 0};
    Eigen::Vector3d lf_forward_torque{0, 0, 0}, rf_forward_torque{0, 0, 0}, lb_forward_torque{0, 0, 0}, rb_forward_torque{0, 0, 0};

    Eigen::Vector3d lf_joint_exp_pos_{0, 0, 0}, rf_joint_exp_pos_{0, 0, 0}, lb_joint_exp_pos_{0, 0, 0}, rb_joint_exp_pos_{0, 0, 0};
    Eigen::Vector3d lf_joint_omega{0, 0, 0}, rf_joint_omega{0, 0, 0}, lb_joint_omega{0, 0, 0}, rb_joint_omega{0, 0, 0};
    Eigen::Vector3d lf_joint_torque{0, 0, 0}, rf_joint_torque{0, 0, 0}, lb_joint_torque{0, 0, 0}, rb_joint_torque{0, 0, 0};

    double lf_wheel_vel{0.0}, rf_wheel_vel{0.0}, lb_wheel_vel{0.0}, rb_wheel_vel{0.0};
    double lf_wheel_force{0.0}, rf_wheel_force{0.0}, lb_wheel_force{0.0}, rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.3;

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;

    double time_s{1.0};
    bool change_flag{true};
    bool allow_vel{true};

    float k_F{1.0f};

    Eigen::Vector2d mass_center_pos;
    double mass;
    bool RL_walk_flag{false};
    bool Cross_wall_over{false};

    Cross_Step lf_step, rf_step, lb_step, rb_step;
    DiagonalWalkController diagonal_walk;

    // stage 9 位控站立相关状态
    Eigen::Vector3d stand_joint_pos[4];        // 站立目标关节角 (RF, LF, RB, LB)
    Eigen::Vector3d stand_start_joint_pos[4];  // 进入 stage 9 瞬间的关节角
    bool stand_target_ready{false};            // 是否已通过 setStandJointPos 注入目标
    bool stand_initialized{false};
    double stand_duration{1.5};                // 位控站立过渡时间(s)

    // 暂停保持：缓存上一帧输出，并补偿暂停期间流逝的状态机时间
    bool pause_active_{false};
    bool has_last_target_{false};
    std::chrono::steady_clock::time_point pause_start_time_;
    RobotTarget last_target_;
};
