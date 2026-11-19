#include "cross_wall_atdog2.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

using Vector3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;

static bool use_limit_lf = false;
static bool use_limit_rf = false;
static bool use_limit_lb = false;
static bool use_limit_rb = false;


std::tuple<Vector3D, double> CrossWallStateAtdog2::get_robot_mass_info(
    const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos) {
    // 使用 KDL 计算全身质心（在 body_link 坐标系下）
    // 注意：这里用各 link 的 RigidBodyInertia 来做质量加权平均。
    //      需要 URDF 中每个 link 都有 inertial 标签，否则质量会为 0。

    auto accumulate_chain_com = [](const KDL::Chain& chain, const Vector3D& q_eigen, double& mass_sum, KDL::Vector& com_sum) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints() && i < 3; ++i) {
            q(i) = q_eigen[i];
        }

        KDL::Frame T = KDL::Frame::Identity();

        unsigned int joint_idx = 0;
        for (unsigned int seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
            const auto& seg = chain.getSegment(seg_idx);

            // 计算该段末端在基座下的位姿
            if (seg.getJoint().getType() != KDL::Joint::None) {
                T = T * seg.pose(q(joint_idx));
                joint_idx++;
            } else {
                T = T * seg.pose(0.0);
            }

            // 该 segment 的刚体惯量（在 segment 坐标系下）
            const KDL::RigidBodyInertia& rbi = seg.getInertia();
            const double m                   = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            // COM 在 segment 坐标系下的位置
            const KDL::Vector c_seg = rbi.getCOG();
            // 转到基座坐标系
            const KDL::Vector c_base = T * c_seg;

            mass_sum += m;
            com_sum = com_sum + c_base * m;
        }
    };

    double mass_sum = 0.0;
    KDL::Vector com_sum(0.0, 0.0, 0.0);

    // 4 条腿分别从 body_link 到足端 link4；它们共享 body_link，但各自 inertia 不重复（body_link 本体要单独加）
    accumulate_chain_com(robot->lf_leg_chain, lf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rf_leg_chain, rf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->lb_leg_chain, lb_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rb_leg_chain, rb_joint_pos, mass_sum, com_sum);

    // 加上 body_link 自身的质量与质心（树中 body_link 是 root，不在各腿 chain 的 segment 中）
    {
        const auto it = robot->tree.getSegment("body_link");
        if (it != robot->tree.getSegments().end()) {
            const auto body_seg = it->second.segment;
            const auto& rbi     = body_seg.getInertia();
            const double m      = rbi.getMass();
            if (m > 0.0) {
                mass_sum += m;
                com_sum = com_sum + rbi.getCOG() * m; // body_link 在 body_link 坐标系下
            }
        }
    }

    if (mass_sum <= 1e-9) {
        return std::make_tuple(Vector3D(0.0, 0.0, 0.0), 0.0);
    }

    const KDL::Vector com = com_sum / mass_sum;
    return std::make_tuple(Vector3D(com.x(), com.y(), com.z()), mass_sum);
}


CrossWallStateAtdog2::CrossWallStateAtdog2(const std::string& urdf_file_path)
{
        robot = std::make_shared<Robot_t>(urdf_file_path);
        diagonal_walk.configure(0.6, 0.08, 0.6, 0.3, 0.0, 0.0, 2.0, -0.04);
    
        Vector3D com_3d;
        std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
        mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());
        
}

void CrossWallStateAtdog2::setStandJointPos(const std::vector<float>& dof_pos) {
    if (dof_pos.size() < 12) {
        return;
    }
    for (int leg = 0; leg < 4; ++leg) {
        stand_joint_pos[leg] << dof_pos[leg * 3 + 0], dof_pos[leg * 3 + 1], dof_pos[leg * 3 + 2];
    }
    stand_target_ready = true;
}

void CrossWallStateAtdog2::enter() {
    cross_wall_stage = -1;
    change_flag = true;
    allow_vel = true;
    RL_walk_flag = false;
    Cross_wall_over = false;
    robot->lf_leg_calc->set_init_joint_pos(robot->lf_joint_pos);
    robot->rf_leg_calc->set_init_joint_pos(robot->rf_joint_pos);
    robot->lb_leg_calc->set_init_joint_pos(robot->lb_joint_pos);
    robot->rb_leg_calc->set_init_joint_pos(robot->rb_joint_pos);

    robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
    robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
    robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
    robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
    // cross_wall_stage = -1;
    cross_wall_stage_time = std::chrono::steady_clock::now();
    stopping = false;
    stop_t = 0.0;
    diagonal_walk.reset();

    stand_initialized = false;
    pause_active_ = false;
    has_last_target_ = false;
}


RobotTarget CrossWallStateAtdog2::update() {

    // 服务暂停：保持上一帧指令输出，并冻结状态机时间，恢复后轨迹不跳变
    if (action_pause.load() != 0 && has_last_target_) {
        if (!pause_active_) {
            pause_start_time_ = std::chrono::steady_clock::now();
            pause_active_ = true;
        }
        return last_target_;
    }
    if (pause_active_) {
        cross_wall_stage_time += std::chrono::steady_clock::now() - pause_start_time_;
        pause_active_ = false;
    }

    // lf_foot_exp_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rf_foot_exp_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    // lb_foot_exp_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rb_foot_exp_pos = Eigen::Vector3d(0.0, 0.0, 0.0);

    // lf_foot_exp_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rf_foot_exp_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    // lb_foot_exp_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rb_foot_exp_vel = Eigen::Vector3d(0.0, 0.0, 0.0);

    // lf_foot_exp_acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rf_foot_exp_acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    // lb_foot_exp_acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rb_foot_exp_acc = Eigen::Vector3d(0.0, 0.0, 0.0);

    // lf_foot_exp_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rf_foot_exp_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    // lb_foot_exp_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    // rb_foot_exp_force = Eigen::Vector3d(0.0, 0.0, 0.0);



    Vector3D com_3d;
        std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
        mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());

    RobotTarget joints_target;

    lf_wheel_force        = 0.0f;
    static int last_stage = -1;

    // 辅助函数:计算时间差(秒)
    auto get_elapsed_time = [this]() -> double {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - cross_wall_stage_time);
        return duration.count() / 1000.0;
    };

    if (cross_wall_stage == -1) {
        last_stage  = -1;
        double time = get_elapsed_time();
        if (time > 0.5)
            cross_wall_stage = 0;
    }

        static int debug_print_cnt = 0;
    debug_print_cnt++;
    if (debug_print_cnt >= 250) {
        debug_print_cnt = 0;
        std::cerr << "\033[31mchange_flag = " << change_flag << ", cross_wall_stage = " << cross_wall_stage << "\033[0m" << std::endl;
    }

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

        if(cross_wall_stage == 0  && change_flag == true){

            use_limit_lf = false;
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;               

            lf_step.update_support_trajectory(lf_cart_pos, Vector3D(0.0,0.1,0.0), 1.0);
            rf_step.update_support_trajectory(rf_cart_pos, Vector3D(0.0,0.1,0.0), 1.0);
            lb_step.update_support_trajectory(lb_cart_pos, Vector3D(0.0,0.1,0.0), 1.0);
            rb_step.update_support_trajectory(rb_cart_pos, Vector3D(0.0,0.1,0.0), 1.0);

            //change_flag=false;
            cross_wall_stage = 1;       
        }
        else if (cross_wall_stage == 1 && change_flag == true){        
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();
                last_stage = cross_wall_stage;
            }

            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;

            bool success=false;
            double time=get_elapsed_time();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);   //在简化的二维平面模型中，提供支撑力的位置
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            // 使用Eigen求解每条腿的支撑力
            // 构建方程组: A * f = b
            // 约束条件:
            // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
            // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
            // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
            Eigen::Matrix<double, 3, 4> A;
            Eigen::Vector3d b;
            
            // 第一行: 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // lb
            A(0, 3) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 第二行: 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = lb_pos.x() - mass_center_pos.x();
            A(1, 3) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 第三行: 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = lb_pos.y() - mass_center_pos.y();
            A(2, 3) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 使用最小二乘法求解超定方程组
            Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;
            
            // 将求解的垂直力存入各腿的期望力向量（z分量）
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));
            
            if(!success)
            {
                lf_step.update_support_trajectory(lf_cart_pos,lf_cart_pos,1.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,1.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,1.0);
                lb_step.update_flight_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0), Vector3D(0.1,0.08,-0.02), Vector3D(0.0,0.0,0.0),1.0, 0.1);
                // change_flag=false;
                cross_wall_stage=2;     
            }
        }//1:其它保持不变,迈左后腿
        else if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            double time=get_elapsed_time();

            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            
            // 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 求解3x3方程组
            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
            
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

            if(!success)
            { 
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(0.5, 0.2, -0.1),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 
                
                // change_flag=false;
                cross_wall_stage=3;    
            }
        }
        else if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();
            
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(0.5, -1.0, 1.1),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 
                
                // change_flag=false;
                cross_wall_stage=4;
            }
        }
        else if (cross_wall_stage == 4 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false; 
            bool success=false;
            double time=get_elapsed_time();

                      std::tie(lf_joint_exp_pos_,lf_joint_omega,lf_joint_torque)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(0.5, -0.1, 0),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 

                // change_flag=false;
                cross_wall_stage=5;
            }
        }
        else if(cross_wall_stage==5 && change_flag == true){

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();

                       std::tie(lf_joint_exp_pos_,lf_joint_omega,lf_joint_torque)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            
            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero();
            
            if(!success)
            {
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(0.75, -0.35, 0.6),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 
                // change_flag=false;
                cross_wall_stage=6;
            }
        }
        //右前腿规划：6-9
        else if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();  
                   std::tie(lf_joint_exp_pos_,lf_joint_omega,lf_joint_torque)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
  // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿
            if(!success)
            {
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(-0.1, -0.35, 0.6),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 

                // change_flag=false;
                cross_wall_stage=7;
            }
        }
        else if(cross_wall_stage == 7 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();  
                    std::tie(lf_joint_exp_pos_,lf_joint_omega,lf_joint_torque)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
  // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿
            if(!success)
            {
            lf_step.update_support_trajectory(robot->lf_joint_pos, Vector3D(0, -0.8, 0),3.0);
            rf_step.update_support_trajectory(robot->rf_joint_pos, Vector3D(0, -0.8, 0), 3.0);
            lb_step.update_support_trajectory(robot->lb_joint_pos, Vector3D(0, 0.8, 0), 3.0);
            rb_step.update_support_trajectory(robot->rb_joint_pos, Vector3D(0, -0.8, 0), 3.0);

                // change_flag=false;
                cross_wall_stage=8;
            }
        }
        else if(cross_wall_stage == 8 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();  
            std::tie(lf_joint_exp_pos_,lf_joint_omega,lf_joint_torque)=lf_step.get_target(time, success);
            std::tie(rf_joint_exp_pos_,rf_joint_omega,rf_joint_torque)=rf_step.get_target(time, success);
            std::tie(lb_joint_exp_pos_,lb_joint_omega,lb_joint_torque)=lb_step.get_target(time, success);
            std::tie(rb_joint_exp_pos_,rb_joint_omega,rb_joint_torque)=rb_step.get_target(time, success);
           // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿
            if(!success)
            {
                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(0.7, -0.7, -1.0),2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0); 

                // change_flag=false;
                cross_wall_stage=9;
            }
        }
        // 跨墙结束后重新进入位置控制模式站立：四腿位控，平滑过渡到 default_dof_pos
        else if (cross_wall_stage == 9 && change_flag == true) {
            if (cross_wall_stage != last_stage) {
                cross_wall_stage_time = std::chrono::steady_clock::now();
                last_stage = cross_wall_stage;
                stand_initialized = false;
            }

            use_limit_lf = true;
            use_limit_rf = true;
            use_limit_lb = true;
            use_limit_rb = true;

            if (!stand_initialized) {
                stand_start_joint_pos[0] = robot->rf_joint_pos;
                stand_start_joint_pos[1] = robot->lf_joint_pos;
                stand_start_joint_pos[2] = robot->rb_joint_pos;
                stand_start_joint_pos[3] = robot->lb_joint_pos;
                if (!stand_target_ready) {
                    for (int leg = 0; leg < 4; ++leg) {
                        stand_joint_pos[leg] = stand_start_joint_pos[leg];
                    }
                }
                stand_initialized = true;
            }

            double time = get_elapsed_time();
            double p = std::clamp(time / std::max(stand_duration, 1e-3), 0.0, 1.0);
            double s = p * p * (3.0 - 2.0 * p); // smoothstep，避免位控目标突变

            rf_joint_exp_pos_ = stand_start_joint_pos[0] + s * (stand_joint_pos[0] - stand_start_joint_pos[0]);
            lf_joint_exp_pos_ = stand_start_joint_pos[1] + s * (stand_joint_pos[1] - stand_start_joint_pos[1]);
            rb_joint_exp_pos_ = stand_start_joint_pos[2] + s * (stand_joint_pos[2] - stand_start_joint_pos[2]);
            lb_joint_exp_pos_ = stand_start_joint_pos[3] + s * (stand_joint_pos[3] - stand_start_joint_pos[3]);

            // 站立过渡完成，通知 FSM 切回 GetUp（CheckChange 会置 resume_locomotion_after_crosswall）
            if (p >= 1.0) {
                change_flag = false;
                Cross_wall_over = true;
            }
        }
        // else if(cross_wall_stage == 9 && change_flag == true){
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = std::chrono::steady_clock::now();;
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = true;
        //     use_limit_rb = false;
        //     use_limit_rf = true;
        //     bool success=false;
        //     double time=get_elapsed_time();
           
        //     std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
    
        //     if(!success)
        //     {
        //         lf_step.update_support_trajectory(lf_cart_pos,Vector3D( 0.0, 0.0, 0.0),2.0);
        //         rf_step.update_support_trajectory(rf_cart_pos,Vector3D( 0.0, 0.0, 0.0),2.0);
        //         lb_step.update_support_trajectory(lb_cart_pos,Vector3D( 0.0, 0.0, 0.0),2.0);
        //         rb_step.update_support_trajectory(rb_cart_pos,Vector3D( 0.0, 0.0, 0.0),2.0);
                
        //         // change_flag=false;
        //         cross_wall_stage=10;      
        //     }
        // }
        // else if(cross_wall_stage == 9 && change_flag == true){
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = std::chrono::steady_clock::now();;
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     bool success = false;
        //     double time=get_elapsed_time();

        //     std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
           
        //     if(!success)
        //     {
        //         // change_flag = false;   
        //         cross_wall_stage = 10;
        //     }
        // }
        // else if(cross_wall_stage == 10 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
        //         diagonal_walk.start(robot);
        //         cross_wall_stage_time = std::chrono::steady_clock::now();
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     DiagonalWalkTargets walk_targets;
        //     const bool finished = diagonal_walk.update(robot, mass, mass_center_pos, get_elapsed_time(), walk_targets);

        //     lf_foot_exp_pos = walk_targets.lf_pos ;
        //     rf_foot_exp_pos = walk_targets.rf_pos ;
        //     lb_foot_exp_pos = walk_targets.lb_pos ;
        //     rb_foot_exp_pos = walk_targets.rb_pos ;
        //     lf_foot_exp_vel = walk_targets.lf_vel;
        //     rf_foot_exp_vel = walk_targets.rf_vel;
        //     lb_foot_exp_vel = walk_targets.lb_vel;
        //     rb_foot_exp_vel = walk_targets.rb_vel;
        //     lf_foot_exp_acc = walk_targets.lf_acc;
        //     rf_foot_exp_acc = walk_targets.rf_acc;
        //     lb_foot_exp_acc = walk_targets.lb_acc;
        //     rb_foot_exp_acc = walk_targets.rb_acc;
        //     lf_foot_exp_force = walk_targets.lf_force;
        //     rf_foot_exp_force = walk_targets.rf_force;
        //     lb_foot_exp_force = walk_targets.lb_force;
        //     rb_foot_exp_force = walk_targets.rb_force;

        //     if (finished)
        //     {
               
        //         lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0);         
        //         rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
        //         rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
        //         lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);

        //         // change_flag = false;   
        //         cross_wall_stage = 11;
        //     }
        // }
        // else if (cross_wall_stage == 11 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
              
        //         cross_wall_stage_time = std::chrono::steady_clock::now();;
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     bool success=false;

        //     double time=get_elapsed_time();

        //     std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
        //     if(!success)
        //     {
        //         lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
        //         rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
        //         lb_step.update_support_trajectory(lb_cart_pos,Vector3D(-0.15, 0.18,0.45),2.0);
        //         rb_step.update_support_trajectory(rb_cart_pos,Vector3D(-0.15,-0.18,0.45),2.0);
                
        //         // change_flag=false;
        //         cross_wall_stage=12;
        //     }
        // }
        // else if (cross_wall_stage == 12 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
              
        //         cross_wall_stage_time = std::chrono::steady_clock::now();;
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     bool success=false;

        //     double time=get_elapsed_time();

        //     std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
        //     if(!success)
        //     {
        //         lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
        //         rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
        //         lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.3, 0.1,0.32),2.0);
        //         rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.3,-0.1,0.32),2.0);
                
        //         // change_flag=false;
        //         cross_wall_stage=13;
        //     }
        // }
        // else if (cross_wall_stage == 13 && change_flag == true)
        // { 
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = std::chrono::steady_clock::now();;
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     bool success=false;

        //     double time=get_elapsed_time();

        //     std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
        //     if(!success)
        //     {
        //         // lf_step.update_flight_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),Vector3D(0.12,0.08,0.0),Vector3D(0.0,0.0,0.0),2.0,0.2);
        //         lf_step.update_flight_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),Vector3D(0.12, 0.08,0.15),Vector3D(0.0,0.0,0.0),2.0,0.2);
        //         rf_step.update_flight_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),Vector3D(0.12,-0.08,0.15),Vector3D(0.0,0.0,0.0),2.0,0.2);
        //         // lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.12, 0.08, 0.15),2.0); 
        //         // rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.12,-0.08, 0.15),2.0);  
        //         lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
        //         rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
        //         // change_flag = false;   
        //         cross_wall_stage=14;
        //     }
        // }
        // else if(cross_wall_stage == 14 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = std::chrono::steady_clock::now();
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
         
        //     bool success=false;

        //     double time=get_elapsed_time();

        //     std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
        //     std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);

        //     if (!success)
        //     {
        //         // change_flag = false;   
        //         cross_wall_stage=15;
        //         // Cross_wall_over = true;
        //     }
        // }
        // else if(cross_wall_stage == 15 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
        //         diagonal_walk.configure(0.6, 0.08, 0.3, 0.3, 0.0, 0.0, 0.8, 0.0);
        //         diagonal_walk.start(robot);
        //         cross_wall_stage_time = std::chrono::steady_clock::now();
        //         last_stage = cross_wall_stage;
        //     }
        //     use_limit_lb = false;
        //     use_limit_lf = false;
        //     use_limit_rb = false;
        //     use_limit_rf = false;
        //     DiagonalWalkTargets walk_targets;
        //     const bool finished = diagonal_walk.update(robot, mass, mass_center_pos, get_elapsed_time(), walk_targets);

        //     lf_foot_exp_pos = walk_targets.lf_pos + Eigen::Vector3d(0.0,0.0,0.06);
        //     rf_foot_exp_pos = walk_targets.rf_pos + Eigen::Vector3d(0.0,0.0,0.06);

        //     // lf_foot_exp_pos = walk_targets.lf_pos;
        //     // rf_foot_exp_pos = walk_targets.rf_pos;
        //     lb_foot_exp_pos = walk_targets.lb_pos;
        //     rb_foot_exp_pos = walk_targets.rb_pos; 
        //     lf_foot_exp_vel = walk_targets.lf_vel;
        //     rf_foot_exp_vel = walk_targets.rf_vel;
        //     lb_foot_exp_vel = walk_targets.lb_vel;
        //     rb_foot_exp_vel = walk_targets.rb_vel;
        //     lf_foot_exp_acc = walk_targets.lf_acc;
        //     rf_foot_exp_acc = walk_targets.rf_acc;
        //     lb_foot_exp_acc = walk_targets.lb_acc;
        //     rb_foot_exp_acc = walk_targets.rb_acc;
        //     lf_foot_exp_force = walk_targets.lf_force;
        //     rf_foot_exp_force = walk_targets.rf_force;
        //     lb_foot_exp_force = walk_targets.lb_force;
        //     rb_foot_exp_force = walk_targets.rb_force;
        //     lb_foot_exp_vel = Eigen::Vector3d(0.0,0.0,0.0);
        //     rb_foot_exp_vel = Eigen::Vector3d(0.0,0.0,0.0);
        //     lb_foot_exp_acc = Eigen::Vector3d(0.0,0.0,0.0);
        //     rb_foot_exp_acc = Eigen::Vector3d(0.0,0.0,0.0);
        //     lb_foot_exp_force = Eigen::Vector3d(0.0,0.0,0.0);
        //     rb_foot_exp_force = Eigen::Vector3d(0.0,0.0,0.0);

        //     if (finished)
        //     {
        //         change_flag = false;   
        //         Cross_wall_over = true;
        //     }
        // }

    /*******************************lf**********************************/
    if (use_limit_lf) {

        for (int i = 0; i < 3; i++) {
            joints_target.legs[1].joints[i].kp  = robot->lf_leg_calc->kp[i];
            joints_target.legs[1].joints[i].kd  = robot->lf_leg_calc->kd[i];
            joints_target.legs[1].joints[i].rad = lf_joint_exp_pos_[i];
        }
        robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
        lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        // static int cnt_ = 0;
        // cnt_++;
        // if(cnt_>=1)
        // {
        //     cnt_ = 0;
        //     std::cerr << "\033[31mlf_joint_pos = (" << robot->lf_joint_pos[0] << ", " 
        //              << robot->lf_joint_pos[1] << " " << robot->lf_joint_pos[2] 
        //              << "), lf_joint_exp_pos_ = (" << lf_joint_exp_pos_[0] << ", " 
        //              << lf_joint_exp_pos_[1] << ", " << lf_joint_exp_pos_[2] << ")\033[0m" << std::endl;
        // }
    } else {
        joints_target.legs[1] = robot->lf_leg_calc->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    }

    /*******************************rf**********************************/
    if (use_limit_rf) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[0].joints[i].rad = rf_joint_exp_pos_[i];
            joints_target.legs[0].joints[i].kp  = robot->rf_leg_calc->kp[i];
            joints_target.legs[0].joints[i].kd  = robot->rf_leg_calc->kd[i];
        }
        robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
        rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    } else {
        joints_target.legs[0] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    }
    
    /*******************************lb**********************************/
    if (use_limit_lb) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[3].joints[i].kp  = robot->lb_leg_calc->kp[i];
            joints_target.legs[3].joints[i].kd  = robot->lb_leg_calc->kd[i];
            joints_target.legs[3].joints[i].rad = lb_joint_exp_pos_[i];
        }
        robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
        lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    } else {
        joints_target.legs[3] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    }
    
    /*******************************rb**********************************/
    if (use_limit_rb) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[2].joints[i].kp  = robot->rb_leg_calc->kp[i];
            joints_target.legs[2].joints[i].kd  = robot->rb_leg_calc->kd[i];
            joints_target.legs[2].joints[i].rad = rb_joint_exp_pos_[i];
        }
        robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
        rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    } else {
        joints_target.legs[2] = robot->rb_leg_calc->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);
    }

    // TODO: 替换为纯 C++ 的控制指令输出方式
    // robot->legs_target_pub->publish(joints_target);
    // 目前直接返回 joints_target,由调用者处理
    //     if (std::isnan(robot->lf_joint_torque[0]) || std::isinf(robot->lf_joint_torque[0])
    //     || std::isnan(robot->rf_joint_torque[0]) || std::isinf(robot->rf_joint_torque[0])
    //     || std::isnan(robot->lb_joint_torque[0]) || std::isinf(robot->lb_joint_torque[0])
    //     || std::isnan(robot->rb_joint_torque[0]) || std::isinf(robot->rb_joint_torque[0])) {
        
    //     std::cerr << "\033[31mError: Invalid joint torque value detected!"<< ", cross_wall_stage = " << cross_wall_stage << "\033[0m" << std::endl;
    // }
        // 打印当前各关节的输出
    static int print_counter = 0;
    if (print_counter++ % 500 == 0) {  // 每10次调用打印一次，避免刷屏
        std::cout << "\n========== Joint Output (stage=" << cross_wall_stage << ") ==========" << std::endl;
        
        // 左前腿 (LF) - legs[1]
        std::cout << "LF: pos=[" << joints_target.legs[1].joints[0].rad << ", " 
                  << joints_target.legs[1].joints[1].rad << ", " 
                  << joints_target.legs[1].joints[2].rad << "] | "
                  << "vel=[" << joints_target.legs[1].joints[0].omega << ", " 
                  << joints_target.legs[1].joints[1].omega << ", " 
                  << joints_target.legs[1].joints[2].omega << "] | "
                  << "kp=[" << joints_target.legs[0].joints[0].kp << ", " 
                  << joints_target.legs[1].joints[1].kp << ", " 
                  << joints_target.legs[1].joints[2].kp << "] | "
                  << "kd=[" << joints_target.legs[1].joints[0].kd << ", " 
                  << joints_target.legs[1].joints[1].kd << ", " 
                  << joints_target.legs[1].joints[2].kd << "]" << std::endl;
        
        // 右前腿 (RF) - legs[0]
        std::cout << "RF: pos=[" << joints_target.legs[0].joints[0].rad << ", " 
                  << joints_target.legs[0].joints[1].rad << ", " 
                  << joints_target.legs[0].joints[2].rad << "] | "
                  << "vel=[" << joints_target.legs[0].joints[0].omega << ", " 
                  << joints_target.legs[0].joints[1].omega << ", " 
                  << joints_target.legs[0].joints[2].omega << "] | "
                  << "kp=[" << joints_target.legs[0].joints[0].kp << ", " 
                  << joints_target.legs[0].joints[1].kp << ", " 
                  << joints_target.legs[0].joints[2].kp << "] | "
                  << "kd=[" << joints_target.legs[0].joints[0].kd << ", " 
                  << joints_target.legs[0].joints[1].kd << ", " 
                  << joints_target.legs[0].joints[2].kd << "]" << std::endl;
        
        // 左后腿 (LB) - legs[3]
        std::cout << "LB: pos=[" << joints_target.legs[3].joints[0].rad << ", " 
                  << joints_target.legs[3].joints[1].rad << ", " 
                  << joints_target.legs[3].joints[2].rad << "] | "
                  << "vel=[" << joints_target.legs[3].joints[0].omega << ", " 
                  << joints_target.legs[3].joints[1].omega << ", " 
                  << joints_target.legs[3].joints[2].omega << "] | "
                  << "kp=[" << joints_target.legs[3].joints[0].kp << ", " 
                  << joints_target.legs[3].joints[1].kp << ", " 
                  << joints_target.legs[3].joints[2].kp << "] | "
                  << "kd=[" << joints_target.legs[3].joints[0].kd << ", " 
                  << joints_target.legs[3].joints[1].kd << ", " 
                  << joints_target.legs[3].joints[2].kd << "]" << std::endl;
        std::cout << "lb_foot_exp_pos: " << lb_foot_exp_pos << std::endl;
        // 右后腿 (RB) - legs[2]
        std::cout << "RB: pos=[" << joints_target.legs[2].joints[0].rad << ", " 
                  << joints_target.legs[2].joints[1].rad << ", " 
                  << joints_target.legs[2].joints[2].rad << "] | "
                  << "vel=[" << joints_target.legs[2].joints[0].omega << ", " 
                  << joints_target.legs[2].joints[1].omega << ", " 
                  << joints_target.legs[2].joints[2].omega << "] | "
                  << "kp=[" << joints_target.legs[2].joints[0].kp << ", " 
                  << joints_target.legs[2].joints[1].kp << ", " 
                  << joints_target.legs[2].joints[2].kp << "] | "
                  << "kd=[" << joints_target.legs[2].joints[0].kd << ", " 
                  << joints_target.legs[2].joints[1].kd << ", " 
                  << joints_target.legs[2].joints[2].kd << "]" << std::endl;
        
        std::cout << "==============================================\n" << std::endl;


        // std::cout << "==============================================\n" << std::endl;
        
        // // 打印期望关节角和实际关节角的差值 (Expected Joint - Actual Joint)
        // std::cout << "========== Joint Angle Error (Exp - Act) ==========" << std::endl;
        
        // // LF 误差 (注意：在 joints_target 中 LF 对应 legs[1])
        // std::cout << "LF err: [" 
        //           << joints_target.legs[1].joints[0].rad - robot->lf_joint_pos[0] << ", "
        //           << joints_target.legs[1].joints[1].rad - robot->lf_joint_pos[1] << ", "
        //           << joints_target.legs[1].joints[2].rad - robot->lf_joint_pos[2] << "]" << std::endl;
        
        // // RF 误差 (注意：在 joints_target 中 RF 对应 legs[0])
        // std::cout << "RF err: [" 
        //           << joints_target.legs[0].joints[0].rad - robot->rf_joint_pos[0] << ", "
        //           << joints_target.legs[0].joints[1].rad - robot->rf_joint_pos[1] << ", "
        //           << joints_target.legs[0].joints[2].rad - robot->rf_joint_pos[2] << "]" << std::endl;
        
        // // LB 误差 (注意：在 joints_target 中 LB 对应 legs[3])
        // std::cout << "LB err: [" 
        //           << joints_target.legs[3].joints[0].rad - robot->lb_joint_pos[0] << ", "
        //           << joints_target.legs[3].joints[1].rad - robot->lb_joint_pos[1] << ", "
        //           << joints_target.legs[3].joints[2].rad - robot->lb_joint_pos[2] << "]" << std::endl;
        
        // // RB 误差 (注意：在 joints_target 中 RB 对应 legs[2])
        // std::cout << "RB err: [" 
        //           << joints_target.legs[2].joints[0].rad - robot->rb_joint_pos[0] << ", "
        //           << joints_target.legs[2].joints[1].rad - robot->rb_joint_pos[1] << ", "
        //           << joints_target.legs[2].joints[2].rad - robot->rb_joint_pos[2] << "]" << std::endl;
        
        // std::cout << "=====================================================\n" << std::endl;
    }

        
    

    last_target_ = joints_target;
    has_last_target_ = true;
    return joints_target;
}
