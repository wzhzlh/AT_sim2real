/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATDOG2_FSM_HPP
#define ATDOG2_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <Eigen/Dense>
#include <cmath>  // for std::isnan, std::isinf
#include "bridge_B.hpp"
#include "cross_wall_atdog2.hpp"
namespace atdog2_fsm
{

inline std::string ResolveRemoteModeState(int mode, const std::string& current_state)
{
    std::string target_state;
    switch (mode)
    {
        case 1:
            target_state = "RLFSMStateGetUp";
            break;
        case 2:
            target_state = "RLFSMStateRLLocomotion";
            break;
        case 3:
            target_state = "RLFSMStateRLStairs";
            break;
        case 4:
            target_state = "RLFSMStateRLSand";
            break;
        case 5:
            target_state = "RLFSMStateRLSlope";
            break;
        case 6:
            target_state = "RLFSMStateRLBar";
            break;
        case 7:
            target_state = "RLFSMStateRLBridge";
            break;
        case 8:
            target_state = "RLFSMStateCrosswall";
            break;
        case 9:
            if (current_state == "RLFSMStateGetUp")
                target_state = "RLFSMStateCheck";
            break;
        default:
            return current_state;
    }

    return target_state == current_state ? current_state : target_state;
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}


    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, -0.8, 0.0,
        0.00, 0.8, 0.0,
        0.00, -0.8, 0.0,
        0.00, 0.8, 0.0,
        0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;
    bool stand_from_check = false;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        stand_from_check = false;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else if (rl.fsm.previous_state_->GetStateName() == "RLFSMStateCheck")
        {
            stand_from_check = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        std::cout<<"当前关节角:"<<rl.now_state.motor_state.q<<std::endl;
    }

    void Run() override
    {
        if(stand_from_check)
        {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 3.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 3.0f, "Getting up", true)) return;
        }
        else 
        {
            if(stand_from_passive)
            {

                if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
                if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
            }
            else
            {
                if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
            }

        }


    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.resume_locomotion_after_crosswall)
            {
                rl.resume_locomotion_after_crosswall = false;
                return "RLFSMStateRLLocomotion";
            }
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
            {
                return "RLFSMStateCrosswall";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num4)
            {
                return "RLFSMStateBridgeB";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::Y)
            {
                return "RLFSMStateCheck";
            }

            return ResolveRemoteModeState(rl.control.mode, state_name_);
        }
        
        return state_name_;
    }
};

class RLFSMStateCheck : public RLFSMState
{
public:
    RLFSMStateCheck(RL *rl) : RLFSMState(*rl, "RLFSMStateCheck") {}


    float percent_pre_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, -1.13, 0.306,
        0.00, 1.13, -0.306,
        0.00, 1.13, -3.52,
        0.00, -1.13, 3.52,
        0.00, 0.00, 0.00, 0.00
    };

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        std::cout<<"当前关节角:"<<rl.now_state.motor_state.q<<std::endl;
    }

    void Run() override
    {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 5.0f, "RLFSMStateCheck", true)) return;
    }

    void Exit() override {}

    std::string CheckChange() override
    {

        if (percent_pre_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            {
                return "RLFSMStateGetUp";
            }

            //std::cout<<"检查切换\n";
            if(rl.control.mode==1)   //转到位控站立状态
            {
                return "RLFSMStateGetUp";
            }

        }
        
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

class RLFSMStateCrosswall : public RLFSMState {


public:
    int num_dofs;
    std::shared_ptr<CrossWallStateAtdog2> cross_wall_state;
    RLFSMStateCrosswall(RL *rl) : RLFSMState(*rl, "RLFSMStateCrosswall") {
        std::string urdf_path = "src/rl_sar_zoo/" + rl->robot_name + "_description/urdf/dog2.urdf";
        cross_wall_state = std::make_shared<CrossWallStateAtdog2>(urdf_path);
        // stage 9 位控站立目标：复用 RL 配置中的标准站立关节角
        cross_wall_state->setStandJointPos(rl->params.Get<std::vector<float>>("default_dof_pos"));
    }
    
    

    void Enter() override
    {
        rl.now_state = *fsm_state;
        num_dofs = rl.params.Get<int>("num_of_dofs");
        cross_wall_state->enter();
        
    }

    void Run() override
    {
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;    // ✅ atdog2 是 3 关节/腿
            int jonit_index = i % 3;  // ✅ 索引范围 0-2
            // 获取当前电机状态
            switch (leg_index) 
            {
                case 0:
                    cross_wall_state->robot->rf_joint_pos[jonit_index] = fsm_state->motor_state.q[i];      // 位置
                    cross_wall_state->robot->rf_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];    // 速度
                break;
                case 1:
                    cross_wall_state->robot->lf_joint_pos[jonit_index] = fsm_state->motor_state.q[i];      
                    cross_wall_state->robot->lf_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                case 3:
                    cross_wall_state->robot->lb_joint_pos[jonit_index] = fsm_state->motor_state.q[i];
                    cross_wall_state->robot->lb_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                case 2:
                    cross_wall_state->robot->rb_joint_pos[jonit_index] = fsm_state->motor_state.q[i];
                    cross_wall_state->robot->rb_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                default:
                    break;
            }
            
        }
        RobotTarget joints_target;
        joints_target = cross_wall_state->update();
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;    // ✅ atdog2 是 3 关节/腿
            int jonit_index = i % 3;  // ✅ 索引范围 0-2
            
            // 防止 NaN 或 Inf 值
            float q_val = joints_target.legs[leg_index].joints[jonit_index].rad;
            float dq_val = joints_target.legs[leg_index].joints[jonit_index].omega;
            float kp_val = joints_target.legs[leg_index].joints[jonit_index].kp;
            float kd_val = joints_target.legs[leg_index].joints[jonit_index].kd;

            
            // 检查并修正无效值
            // if (std::isnan(q_val) || std::isinf(q_val)) q_val = 0.0f;
            // if (std::isnan(dq_val) || std::isinf(dq_val)) dq_val = 0.0f;
            // if (std::isnan(kp_val) || std::isinf(kp_val)) kp_val = 0.0f;
            // if (std::isnan(kd_val) || std::isinf(kd_val)) kd_val = 0.0f;
            
            fsm_command->motor_command.q[i] = q_val;
            fsm_command->motor_command.dq[i] = dq_val;
            fsm_command->motor_command.kp[i] = kp_val;
            fsm_command->motor_command.kd[i] = kd_val;
            fsm_command->motor_command.tau[i] = joints_target.legs[leg_index].joints[jonit_index].torque;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        const std::string remote_target = ResolveRemoteModeState(rl.control.mode, state_name_);
        if (remote_target != state_name_)
        {
            rl.resume_locomotion_after_crosswall = false;
            cross_wall_state->RL_walk_flag = false;
            cross_wall_state->Cross_wall_over = false;
            cross_wall_state->change_flag = true;
            cross_wall_state->cross_wall_stage = -1;
            return remote_target;
        }
        if(cross_wall_state->RL_walk_flag == true)
        {
            rl.resume_locomotion_after_crosswall = false;
            cross_wall_state->RL_walk_flag = false;
            cross_wall_state->cross_wall_stage = 13;
            return "RLFSMStateRLLocomotion";
        }
        if(cross_wall_state->Cross_wall_over == true)
        {
            rl.resume_locomotion_after_crosswall = true;
            cross_wall_state->Cross_wall_over = false;
            cross_wall_state->cross_wall_stage = -1;
            return "RLFSMStateGetUp";
        }
        
        return state_name_;
    }
};

class RLFSMStateBridgeB : public RLFSMState {
public:
    int num_dofs;
    std::shared_ptr<BridgeBStateAtdog2> bridge_b_state;

    RLFSMStateBridgeB(RL *rl) : RLFSMState(*rl, "RLFSMStateBridgeB") {
        std::string urdf_path = "src/rl_sar_zoo/" + rl->robot_name + "_description/urdf/dog2.urdf";
        bridge_b_state = std::make_shared<BridgeBStateAtdog2>(urdf_path);
    }

    void Enter() override
    {
        rl.now_state = *fsm_state;
        num_dofs = rl.params.Get<int>("num_of_dofs");
        bridge_b_state->enter();
    }

    void Run() override
    {
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;
            int joint_index = i % 3;
            switch (leg_index)
            {
                case 0:
                    bridge_b_state->robot->rf_joint_pos[joint_index] = fsm_state->motor_state.q[i];
                    bridge_b_state->robot->rf_joint_vel[joint_index] = fsm_state->motor_state.dq[i];
                    break;
                case 1:
                    bridge_b_state->robot->lf_joint_pos[joint_index] = fsm_state->motor_state.q[i];
                    bridge_b_state->robot->lf_joint_vel[joint_index] = fsm_state->motor_state.dq[i];
                    break;
                case 2:
                    bridge_b_state->robot->rb_joint_pos[joint_index] = fsm_state->motor_state.q[i];
                    bridge_b_state->robot->rb_joint_vel[joint_index] = fsm_state->motor_state.dq[i];
                    break;
                case 3:
                    bridge_b_state->robot->lb_joint_pos[joint_index] = fsm_state->motor_state.q[i];
                    bridge_b_state->robot->lb_joint_vel[joint_index] = fsm_state->motor_state.dq[i];
                    break;
                default:
                    break;
            }
        }

        RobotTarget joints_target = bridge_b_state->update();
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;
            int joint_index = i % 3;
            fsm_command->motor_command.q[i] = joints_target.legs[leg_index].joints[joint_index].rad;
            fsm_command->motor_command.dq[i] = joints_target.legs[leg_index].joints[joint_index].omega;
            fsm_command->motor_command.kp[i] = joints_target.legs[leg_index].joints[joint_index].kp;
            fsm_command->motor_command.kd[i] = joints_target.legs[leg_index].joints[joint_index].kd;
            fsm_command->motor_command.tau[i] = joints_target.legs[leg_index].joints[joint_index].torque;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (bridge_b_state->bridge_over == true)
        {
            bridge_b_state->bridge_over = false;
            bridge_b_state->bridge_stage = -1;
            return "RLFSMStateGetUp";
        }
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;
    std::chrono::steady_clock::time_point cross_enter_time = std::chrono::steady_clock::now();
    bool RL_to_Cross = false;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStateCrosswall")
        {
            cross_enter_time = std::chrono::steady_clock::now();
            RL_to_Cross = true;
            rl.control.setVel(0.4f, 0.0f, 0.0f);
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if((std::chrono::steady_clock::now() - cross_enter_time > std::chrono::milliseconds(3000)) && RL_to_Cross)
        {
            RL_to_Cross = false;
            rl.control.setVel(0.0f, 0.0f, 0.0f);
            return "RLFSMStateCrosswall";
        }
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateCrosswall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4)
        {
            return "RLFSMStateBridgeB";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};


class RLFSMStateRLStairs : public RLFSMState
{
public:
    RLFSMStateRLStairs(RL *rl) : RLFSMState(*rl, "RLFSMStateRLStairs") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_stairs";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

class RLFSMStateRLSand : public RLFSMState
{
public:
    RLFSMStateRLSand(RL *rl) : RLFSMState(*rl, "RLFSMStateRLSand") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_sand";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

//限高杆状态
class RLFSMStateRLBar : public RLFSMState
{
public:
    RLFSMStateRLBar(RL *rl) : RLFSMState(*rl, "RLFSMStateRLBar") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_bar";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

class RLFSMStateRLSlope : public RLFSMState
{
public:
    RLFSMStateRLSlope(RL *rl) : RLFSMState(*rl, "RLFSMStateRLSlope") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_slope";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

class RLFSMStateRLBridge : public RLFSMState
{
public:
    RLFSMStateRLBridge(RL *rl) : RLFSMState(*rl, "RLFSMStateRLBridge") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_bridge";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }

        return ResolveRemoteModeState(rl.control.mode, state_name_);
    }
};

} // namespace atdog2_fsm

class ATDog2FSMFactory : public FSMFactory
{
public:
    ATDog2FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<atdog2_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<atdog2_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<atdog2_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<atdog2_fsm::RLFSMStateRLLocomotion>(rl);
        else if (state_name == "RLFSMStateRLStairs")
            return std::make_shared<atdog2_fsm::RLFSMStateRLStairs>(rl);
        else if (state_name == "RLFSMStateRLSand")
            return std::make_shared<atdog2_fsm::RLFSMStateRLSand>(rl);
        else if (state_name == "RLFSMStateCrosswall")
            return std::make_shared<atdog2_fsm::RLFSMStateCrosswall>(rl);
        else if (state_name == "RLFSMStateBridgeB")
            return std::make_shared<atdog2_fsm::RLFSMStateBridgeB>(rl);
        else if (state_name == "RLFSMStateRLBar")
            return std::make_shared<atdog2_fsm::RLFSMStateRLBar>(rl);
        else if (state_name == "RLFSMStateRLSlope")
            return std::make_shared<atdog2_fsm::RLFSMStateRLSlope>(rl);
        else if (state_name == "RLFSMStateRLBridge")
            return std::make_shared<atdog2_fsm::RLFSMStateRLBridge>(rl);
        else if (state_name == "RLFSMStateCheck")
            return std::make_shared<atdog2_fsm::RLFSMStateCheck>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "atdog2"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion",
            "RLFSMStateRLStairs",
            "RLFSMStateRLSand",
            "RLFSMStateCrosswall",
            "RLFSMStateBridgeB",
            "RLFSMStateRLBar",
            "RLFSMStateRLSlope",
            "RLFSMStateRLBridge",
            "RLFSMStateCheck"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(ATDog2FSMFactory, "RLFSMStatePassive")

#endif // ATDOG2_FSM_HPP
