/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_atdog3.hpp"
#include "fsm_atdog3.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <atomic>

RL_Real::RL_Real(int argc, char** argv, const rclcpp::Node::SharedPtr node) {

    this->node_ = node;
    this->robot_name = "atdog3";
    this->ReadYaml("atdog3", "base.yaml");

    // 创建状态机
    this->fsm = *FSMManager::GetInstance().CreateFSM("atdog3", this);

    // 初始化机器人
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    // Shut down motion control-related service

    leg_driver = std::make_unique<LegDriver>();
    std::weak_ptr<rclcpp::Node> weak_node = node_;
    leg_driver->set_motor_error_callback([weak_node](uint16_t motor_state) {
        static std::atomic_bool motor_error_shutdown_requested{false};
        if (motor_error_shutdown_requested.exchange(true)) {
            return;
        }
        auto node = weak_node.lock();
        if (!node) {
            return;
        }
        std::cout << LOGGER::ERROR << "atdog3 电机异常，准备退出节点，motor_state=" << motor_state << std::endl;
        rclcpp::shutdown();
    });

    cmd_sub =
        node_->create_subscription<robot_msgs::msg::Cmd>("robot_move_cmd", 10, [this](const robot_msgs::msg::Cmd& msg) {
            remote_cmd = msg;
            if (msg.mode != last_logged_remote_mode_) {
                last_logged_remote_mode_ = msg.mode;
                RCLCPP_INFO(node_->get_logger(), "robot_move_cmd mode=%d", msg.mode);
            }
        });


    // 键盘控制、底层控制、策略推理循环
    this->loop_command = std::make_shared<LoopFunc>("loop_command", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl      = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_command->start();
    this->loop_control->start();
    this->loop_rl->start();

    leg_driver->enable_control(true);
}

RL_Real::~RL_Real() {
    this->loop_command->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float>* state) {
    if (state == nullptr) {
        return;
    }

    const int dof_count = this->params.Get<int>("num_of_dofs");
    if (dof_count > 0 && static_cast<int>(state->motor_state.q.size()) != dof_count) {
        state->motor_state.resize(static_cast<size_t>(dof_count));
    }

    std::array<float, 4> q{};
    std::array<float, 3> w{};
    const bool has_imu_state = this->leg_driver != nullptr && this->leg_driver->get_imu_state(q, w);
    if (has_imu_state) {
        state->imu.quaternion[0] = static_cast<float>(q[0]);
        state->imu.quaternion[1] = static_cast<float>(q[1]);
        state->imu.quaternion[2] = static_cast<float>(q[2]);
        state->imu.quaternion[3] = static_cast<float>(q[3]);

        state->imu.gyroscope[0] = static_cast<float>(w[0]);
        state->imu.gyroscope[1] = static_cast<float>(w[1]);
        state->imu.gyroscope[2] = static_cast<float>(w[2]);
    } else {
        state->imu.quaternion[0] = 1.0;
        state->imu.quaternion[1] = 0.0;
        state->imu.quaternion[2] = 0.0;
        state->imu.quaternion[3] = 0.0;

        state->imu.gyroscope[0] = 0.0;
        state->imu.gyroscope[1] = 0.0;
        state->imu.gyroscope[2] = 0.0;
    }

    if (this->leg_driver == nullptr || dof_count <= 0) {
        return;
    }

    std::array<LegState_t, 4> legs_state{};
    if (!this->leg_driver->get_leg_state(legs_state)) {
        return;
    }

    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < dof_count; ++i) {
        const int hw_index    = (i < static_cast<int>(joint_mapping.size())) ? joint_mapping[i] : i;
        const int leg_index   = hw_index / 3;
        const int joint_index = hw_index % 3;
        if (leg_index < 0 || leg_index >= static_cast<int>(legs_state.size())) {
            continue;
        }

        state->motor_state.q[i]       = legs_state[leg_index].joint[joint_index].rad;
        state->motor_state.dq[i]      = legs_state[leg_index].joint[joint_index].omega;
        state->motor_state.tau_est[i] = legs_state[leg_index].joint[joint_index].torque;
    }

    // std::cout<<"cur_pos:"<<state->motor_state.q<<std::endl;
}

void RL_Real::RobotControl() {
    // 获取各个传感器数据，遥控器期望，填写到robot_state中
    this->GetState(&this->robot_state);

    if (remote_cmd.mode != 0) {     //由ROS2上层接管控制
        this->control.setMode(remote_cmd.mode);
        const float vx = std::clamp(remote_cmd.vx, -1.5f, 1.5f);
        const float vy = std::clamp(remote_cmd.vy, -1.0f, 1.0f);
        const float vz = std::clamp(remote_cmd.vz, -1.5f, 1.5f);
        this->control.setVel(vx, vy, vz);
    }
    else {
        this->control.setMode(0);
    }

    // 执行状态机，送入state，输出command
    this->StateController(&this->robot_state, &this->robot_command);

    // 清空上一次遥控器的输入
    this->control.ClearInput();

    // 将命令发送到电机
    this->SetCommand(&this->robot_command);
}

void RL_Real::SetCommand(const RobotCommand<float>* command) {
    if (command == nullptr || this->leg_driver == nullptr) {
        return;
    }

    std::array<LegTarget_t, 4> legs_target{};
    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            legs_target[leg].joint[joint].rad    = 0.0f;
            legs_target[leg].joint[joint].omega  = 0.0f;
            legs_target[leg].joint[joint].torque = 0.0f;
            legs_target[leg].joint[joint].kp     = 0.0f;
            legs_target[leg].joint[joint].kd     = 0.0f;
        }
        legs_target[leg].wheel.omega  = 0.0f;
        legs_target[leg].wheel.torque = 0.0f;
    }

    const int dof_count      = this->params.Get<int>("num_of_dofs");
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");

    for (int dof = 0; dof < dof_count; ++dof) {
        const int hw_index       = (dof < static_cast<int>(joint_mapping.size())) ? joint_mapping[dof] : dof;
        const int leg_index      = hw_index / 3;
        const int actuator_index = hw_index % 3;

        if (dof >= 12) {
            legs_target[dof - 12].wheel.omega  = command->motor_command.dq[dof];
            legs_target[dof - 12].wheel.torque = command->motor_command.tau[dof];
        } else {
            if (dof < static_cast<int>(command->motor_command.q.size())) {
                legs_target[leg_index].joint[actuator_index].rad = command->motor_command.q[dof];
            }
            if (dof < static_cast<int>(command->motor_command.dq.size())) {
                legs_target[leg_index].joint[actuator_index].omega = command->motor_command.dq[dof];
            }
            if (dof < static_cast<int>(command->motor_command.tau.size())) {
                legs_target[leg_index].joint[actuator_index].torque = command->motor_command.tau[dof];
            }
            if (dof < static_cast<int>(command->motor_command.kp.size())) {
                legs_target[leg_index].joint[actuator_index].kp = command->motor_command.kp[dof];
            }
            if (dof < static_cast<int>(command->motor_command.kd.size())) {
                legs_target[leg_index].joint[actuator_index].kd = command->motor_command.kd[dof];
            }
        }
    }

    const auto now = std::chrono::steady_clock::now();
    const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    this->leg_driver->set_leg_target(legs_target, static_cast<uint32_t>(ms));
}

void RL_Real::RunModel() {
    if (this->rl_init_done) {
        this->episode_length_buf += 1;
        this->obs.ang_vel  = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};

        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos   = this->robot_state.motor_state.q;
        this->obs.dof_vel   = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty()) {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty()) {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty()) {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);
    }
}

std::vector<float> RL_Real::Forward() {
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock()) {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty()) {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions           = this->model->forward({this->history_obs});
    } else {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty()
        && !this->params.Get<std::vector<float>>("clip_actions_lower").empty()) {
        return clamp(
            actions, this->params.Get<std::vector<float>>("clip_actions_lower"),
            this->params.Get<std::vector<float>>("clip_actions_upper"));
    } else {
        return actions;
    }
}


volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum) {
    std::cout << LOGGER::INFO << "Received signal " << signum << ", shutting down..." << std::endl;
    g_shutdown_requested = 1;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_controller_node");
    RL_Real rl_sar(argc, argv, node);
    rclcpp::spin(node);
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;
    rclcpp::shutdown();
    return 0;
}
