/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_ATDOG2_HPP
#define RL_REAL_ATDOG2_HPP

// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "loop.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "leg_driver/leg_driver.hpp"
#include "imu_driver/imu_driver.hpp"

#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>


class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv,const rclcpp::Node::SharedPtr node);
    ~RL_Real();

private:
    rclcpp::Node::SharedPtr node_;      //用于ROS2订阅通信的节点
    rclcpp::Subscription<robot_msgs::msg::Cmd>::SharedPtr cmd_sub;
    robot_msgs::msg::Cmd remote_cmd;
    int last_logged_remote_mode_ = -1;
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_command;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    //real_port
    std::unique_ptr<IMUDriver> imu_driver;
    std::unique_ptr<LegDriver> leg_driver;
    
    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;
};

#endif // RL_REAL_GO2_HPP
