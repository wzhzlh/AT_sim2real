#ifndef RL_REAL_ARM1_HPP
#define RL_REAL_ARM1_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class Arm1CDC;

class RL_RealArm1 : public RL
{
public:
    RL_RealArm1(int argc, char **argv, const std::string &device_override = "");
    ~RL_RealArm1();

private:
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    std::shared_ptr<LoopFunc> loop_keyboard_;
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;
    std::unique_ptr<Arm1CDC> cdc_;
    std::chrono::steady_clock::time_point last_tx_{};
    std::mutex state_mutex_;
};

#endif
