#ifndef ARM1_FSM_HPP
#define ARM1_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

class Arm1FSMStatePassive;

class Arm1FSMState : public RLFSMState
{
public:
    explicit Arm1FSMState(RL *rl) : RLFSMState(*rl, "Arm1FSMState") {}

    void Enter() override
    {
        rl.rl_init_done = false;
        rl.episode_length_buf = 0;
        try
        {
            rl.config_name = "arm1";
            rl.InitRL("arm1/arm1");
            if (fsm_state) rl.now_state = *fsm_state;
            std::cout << LOGGER::INFO << "Loaded arm1 policy. Press '0' to return to the hold pose." << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Could not initialize arm1 policy: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("Arm1FSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;
        RLControl();
        std::vector<float> unused_tau;
        while (rl.output_dof_tau_queue.try_pop(unused_tau)) {}
    }

    void Exit() override { rl.rl_init_done = false; }

    std::string CheckChange() override;
};

class Arm1FSMStatePassive : public RLFSMState
{
public:
    explicit Arm1FSMStatePassive(RL *rl) : RLFSMState(*rl, "Arm1FSMStatePassive") {}

    void Enter() override
    {
        rl.rl_init_done = false;
        std::cout << LOGGER::INFO << "Arm hold mode. Press '1' to load and run the arm1 policy." << std::endl;
    }

    void Run() override
    {
        if (!fsm_state || !fsm_command) return;
        if (returning_home_)
        {
            if (Interpolate(return_percent_, return_start_, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Returning to hold pose", true))
                return;
            returning_home_ = false;
            hold_position_ = rl.params.Get<std::vector<float>>("default_dof_pos");
        }
        else if (hold_position_.size() != fsm_state->motor_state.q.size())
        {
            hold_position_ = fsm_state->motor_state.q;
        }
        const auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        const auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
        for (size_t i = 0; i < fsm_state->motor_state.q.size(); ++i)
        {
            fsm_command->motor_command.q[i] = hold_position_[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.tau[i] = 0.0f;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
        }
    }

    void BeginReturn(const std::vector<float> &start)
    {
        return_start_ = start;
        return_percent_ = 0.0f;
        returning_home_ = true;
        hold_position_.clear();
    }

private:
    std::vector<float> return_start_;
    std::vector<float> hold_position_;
    float return_percent_ = 0.0f;
    bool returning_home_ = false;

public:
    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            return "Arm1FSMState";
        return state_name_;
    }
};

class Arm1FSMFactory : public FSMFactory
{
public:
    explicit Arm1FSMFactory(std::string initial) : initial_state_(std::move(initial)) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        auto *rl = static_cast<RL *>(context);
        if (state_name == "Arm1FSMState") return std::make_shared<Arm1FSMState>(rl);
        if (state_name == "Arm1FSMStatePassive") return std::make_shared<Arm1FSMStatePassive>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "arm1"; }
    std::vector<std::string> GetSupportedStates() const override { return {"Arm1FSMState", "Arm1FSMStatePassive"}; }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(Arm1FSMFactory, "Arm1FSMStatePassive")

inline std::string Arm1FSMState::CheckChange()
{
    if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
    {
        auto it = rl.fsm.states_.find("Arm1FSMStatePassive");
        if (it != rl.fsm.states_.end())
        {
            auto passive = std::dynamic_pointer_cast<Arm1FSMStatePassive>(it->second);
            if (passive && fsm_state) passive->BeginReturn(fsm_state->motor_state.q);
        }
        return "Arm1FSMStatePassive";
    }
    return state_name_;
}

#endif
