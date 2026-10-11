#include "rl_real_arm1.hpp"
#include "fsm_arm1.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <iterator>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <vector>

#if defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif

namespace
{
constexpr size_t kJointCount = 5;
constexpr size_t kPacketJointCount = 6;
constexpr auto kTransmitPeriod = std::chrono::milliseconds(20);
constexpr std::array<float, kJointCount> kJointLower{-1.8f, -1.5707963f, -2.9f, -2.0f, -0.610865f};
constexpr std::array<float, kJointCount> kJointUpper{1.8f, 1.5707963f, 2.9f, 1.8f, 0.610865f};

#pragma pack(push, 1)
struct ArmMotorPacket
{
    float rad;
    float omega;
    float torque;
};

struct ArmTargetPacket
{
    int32_t pack_type;
    ArmMotorPacket joints[kPacketJointCount];
    uint8_t air_pump;
    uint8_t worked;
};

struct ArmStatePacket
{
    int32_t pack_type;
    ArmMotorPacket joints[kPacketJointCount];
    uint8_t message;
};
#pragma pack(pop)

static_assert(sizeof(ArmTargetPacket) == 78, "ArmTarget_t packet layout changed");
static_assert(sizeof(ArmStatePacket) == 77, "ArmState_t packet layout changed");
}

class Arm1CDC
{
public:
    explicit Arm1CDC(const std::string &device)
    {
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
            throw std::runtime_error("Cannot open USB CDC device " + device + ": " + std::strerror(errno));

        termios options{};
        if (tcgetattr(fd_, &options) == 0)
        {
            options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            options.c_oflag &= ~OPOST;
            options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            options.c_cflag &= ~(CSIZE | PARENB);
            options.c_cflag |= CS8;
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            options.c_cflag |= CLOCAL | CREAD;
            options.c_cflag &= ~CSTOPB;
            options.c_cflag &= ~CRTSCTS;
            tcsetattr(fd_, TCSANOW, &options);
        }
        tcflush(fd_, TCIOFLUSH);
        reader_ = std::thread(&Arm1CDC::ReadLoop, this);
        std::cout << LOGGER::INFO << "USB CDC connected: " << device << std::endl;
    }

    ~Arm1CDC()
    {
        running_.store(false);
        if (reader_.joinable()) reader_.join();
        if (fd_ >= 0) ::close(fd_);
    }

    bool GetState(ArmStatePacket &out)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!state_seen_ || std::chrono::steady_clock::now() - last_state_rx_ > std::chrono::milliseconds(100)) return false;
        out = state_;
        return true;
    }

    void Send(const ArmTargetPacket &target)
    {
        const auto *data = reinterpret_cast<const uint8_t *>(&target);
        size_t sent = 0;
        while (sent < sizeof(target))
        {
            const ssize_t count = ::write(fd_, data + sent, sizeof(target) - sent);
            if (count > 0)
            {
                sent += static_cast<size_t>(count);
            }
            else if (count < 0 && errno == EINTR)
            {
                continue;
            }
            else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                pollfd pfd{fd_, POLLOUT, 0};
                if (::poll(&pfd, 1, 100) <= 0) break;
            }
            else
            {
                throw std::runtime_error("USB CDC write failed: " + std::string(std::strerror(errno)));
            }
        }
        if (sent != sizeof(target)) throw std::runtime_error("USB CDC write timed out before sending a full command packet");
    }

private:
    void Consume(const uint8_t *data, size_t size)
    {
        pending_.insert(pending_.end(), data, data + size);
        constexpr uint8_t header[] = {1, 0, 0, 0};
        while (pending_.size() >= sizeof(ArmStatePacket))
        {
            auto it = std::search(pending_.begin(), pending_.end(), std::begin(header), std::end(header));
            if (it == pending_.end())
            {
                if (pending_.size() > 3) pending_.erase(pending_.begin(), pending_.end() - 3);
                return;
            }
            if (it != pending_.begin()) pending_.erase(pending_.begin(), it);
            if (pending_.size() < sizeof(ArmStatePacket)) return;

            ArmStatePacket packet{};
            std::memcpy(&packet, pending_.data(), sizeof(packet));
            if (packet.pack_type != 1)
            {
                pending_.erase(pending_.begin());
                continue;
            }
            bool valid = true;
            for (size_t i = 0; i < kJointCount; ++i)
                valid = valid && std::isfinite(packet.joints[i].rad) && std::isfinite(packet.joints[i].omega) &&
                        std::isfinite(packet.joints[i].torque);
            if (!valid)
            {
                pending_.erase(pending_.begin());
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_ = packet;
                state_seen_ = true;
                last_state_rx_ = std::chrono::steady_clock::now();
            }
            pending_.erase(pending_.begin(), pending_.begin() + sizeof(packet));
        }
    }

    void ReadLoop()
    {
        uint8_t buffer[256];
        while (running_.load())
        {
            pollfd pfd{fd_, POLLIN, 0};
            const int ready = ::poll(&pfd, 1, 100);
            if (ready <= 0) continue;
            const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
            if (count > 0) Consume(buffer, static_cast<size_t>(count));
            else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
                std::cerr << LOGGER::ERROR << "USB CDC read failed: " << std::strerror(errno) << std::endl;
                running_.store(false);
            }
        }
    }

    int fd_ = -1;
    std::atomic<bool> running_{true};
    std::thread reader_;
    std::mutex state_mutex_;
    ArmStatePacket state_{};
    bool state_seen_ = false;
    std::chrono::steady_clock::time_point last_state_rx_{};
    std::vector<uint8_t> pending_;
};

RL_RealArm1::RL_RealArm1(int argc, char **argv, const std::string &device_override)
{
    this->robot_name = "arm1";
    this->ReadYaml("arm1", "base.yaml");
    this->InitJointNum(kJointCount);

    const char *env_device = std::getenv("ARM1_CDC_DEVICE");
    const std::string device = !device_override.empty()
        ? device_override
        : (argc > 1 ? argv[1] : (env_device ? env_device : "/dev/ttyACM0"));
    cdc_ = std::make_unique<Arm1CDC>(device);

    this->fsm = *FSMManager::GetInstance().CreateFSM("arm1", this);
    loop_keyboard_ = std::make_shared<LoopFunc>("arm1_keyboard", 0.05, std::bind(&RL_RealArm1::KeyboardInterface, this));
    loop_control_ = std::make_shared<LoopFunc>("arm1_control", this->params.Get<float>("dt"), std::bind(&RL_RealArm1::RobotControl, this));
    loop_rl_ = std::make_shared<LoopFunc>("arm1_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_RealArm1::RunModel, this));
    loop_keyboard_->start();
    loop_control_->start();
    loop_rl_->start();
}

RL_RealArm1::~RL_RealArm1()
{
    if (loop_keyboard_) loop_keyboard_->shutdown();
    if (loop_control_) loop_control_->shutdown();
    if (loop_rl_) loop_rl_->shutdown();
}

void RL_RealArm1::GetState(RobotState<float> *state)
{
    if (!state || !cdc_) return;
    ArmStatePacket packet{};
    if (!cdc_->GetState(packet)) return;
    for (size_t i = 0; i < kJointCount; ++i)
    {
        state->motor_state.q[i] = packet.joints[i].rad;
        state->motor_state.dq[i] = packet.joints[i].omega;
        state->motor_state.tau_est[i] = packet.joints[i].torque;
    }
    state->imu.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    state->imu.gyroscope = {0.0f, 0.0f, 0.0f};
}

void RL_RealArm1::SetCommand(const RobotCommand<float> *command)
{
    if (!command || !cdc_) return;
    ArmStatePacket state{};
    if (!cdc_->GetState(state)) return;
    const auto now = std::chrono::steady_clock::now();
    if (last_tx_.time_since_epoch().count() != 0 && now - last_tx_ < kTransmitPeriod) return;

    ArmTargetPacket target{};
    target.pack_type = 1;
    target.air_pump = 0;
    target.worked = 0;
    for (size_t i = 0; i < kJointCount; ++i)
    {
        const float desired_q = std::isfinite(command->motor_command.q[i])
            ? command->motor_command.q[i]
            : state.joints[i].rad;
        target.joints[i].rad = std::clamp(desired_q, kJointLower[i], kJointUpper[i]);
        target.joints[i].omega = std::isfinite(command->motor_command.dq[i])
            ? std::clamp(command->motor_command.dq[i], -10.0f, 10.0f)
            : 0.0f;
        target.joints[i].torque = 0.0f;
    }
    try
    {
        cdc_->Send(target);
        last_tx_ = now;
    }
    catch (const std::exception &e)
    {
        std::cerr << LOGGER::ERROR << e.what() << std::endl;
    }
}

void RL_RealArm1::RobotControl()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
    }
    this->SetCommand(&this->robot_command);
}

void RL_RealArm1::RunModel()
{
    if (!this->rl_init_done) return;
    this->episode_length_buf++;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;
    }
    this->obs.actions = this->Forward();
    this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
    output_dof_pos_queue.push(this->output_dof_pos);
    output_dof_vel_queue.push(this->output_dof_vel);
    output_dof_tau_queue.push(this->output_dof_tau);
}

std::vector<float> RL_RealArm1::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
    if (!lock.owns_lock()) return this->obs.actions;
    const auto clamped_obs = this->ComputeObservation();
    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }
    const auto lower = this->params.Get<std::vector<float>>("clip_actions_lower");
    const auto upper = this->params.Get<std::vector<float>>("clip_actions_upper");
    if (!lower.empty() && !upper.empty()) actions = clamp(actions, lower, upper);
    return actions;
}

int main(int argc, char **argv)
{
#if defined(USE_ROS2)
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<rclcpp::Node>("arm1_rl_deployment");
        const std::string device = node->declare_parameter<std::string>("device", "/dev/ttyACM0");
        RL_RealArm1 arm(argc, argv, device);
        RCLCPP_INFO(node->get_logger(), "Arm1 RL node ready on %s; press 1 for policy and 0 to return to hold pose.", device.c_str());
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << LOGGER::ERROR << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
#else
    try
    {
        RL_RealArm1 arm(argc, argv);
        std::cout << LOGGER::INFO << "Arm1 ready: press 1 for policy, 0 to return to hold pose." << std::endl;
        while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    catch (const std::exception &e)
    {
        std::cerr << LOGGER::ERROR << e.what() << std::endl;
        return 1;
    }
#endif
    return 0;
}
