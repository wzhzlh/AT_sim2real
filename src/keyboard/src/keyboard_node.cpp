#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {

constexpr uint32_t bit(int index)
{
    return 1u << index;
}

class TerminalRawMode {
public:
    TerminalRawMode()
    {
        if (!isatty(STDIN_FILENO)) {
            return;
        }

        if (tcgetattr(STDIN_FILENO, &original_) != 0) {
            return;
        }

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            active_ = true;
        }
    }

    ~TerminalRawMode()
    {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
        }
    }

    bool active() const
    {
        return active_;
    }

private:
    termios original_{};
    bool active_{false};
};

}  // namespace

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode()
        : Node("keyboard_node")
    {
        linear_value_ = static_cast<float>(declare_parameter<double>("linear_value", 1200.0));
        angular_value_ = static_cast<float>(declare_parameter<double>("angular_value", 1200.0));
        command_timeout_ = std::chrono::duration<double>(
            declare_parameter<double>("command_timeout", 0.5));
        pulse_duration_ = std::chrono::duration<double>(
            declare_parameter<double>("key_pulse_duration", 0.2));
        const auto publish_rate = declare_parameter<double>("publish_rate", 50.0);

        cmd_pub_ = create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

        print_help();

        input_thread_ = std::thread([this]() { input_loop(); });

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_remote(); });
    }

    ~KeyboardNode() override
    {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    struct Axis {
        float value{0.0f};
        std::chrono::steady_clock::time_point updated{};
    };

    void print_help()
    {
        RCLCPP_INFO(get_logger(), "Keyboard remote publisher started.");
        RCLCPP_INFO(get_logger(), "Move: w/s forward/back, a/d left/right, q/e spin.");
        RCLCPP_INFO(get_logger(), "Policy keys: 1 walk, 2 stairs.");
        RCLCPP_INFO(get_logger(), "Use Ctrl-C to exit.");
    }

    void input_loop()
    {
        TerminalRawMode terminal;
        if (!terminal.active()) {
            RCLCPP_WARN(get_logger(), "stdin is not a TTY; keyboard input may be unavailable.");
        }

        pollfd fd{};
        fd.fd = STDIN_FILENO;
        fd.events = POLLIN;

        while (running_ && rclcpp::ok()) {
            const int ready = poll(&fd, 1, 50);
            if (ready <= 0 || !(fd.revents & POLLIN)) {
                continue;
            }

            char ch = 0;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                handle_key(ch);
            }
        }
    }

    void handle_key(char ch)
    {
        const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        const auto now = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(mutex_);

        switch (key) {
            case 'w':
                ly_axis_ = Axis{linear_value_, now};
                break;
            case 's':
                ly_axis_ = Axis{-linear_value_, now};
                break;
            case 'a':
                lx_axis_ = Axis{-linear_value_, now};
                break;
            case 'd':
                lx_axis_ = Axis{linear_value_, now};
                break;
            case 'q':
                rx_axis_ = Axis{-angular_value_, now};
                break;
            case 'e':
                rx_axis_ = Axis{angular_value_, now};
                break;
            case ' ':
                lx_axis_ = Axis{};
                ly_axis_ = Axis{};
                rx_axis_ = Axis{};
                break;
            case '1':
                mode_ = 2;  // walk policy
                RCLCPP_INFO(get_logger(), "Keyboard policy: walk (mode 2)");
                break;
            case '2':
                mode_ = 3;  // stairs policy
                RCLCPP_INFO(get_logger(), "Keyboard policy: stairs (mode 3)");
                break;
            default:
                break;
        }
    }

    void publish_remote()
    {
        robot_msgs::msg::Cmd msg;
        const auto now = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg.mode = mode_;
            msg.vx = axis_value(ly_axis_, now) / 1000.0f;
            msg.vy = axis_value(lx_axis_, now) / 1000.0f;
            msg.vz = axis_value(rx_axis_, now) / 1000.0f;
            msg.wheel_vel = 0.0f;
        }
        cmd_pub_->publish(msg);
    }

    float axis_value(const Axis& axis, std::chrono::steady_clock::time_point now) const
    {
        if (axis.updated.time_since_epoch().count() == 0) {
            return 0.0f;
        }
        if (now - axis.updated > command_timeout_) {
            return 0.0f;
        }
        return axis.value;
    }

    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::thread input_thread_;
    std::atomic_bool running_{true};
    std::mutex mutex_;

    float linear_value_{1200.0f};
    float angular_value_{1200.0f};
    std::chrono::duration<double> command_timeout_{0.5};
    std::chrono::duration<double> pulse_duration_{0.2};

    Axis lx_axis_;
    Axis ly_axis_;
    Axis rx_axis_;
    int32_t mode_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
