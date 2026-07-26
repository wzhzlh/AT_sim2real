#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <robot_msgs/msg/cmd.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace {

constexpr const char* kMapFrame = "map";
constexpr const char* kBaseFrame = "base_link";
constexpr const char* kJoint5Frame = "joint5";
constexpr const char* kRed = "\033[31m";
constexpr const char* kReset = "\033[0m";

struct Command {
    int line{-1};
    int column{-1};
};

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct Measurement {
    Pose2D base_link;
    Pose2D joint5;
};

struct RowKeys {
    const char* arm_key;
    const char* base_key;
};

void print_red(const std::string& message) {
    std::cerr << kRed << message << kReset << std::endl;
}

std::string default_point_yaml_path() {
    return ament_index_cpp::get_package_share_directory("task_game") + "/config/point.yaml";
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

Pose2D pose_from_transform(const geometry_msgs::msg::TransformStamped& transform) {
    return {
        transform.transform.translation.x,
        transform.transform.translation.y,
        yaw_from_quaternion(transform.transform.rotation),
    };
}

Command parse_command(const std::string& line) {
    Command command;
    std::istringstream stream(line);
    std::string token;

    while (stream >> token) {
        if (token == "-line") {
            if (!(stream >> command.line)) {
                throw std::runtime_error("-line 后缺少行号");
            }
        } else if (token == "-row") {
            if (!(stream >> command.column)) {
                throw std::runtime_error("-row 后缺少列号");
            }
        } else {
            throw std::runtime_error("未知参数: " + token);
        }
    }

    if (command.line < 1 || command.line > 3) {
        throw std::runtime_error("-line 必须是 1、2 或 3");
    }
    if (command.column < 1 || command.column > 4) {
        throw std::runtime_error("-row 必须是 1、2、3 或 4");
    }

    return command;
}

RowKeys keys_for_line(int line) {
    // 现场输入的 line 与配置文件中的行名保持固定映射。
    switch (line) {
        case 1:
            return {"arm_pick_line_1", "pick_line_1"};
        case 2:
            return {"arm_pick_line_0", "pick_line_0"};
        case 3:
            return {"arm_place", "place"};
        default:
            throw std::runtime_error("-line 必须是 1、2 或 3");
    }
}

YAML::Node make_point(std::initializer_list<double> values) {
    YAML::Node point(YAML::NodeType::Sequence);
    point.SetStyle(YAML::EmitterStyle::Flow);
    for (const double value : values) {
        point.push_back(value);
    }
    return point;
}

YAML::Node make_zero_point(size_t point_size) {
    YAML::Node point(YAML::NodeType::Sequence);
    point.SetStyle(YAML::EmitterStyle::Flow);
    for (size_t i = 0; i < point_size; ++i) {
        point.push_back(0.0);
    }
    return point;
}

void ensure_map(YAML::Node parent, const std::string& key) {
    if (!parent[key] || !parent[key].IsMap()) {
        parent[key] = YAML::Node(YAML::NodeType::Map);
    }
}

void ensure_point_row(YAML::Node parent, const std::string& key, size_t point_size) {
    // 每一行固定 4 列；缺失的列用零点补齐，避免乱序录点时越界。
    if (!parent[key] || !parent[key].IsSequence()) {
        parent[key] = YAML::Node(YAML::NodeType::Sequence);
    }

    YAML::Node row = parent[key];
    while (row.size() < 4) {
        row.push_back(make_zero_point(point_size));
    }

    for (size_t i = 0; i < row.size(); ++i) {
        if (!row[i] || !row[i].IsSequence() || row[i].size() != point_size) {
            row[i] = make_zero_point(point_size);
        } else {
            row[i].SetStyle(YAML::EmitterStyle::Flow);
        }
    }
}

void migrate_legacy_box_positions_key(YAML::Node root) {
    constexpr const char* kLegacyKey = "box_positions:box_positions";
    if (!root["box_positions"] && root[kLegacyKey]) {
        root["box_positions"] = root[kLegacyKey];
        root.remove(kLegacyKey);
    }
}

void write_measurement_to_yaml(const std::string& yaml_path, const Command& command, const Measurement& measurement) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root || !root.IsMap()) {
        throw std::runtime_error(yaml_path + " 不是有效的 YAML map");
    }

    migrate_legacy_box_positions_key(root);

    ensure_map(root, "arm_box_positions");
    ensure_map(root, "box_positions");

    const RowKeys keys = keys_for_line(command.line);
    YAML::Node arm_box_positions = root["arm_box_positions"];
    YAML::Node box_positions = root["box_positions"];
    ensure_point_row(arm_box_positions, keys.arm_key, 2);
    ensure_point_row(box_positions, keys.base_key, 3);

    // 用户输入列号为 1-4；YAML 数组下标为 0-3。
    const int column_index = command.column - 1;
    arm_box_positions[keys.arm_key][column_index] = make_point({measurement.joint5.x, measurement.joint5.y});
    box_positions[keys.base_key][column_index] =
        make_point({measurement.base_link.x, measurement.base_link.y, measurement.base_link.yaw});

    YAML::Emitter emitter;
    emitter.SetIndent(2);
    emitter << root;
    if (!emitter.good()) {
        throw std::runtime_error("生成 YAML 失败: " + std::string(emitter.GetLastError()));
    }

    std::ofstream output(yaml_path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        throw std::runtime_error("无法打开 YAML 文件写入: " + yaml_path);
    }
    output << emitter.c_str() << '\n';
}

void print_measurement(const Command& command, const Measurement& measurement) {
    const RowKeys keys = keys_for_line(command.line);
    const int column_index = command.column - 1;

    std::cout << std::fixed << std::setprecision(6)
              << "记录 line=" << command.line << " row=" << command.column << " (第 " << command.column << " 列)"
              << '\n'
              << "  base_link(map): x=" << measurement.base_link.x << ", y=" << measurement.base_link.y
              << ", yaw=" << measurement.base_link.yaw << '\n'
              << "  joint5(map):    x=" << measurement.joint5.x << ", y=" << measurement.joint5.y
              << ", yaw=" << measurement.joint5.yaw << '\n'
              << "  写入 arm_box_positions." << keys.arm_key << '[' << column_index << "] 和 box_positions."
              << keys.base_key << '[' << column_index << ']' << std::endl;
}

}  // namespace

class MeasurePoint {
public:
    MeasurePoint(rclcpp::Node::SharedPtr node);
    ~MeasurePoint();
    void run();

private:
    std::optional<Measurement> lookup_measurement(std::string& error_message);

    void ready();
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string point_yaml_path_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;        //发布机械臂动作命令，用于抬起机械臂到合适的位置
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;            //发布狗身动作命令，用于保持位控站立
};

MeasurePoint::MeasurePoint(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    node_->declare_parameter<std::string>("point_yaml_path", default_point_yaml_path());
    point_yaml_path_ = node_->get_parameter("point_yaml_path").as_string();

    arm_cmd_pub_ = node_->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    RCLCPP_INFO(node_->get_logger(), "point.yaml 路径: %s", point_yaml_path_.c_str());

    ready();
}

MeasurePoint::~MeasurePoint() = default;

void MeasurePoint::ready()  
{
    //arm_cmd_pub_发送机械臂命令6展开机械臂
    //cmd_pub_发送mode=1让狗子处于站立位控
    std_msgs::msg::Int32 arm_msg;
    arm_msg.data = 6;       //发送机械臂抬起的指令，准备录制

    robot_msgs::msg::Cmd dog_msg;
    dog_msg.mode = 1;

    for (int i = 0; rclcpp::ok() && i < 20; ++i) {
        arm_cmd_pub_->publish(arm_msg);
        cmd_pub_->publish(dog_msg);
        std::this_thread::sleep_for(50ms);
    }

    RCLCPP_INFO(node_->get_logger(), "已发送录点准备命令: arm_cmd.mode=6, robot_move_cmd.mode=1");
}

std::optional<Measurement> MeasurePoint::lookup_measurement(std::string& error_message) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    std::string last_error;

    // 两个 TF 必须在同一次记录中都成功；任一失败都会继续重试，最长 5 秒。
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        try {
            const auto base_transform =
                tf_buffer_->lookupTransform(kMapFrame, kBaseFrame, tf2::TimePointZero, tf2::durationFromSec(0.05));
            const auto joint5_transform =
                tf_buffer_->lookupTransform(kMapFrame, kJoint5Frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
            return Measurement{pose_from_transform(base_transform), pose_from_transform(joint5_transform)};
        } catch (const tf2::TransformException& ex) {
            last_error = ex.what();
            std::this_thread::sleep_for(100ms);
        }
    }

    if (last_error.empty()) {
        last_error = "ROS 已退出或等待超时";
    }
    error_message = "5s 内获取 TF 失败: " + last_error;
    return std::nullopt;
}

void MeasurePoint::run() {
    std::cout << "请输入: -line <1|2|3> -row <1|2|3|4>" << std::endl;

    std::string input;
    while (rclcpp::ok() && std::getline(std::cin, input)) {
        if (input.empty()) {
            continue;
        }

        try {
            const Command command = parse_command(input);
            std::cout << "收到命令: line=" << command.line << ", row=" << command.column
                      << "，正在等待 TF map->base_link 和 map->joint5（最多 5s）..." << std::endl;

            std::string error_message;
            const auto measurement = lookup_measurement(error_message);
            if (!measurement) {
                print_red(error_message);
                continue;
            }

            print_measurement(command, *measurement);
            write_measurement_to_yaml(point_yaml_path_, command, *measurement);
            std::cout << "已覆盖写入: " << point_yaml_path_ << std::endl;
        } catch (const std::exception& ex) {
            print_red(std::string("异常: ") + ex.what());
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;

    MeasurePoint measure_point(std::make_shared<rclcpp::Node>("measure_node"));
    measure_point.run();
    rclcpp::shutdown();
    return 0;
}
