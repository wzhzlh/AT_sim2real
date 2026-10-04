#include <chrono>
#include <cmath>
#include <core/robot.hpp>
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>

using namespace std::chrono_literals;

namespace {

std::string trim_copy(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> split_scene_paths(const std::string& scene_path)
{
    std::vector<std::string> paths;
    std::stringstream ss(scene_path);
    std::string item;
    while (std::getline(ss, item, ';')) {
        item = trim_copy(item);
        if (!item.empty()) {
            paths.push_back(item);
        }
    }
    return paths;
}

std::vector<std::string> select_scene_yamls(rclcpp::Node::SharedPtr node, const std::string& scene_path, const std::string& trajectory_dir_path)
{
    if (!scene_path.empty()) {
        auto paths = split_scene_paths(scene_path);
        RCLCPP_INFO(node->get_logger(), "使用参数指定路径文件数量: %zu", paths.size());
        for (std::size_t i = 0; i < paths.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "路径%zu: %s", i, paths[i].c_str());
        }
        return paths;
    }

    const auto trajectory_dir = trajectory_dir_path.empty() ? std::filesystem::path("./trajectory") : std::filesystem::path(trajectory_dir_path);
    const auto preferred_yaml = trajectory_dir / "obstacle_game.yaml";
    if (std::filesystem::exists(preferred_yaml)) {
        RCLCPP_INFO(node->get_logger(), "使用优先路径文件: %s", preferred_yaml.string().c_str());
        return {preferred_yaml.string()};
    }

    RCLCPP_WARN(node->get_logger(), "未找到路径文件: %s，自动轨迹为空", preferred_yaml.string().c_str());
    return {""};
}

}  // namespace


Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
    node_->declare_parameter<std::string>("scene_path","");
    node_->declare_parameter<std::string>("trajectory_dir","./trajectory");
    node_->declare_parameter<int>("switch_path", 0);
    node_->declare_parameter<bool>("stop_target", false);
    node_->declare_parameter<bool>("start_target", false);
    node_->declare_parameter<bool>("stand", false);
    node_->declare_parameter<bool>("begin_game", false);

    const auto yaml_paths = select_scene_yamls(
        node_,
        node_->get_parameter("scene_path").as_string(),
        node_->get_parameter("trajectory_dir").as_string());

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    pilots_.reserve(yaml_paths.size());
    for (const auto& yaml_path : yaml_paths) {
        pilots_.push_back(std::make_shared<Pilot>(node_, yaml_path));
    }
    if (pilots_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "未加载任何轨迹Pilot，自动控制不可用");
    }

    policy_done_sub_ = node_->create_subscription<robot_msgs::msg::Int>("policy_done", 10, [this](const robot_msgs::msg::Int& msg) {
        auto pilot = active_pilot();
        if (pilot) {
            pilot->notify_policy_done(msg.data);
        }

    });

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        // TODO:处理并发布遥控器数据
        if (!check_key_pressed(msg.key,1)) // 拨杆处于中间位置或向下位置，即手动控制
        {
            if (current_control_mode == 1 && ++manual_switch_request_count_ >= kManualSwitchDebounceFrames) {
                cmd.mode = 1;    // 如果刚才是自动控制，那么切入手动控制时进入位控站立模式(可能是有紧急情况)
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->reset();
                    pilot->stop();
                }
                current_control_mode = 0;
                manual_switch_request_count_ = 0;
                RCLCPP_INFO(node_->get_logger(), "请求切入手动控制");
            }
        }
        else {      //拨杆处于向上位置，切入自动模式
            manual_switch_request_count_ = 0;
            if(current_control_mode==0)
            {
                current_control_mode = 1;
                RCLCPP_INFO(node_->get_logger(), "请求切入自动控制");
            }
        }

        // 只有手动控制模式下可以使用遥控器修改机器人当前使用的策略
        if (current_control_mode == 0) {
            if (check_key_trigger(msg.key,4)) {         //模式控制
                cmd.mode = 1; // 位控站立
                RCLCPP_INFO(node_->get_logger(), "getup/位控站立模式");
            } else if (check_key_trigger(msg.key,5)) {
                cmd.mode = 2; // 普通行走策略
                RCLCPP_INFO(node_->get_logger(), "walk/普通行走模式");
            } else if (check_key_trigger(msg.key,6)) {
                cmd.mode = 3; // 台阶策略
                RCLCPP_INFO(node_->get_logger(), "stairs/台阶模式");
            } else if (check_key_trigger(msg.key,3)) {
                cmd.mode = 4; // 沙地策略
                RCLCPP_INFO(node_->get_logger(), "sand/沙地模式");
            } else if (check_key_trigger(msg.key,11)) {
                cmd.mode = 5; // 斜坡策略
                RCLCPP_INFO(node_->get_logger(), "slope/斜坡模式");
            } else if (check_key_trigger(msg.key,12)) {
                cmd.mode = 6; // 限高杆策略
                RCLCPP_INFO(node_->get_logger(), "bar/限高杆模式");
            } else if (check_key_trigger(msg.key,13)) {
                cmd.mode = 7; // 木桥策略
                RCLCPP_INFO(node_->get_logger(), "bridge/木桥模式");
            }

            //摇杆赋值
            cmd.vy=-(float)std::clamp((double)msg.lx/1200.0,-0.8,0.8);
            cmd.vx=(float)std::clamp((double)msg.ly/800.0,-1.8,1.8);
            cmd.vz=-(float)std::clamp((double)msg.rx/1000.0,-1.5,1.5);


        } else if (current_control_mode == 1) {
            if (check_key_trigger(msg.key,4)) {     //复位并停止
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->reset();
                    pilot->stop();
                }
            } else if (check_key_trigger(msg.key, 5)) {
                if (!refresh_robot_pose()) {
                    RCLCPP_ERROR(node_->get_logger(), "自动轨迹启动失败，尚未获取有效map->base_link位姿");
                } else {
                    auto pilot = active_pilot();
                    if (!pilot || !pilot->start()) {  // 开始执行自动控制
                        RCLCPP_ERROR(node_->get_logger(), "自动轨迹启动失败，请检查scene_path和路径点");
                    }
                }
            } else if (check_key_trigger(msg.key, 6)) {
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->stop();         //  自动控制执行暂停
                }
            }
        }

        if (current_control_mode == 0) {
            manual_switch_request_count_ = 0;
        }

        save_key_state(msg.key);
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            const auto& name = param.get_name();
            if (name == "switch_path") {
                const int path_id = param.as_int();
                if (path_id < 0 || static_cast<std::size_t>(path_id) >= pilots_.size()) {
                    RCLCPP_ERROR(node_->get_logger(), "switch_path=%d超过轨迹数量%zu，忽略本次切换", path_id, pilots_.size());
                    continue;
                }
                switch_to_path(path_id);
            } else if (name == "stop_target") {
                if (!param.as_bool()) {
                    continue;
                }
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->stop();
                    RCLCPP_INFO(node_->get_logger(), "stop_target触发，已暂停当前轨迹");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "stop_target触发失败，当前无有效轨迹");
                }
                stop_target_clear_pending_.store(true);
            } else if (name == "start_target") {
                if (!param.as_bool()) {
                    continue;
                }
                auto pilot = active_pilot();
                if (pilot && pilot->start()) {
                    RCLCPP_INFO(node_->get_logger(), "start_target触发，已继续当前轨迹");
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "start_target触发失败，请检查当前轨迹");
                }
                start_target_clear_pending_.store(true);
            } else if (name == "stand") {
                if (!param.as_bool()) {
                    continue;
                }
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->reset();
                    pilot->stop();
                }
                current_control_mode = 0;
                manual_switch_request_count_ = 0;
                cmd.mode = 1;
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = 0.0f;
                RCLCPP_INFO(node_->get_logger(), "stand触发，已切入位控站立模式");
                stand_clear_pending_.store(true);
            } else if (name == "begin_game") {
                if (!param.as_bool()) {
                    continue;
                }
                if (!refresh_robot_pose()) {
                    RCLCPP_ERROR(node_->get_logger(), "begin_game触发失败，尚未获取有效map->base_link位姿");
                } else {
                    auto pilot = active_pilot();
                    if (pilot && pilot->start()) {
                        current_control_mode = 1;
                        manual_switch_request_count_ = 0;
                        RCLCPP_INFO(node_->get_logger(), "begin_game触发，已切入自动控制并开始轨迹");
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "begin_game触发失败，请检查scene_path和路径点");
                    }
                }
                begin_game_clear_pending_.store(true);
            }
        }
        return result;
    });

    control_timer = node_->create_wall_timer(50ms, [this]() {
        if (stop_target_clear_pending_.exchange(false)) {
            node_->set_parameter(rclcpp::Parameter("stop_target", false));
        }
        if (start_target_clear_pending_.exchange(false)) {
            node_->set_parameter(rclcpp::Parameter("start_target", false));
        }
        if (stand_clear_pending_.exchange(false)) {
            node_->set_parameter(rclcpp::Parameter("stand", false));
        }
        if (begin_game_clear_pending_.exchange(false)) {
            node_->set_parameter(rclcpp::Parameter("begin_game", false));
        }

        geometry_msgs::msg::TransformStamped transfer;
        if (!refresh_robot_pose()) {
            if (current_control_mode == 1) {
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->stop();
                }
                cmd.mode = 1;
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = 0.0f;
                current_control_mode = 0;
            }
            cmd_pub_->publish(cmd);
            return;
        }

        transfer = robot_pos_transfer;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "pos=(%lf,%lf)",
            transfer.transform.translation.x,
            transfer.transform.translation.y);

        if (current_control_mode == 1) {
            // geometry_msgs::msg::TransformStamped transfer;
            // try {
            //     transfer = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.05));
            //     robot_pos_transfer=transfer;
            // } catch (const tf2::TransformException& ex) {
            //     RCLCPP_WARN(node_->get_logger(), "获取目标 TF 失败，自动驾驶仪停止运行: %s", ex.what());
            //     current_control_mode = 0;
            //     return;
            // }

            auto pilot = active_pilot();
            if (pilot) {
                if (active_path_id_ != 0 && pilot->get_target_id() < 0) {
                    if (switch_to_path(0)) {
                        node_->set_parameter(rclcpp::Parameter("switch_path", 0));
                        pilot = active_pilot();
                    }
                }
                if (pilot) {
                    cmd = pilot->get_command(std::chrono::high_resolution_clock::now());
                }
            } else {
                cmd.mode = 1;
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = 0.0f;
            }
        }
        cmd_pub_->publish(cmd);
    });
}

bool Robot::refresh_robot_pose()
{
    try {
        const auto transfer = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_pos_transfer = transfer;
        robot_pose_valid_ = sync_pilot_state_from_transform(transfer);
        if (!robot_pose_valid_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "map->base_link位姿数值无效");
        }
        return robot_pose_valid_;
    } catch (const tf2::TransformException& ex) {
        robot_pose_valid_ = false;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "获取目标 TF 失败: %s", ex.what());
        return false;
    }
}

bool Robot::sync_pilot_state_from_transform(const geometry_msgs::msg::TransformStamped& transfer)
{
    const auto& translation = transfer.transform.translation;
    const auto& rotation = transfer.transform.rotation;

    if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
        !std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
        !std::isfinite(rotation.z) || !std::isfinite(rotation.w)) {
        return false;
    }

    tf2::Quaternion q;
    q.setW(rotation.w);
    q.setX(rotation.x);
    q.setY(rotation.y);
    q.setZ(rotation.z);
    if (q.length2() <= 1e-12) {
        return false;
    }
    q.normalize();

    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

    if (!std::isfinite(cur_yaw)) {
        return false;
    }

    for (const auto& pilot : pilots_) {
        pilot->set_state(Eigen::Vector2d(translation.x, translation.y), cur_yaw);
    }
    return true;
}

std::shared_ptr<Pilot> Robot::active_pilot() const
{
    if (active_path_id_ < 0 || static_cast<std::size_t>(active_path_id_) >= pilots_.size()) {
        return nullptr;
    }
    return pilots_[active_path_id_];
}

bool Robot::switch_to_path(int path_id)
{
    if (path_id < 0 || static_cast<std::size_t>(path_id) >= pilots_.size()) {
        RCLCPP_ERROR(node_->get_logger(), "目标轨迹编号越界: path_id=%d, paths=%zu", path_id, pilots_.size());
        return false;
    }

    if (path_id == active_path_id_) {
        return true;
    }

    const int previous_path_id = active_path_id_;
    auto previous_pilot = active_pilot();
    if (previous_pilot) {
        previous_pilot->stop();
    }

    if (path_id != 0 && !pilots_.empty() && previous_path_id != 0) {
        pilots_[0]->stop();
    }

    auto target_pilot = pilots_[path_id];
    if (!target_pilot->start()) {
        RCLCPP_ERROR(node_->get_logger(), "轨迹%d启动失败，保持当前轨迹%d", path_id, active_path_id_);
        return false;
    }

    active_path_id_ = path_id;
    RCLCPP_INFO(node_->get_logger(), "切换执行轨迹: %d -> %d", previous_path_id, active_path_id_);
    return true;
}

bool Robot::check_key_trigger(uint32_t current_key,int index)
{
    bool current_is_true=((current_key>>index)&0x0001);
    bool last_is_false=!((last_key>>index)&0x0001);
    return current_is_true&&last_is_false;
}

bool Robot::check_key_pressed(uint32_t current_key,int index)
{
    return ((current_key>>index)&0x0001);
}

void Robot::save_key_state(uint32_t current_key)
{
    last_key=current_key;
}
