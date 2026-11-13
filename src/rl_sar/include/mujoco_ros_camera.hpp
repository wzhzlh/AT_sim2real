#pragma once

// Render on the viewer thread: it owns the current OpenGL context.
#include <algorithm>
#include <array>
#include <atomic>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mujoco/mujoco.h>

class MujocoRosCamera
{
public:
    explicit MujocoRosCamera(const mjModel* model)
    {
        node_ = std::make_shared<rclcpp::Node>("mujoco_camera");
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            "/mujoco/camera/image_raw", rclcpp::SensorDataQoS());
        info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/mujoco/camera/camera_info", rclcpp::SensorDataQoS());
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mujoco/base_pose", rclcpp::SensorDataQoS());
        base_id_ = mj_name2id(model, mjOBJ_BODY, "base");
        if (base_id_ < 0) {base_id_ = mj_name2id(model, mjOBJ_BODY, "base_link");}
        cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/mujoco/cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                command_ = {msg->linear.x, msg->linear.y, msg->angular.z};
                command_time_ = std::chrono::steady_clock::now();
            });
        stand_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/mujoco/stand", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                stand_requested_ = msg->data;
                stand_time_ = std::chrono::steady_clock::now();
                have_stand_ = true;
            });
        // Optional scene manipulation for moving-tag simulation experiments.
        tag_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mujoco/tag_pose", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                if (msg->header.frame_id != "world") {return;}
                pending_tag_pose_ = msg;
            });
        tag_id_ = mj_name2id(model, mjOBJ_BODY, "apriltag_5");
        start_client_ = node_->create_client<std_srvs::srv::Trigger>("/tag_approach/start");
        stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/tag_approach/stop");
        mjv_defaultScene(&scene_);
        mjv_defaultCamera(&camera_);
        camera_.type = mjCAMERA_FIXED;
        camera_.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "tag_camera");
        mjv_defaultOption(&option_);
        if (camera_.fixedcamid < 0) {
            throw std::runtime_error("Scene has no tag_camera; use scene_tag");
        }
        mjv_makeScene(model, &scene_, 10000);
        RCLCPP_INFO(node_->get_logger(), "MuJoCo ready: 0 = stand, 1 = RL automatic tag tracking");
    }

    ~MujocoRosCamera() { mjv_freeScene(&scene_); }

    void requestTracking(bool enabled)
    {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            // Hold position until fresh detector commands arrive after start.
            command_ = {0.0, 0.0, 0.0};
            stand_requested_ = true;
            have_stand_ = true;
        }
        pending_tracking_.store(enabled ? 1 : 0);
    }

    std::array<float, 3> command()
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (std::chrono::steady_clock::now() - command_time_ > std::chrono::milliseconds(500)) {
            return {0.f, 0.f, 0.f};
        }
        return {static_cast<float>(command_[0]), static_cast<float>(command_[1]),
                static_cast<float>(command_[2])};
    }

    // -1: no mode received yet, 0: locomotion, 1: position-controlled stand.
    int standMode()
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (!have_stand_) {return -1;}
        const auto now = std::chrono::steady_clock::now();
        return stand_requested_ || now - stand_time_ > std::chrono::milliseconds(500) ||
            now - command_time_ > std::chrono::milliseconds(500) ? 1 : 0;
    }

    void render(const mjModel* model, mjData* data, mjrContext& context,
                std::recursive_mutex& physics_mutex)
    {
        if (!rclcpp::ok()) {return;}
        rclcpp::spin_some(node_);
        const int request = pending_tracking_.load();
        if (request >= 0) {
            auto client = request ? start_client_ : stop_client_;
            if (client->service_is_ready()) {
                int expected = request;
                if (pending_tracking_.compare_exchange_strong(expected, -1)) {
                    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_frame_ < std::chrono::milliseconds(33)) {return;}
        last_frame_ = now;
        constexpr int width = 640, height = 480;
        const mjrRect viewport{0, 0, width, height};
        geometry_msgs::msg::PoseStamped pose;
        {
            // Copy the render scene under the physics lock. GPU readback and ROS
            // publication must not block the 200 Hz motor control loop.
            std::lock_guard<std::recursive_mutex> lock(physics_mutex);
            if (pending_tag_pose_ && tag_id_ >= 0 && model->body_mocapid[tag_id_] >= 0) {
                const int id = model->body_mocapid[tag_id_];
                const auto& pose = pending_tag_pose_->pose;
                const double norm = std::sqrt(pose.orientation.w * pose.orientation.w +
                    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                    pose.orientation.z * pose.orientation.z);
                if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
                    std::isfinite(pose.position.z) && std::isfinite(norm) && norm > 1e-6) {
                    data->mocap_pos[3*id] = pose.position.x;
                    data->mocap_pos[3*id+1] = pose.position.y;
                    data->mocap_pos[3*id+2] = pose.position.z;
                    data->mocap_quat[4*id] = pose.orientation.w / norm;
                    data->mocap_quat[4*id+1] = pose.orientation.x / norm;
                    data->mocap_quat[4*id+2] = pose.orientation.y / norm;
                    data->mocap_quat[4*id+3] = pose.orientation.z / norm;
                    mj_forward(model, data);
                }
                pending_tag_pose_.reset();
            }
            mjv_updateScene(model, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
            if (base_id_ >= 0) {
                const auto* position = data->xpos + 3 * base_id_;
                const auto* quaternion = data->xquat + 4 * base_id_;
                pose.pose.position.x = position[0];
                pose.pose.position.y = position[1];
                pose.pose.position.z = position[2];
                pose.pose.orientation.w = quaternion[0];
                pose.pose.orientation.x = quaternion[1];
                pose.pose.orientation.y = quaternion[2];
                pose.pose.orientation.z = quaternion[3];
            }
        }
        const int previous_buffer = context.currentBuffer;
        mjr_setBuffer(mjFB_OFFSCREEN, &context);
        mjr_render(viewport, &scene_, &context);
        std::vector<unsigned char> pixels(width * height * 3);
        mjr_readPixels(pixels.data(), nullptr, viewport, &context);
        mjr_setBuffer(previous_buffer, &context);

        sensor_msgs::msg::Image image;
        image.header.stamp = node_->now();
        image.header.frame_id = "mujoco_camera_optical_frame";
        image.width = width;
        image.height = height;
        image.encoding = "rgb8";
        image.step = width * 3;
        image.data.resize(pixels.size());
        for (int row = 0; row < height; ++row) {
            std::copy_n(pixels.data() + (height - row - 1) * image.step,
                        image.step, image.data.data() + row * image.step);
        }
        sensor_msgs::msg::CameraInfo info;
        info.header = image.header;
        info.width = width;
        info.height = height;
        const double focal = height / (2.0 * std::tan(
            model->cam_fovy[camera_.fixedcamid] * 3.141592653589793 / 360.0));
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);
        info.k = {focal, 0, width / 2.0, 0, focal, height / 2.0, 0, 0, 1};
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info.p = {focal, 0, width / 2.0, 0, 0, focal, height / 2.0, 0, 0, 0, 1, 0};
        if (base_id_ >= 0) {
            pose.header = image.header;
            pose.header.frame_id = "world";
            pose_pub_->publish(pose);
        }
        info_pub_->publish(info);
        image_pub_->publish(image);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_, stop_client_;
    std::atomic<int> pending_tracking_{-1};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    int base_id_{-1}, tag_id_{-1};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_sub_;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pending_tag_pose_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_sub_;
    bool stand_requested_{true}, have_stand_{false};
    std::chrono::steady_clock::time_point stand_time_{};
    mjvScene scene_{};
    mjvCamera camera_{};
    mjvOption option_{};
    std::mutex cmd_mutex_;
    std::array<double, 3> command_{};
    std::chrono::steady_clock::time_point command_time_{}, last_frame_{};
};
