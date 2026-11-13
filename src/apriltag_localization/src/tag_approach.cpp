// =====================================================================
//  apriltag_localization / tag_approach
//
//  纯视觉测试节点：不装雷达、不用 AMCL / map_server / TF 树，
//  仅靠相机 + AprilTag 让机器人走到 tag 码前面的一个固定位姿。
//  图像源：librealsense2 (D435，内参由 SDK 给出) 或 普通 UVC 摄像头
//          (OpenCV VideoCapture，内参需标定后填入 config)；image_source=auto 自动选择。
//
//  所有常数位姿 TF 由 config/approach.yaml 以 4x4 矩阵给出：
//    T_B_C        相机光学系在机体系中的位姿       p_B = T_B_C * p_C
//    T_tag_B_goal 目标机体系在 tag 坐标系中的位姿  p_tag = T_tag_B_goal * p_Bgoal
//
//  位姿链（当前机体系 B 下表达「目标机体」在哪里）：
//    T_C_tag        = solvePnP(...)                 ① tag 在相机系
//    T_B_tag        = T_B_C * T_C_tag              ② tag 在当前机体系
//    T_Bnow_Bgoal   = T_B_tag * T_tag_B_goal       ③ 目标机体在当前机体系
//    (dx, dy, yaw)  = ③ 的平移 x/y 与 getYaw       ④ 控制误差
//
//  已在目标位姿时 B == B_goal，于是 T_B_tag == T_Bgoal_tag，
//  ③ = T_Bgoal_tag * (T_Bgoal_tag)^-1 = I，误差自然为 0。
//
//  速度输出支持：robot_msgs/Cmd(robot_move_cmd，本工作区实机链路) /
//                geometry_msgs/Twist(cmd_vel) / none(干跑)。
//  详细操作见本包 teach.md「11. 无雷达纯视觉测试」。
// =====================================================================

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/cmd.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "apriltag_localization/pose_math.hpp"

namespace pm = apriltag_localization::pose_math;

namespace apriltag_localization
{

struct ApproachError
{
  double dx{0.0};
  double dy{0.0};
  double dz{0.0};
  double yaw{0.0};
  double stamp{0.0};
};

class TagApproach : public rclcpp::Node
{
public:
  explicit TagApproach(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tag_approach", options)
  {
    declareParameters();
    setupRosInterfaces();
    setupAprilTag();
    setupCamera();
  }

  void start()
  {
    active_.store(auto_start_);
    running_.store(true);
    capture_thread_ = std::thread(&TagApproach::captureLoop, this);

    RCLCPP_INFO(get_logger(),
      "tag_approach 已启动 | tag_id=%d | cmd_output=%s | control_rate=%.1f Hz | "
      "%s", tag_id_, cmd_output_.c_str(), control_rate_,
      auto_start_ ? "自动运行" : "待命中，调用 ~/start 开始（~/stop 急停）");
    RCLCPP_INFO(get_logger(),
      "把机器人摆到期望位姿后调用 ~/record_goal 可直接示教 T_tag_B_goal 矩阵");
  }

  ~TagApproach() override
  {
    active_.store(false);
    running_.store(false);
    image_ready_.notify_all();
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    publishCommand(0.0, 0.0, 0.0, true);
    if (td_ != nullptr) {
      apriltag_detector_destroy(td_);
      td_ = nullptr;
    }
    if (tf_ != nullptr) {
      tag36h11_destroy(tf_);
      tf_ = nullptr;
    }
    if (show_image_) {
      try {
        cv::destroyWindow(kWindowTitle);
      } catch (const std::exception &) {
        // GUI 后端已销毁时忽略
      }
    }
    try {
      if (use_realsense_) {
        pipe_.stop();
      }
    } catch (const std::exception &) {
      // 析构阶段忽略设备已断开之类的错误
    }
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

private:
  static constexpr const char * kWindowTitle = "Tag Approach 相机预览";

  // ---------------------------------------------------------------- 参数
  void declareParameters()
  {
    const auto body_camera = declare_parameter<std::vector<double>>(
      "T_B_C", std::vector<double>{});
    std::string error;
    if (!pm::fromMatrix4x4(body_camera, T_B_C_, error)) {
      throw std::runtime_error("参数 T_B_C 无效: " + error);
    }
    const auto goal_in_tag = declare_parameter<std::vector<double>>(
      "T_tag_B_goal", std::vector<double>{});
    if (!pm::fromMatrix4x4(goal_in_tag, T_tag_B_goal_, error)) {
      throw std::runtime_error("参数 T_tag_B_goal 无效: " + error);
    }

    RCLCPP_INFO(get_logger(), "载入 T_B_C (4x4):\n%s",
      pm::matrixToString(body_camera).c_str());
    RCLCPP_INFO(get_logger(), "载入 T_tag_B_goal (4x4):\n%s",
      pm::matrixToString(goal_in_tag).c_str());

    image_source_ = declare_parameter<std::string>("image_source", "auto");
    uvc_device_ = declare_parameter<std::string>("uvc_device", "/dev/video2");
    camera_fx_ = declare_parameter<double>("camera_fx", 0.0);
    camera_fy_ = declare_parameter<double>("camera_fy", 0.0);
    camera_cx_ = declare_parameter<double>("camera_cx", 0.0);
    camera_cy_ = declare_parameter<double>("camera_cy", 0.0);
    distortion_ = declare_parameter<std::vector<double>>(
      "distortion", std::vector<double>(5, 0.0));
    hfov_deg_ = declare_parameter<double>("hfov_deg", 60.0);

    // 画面参数：<0 表示「不动驱动默认值」。数值范围就是 v4l2-ctl --list-ctrls
    // 打印的范围（UVC）；RealSense 走 rs2 option，量纲见 teach.md 11.1。
    auto_exposure_ = declare_parameter<bool>("auto_exposure", true);
    exposure_ = declare_parameter<int>("exposure", -1);
    gain_ = declare_parameter<int>("gain", -1);
    brightness_ = declare_parameter<int>("brightness", -1);
    contrast_ = declare_parameter<int>("contrast", -1);
    saturation_ = declare_parameter<int>("saturation", -1);
    gamma_ = declare_parameter<int>("gamma", -1);
    sharpness_ = declare_parameter<int>("sharpness", -1);
    backlight_compensation_ = declare_parameter<int>("backlight_compensation", -1);
    white_balance_auto_ = declare_parameter<bool>("white_balance_auto", true);
    white_balance_temperature_ =
      declare_parameter<int>("white_balance_temperature", -1);

    tag_id_ = declare_parameter<int>("tag_id", 0);
    tag_size_ = static_cast<float>(declare_parameter<double>("tag_size", 0.118));
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    quad_decimate_ = declare_parameter<double>("quad_decimate", 1.0);
    quad_sigma_ = declare_parameter<double>("quad_sigma", 0.0);
    nthreads_ = declare_parameter<int>("nthreads", 4);
    refine_edges_ = declare_parameter<bool>("refine_edges", true);
    show_image_ = declare_parameter<bool>("show_image", false);

    cmd_output_ = declare_parameter<std::string>("cmd_output", "robot_move_cmd");
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "robot_move_cmd");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "cmd_vel");
    cmd_mode_ = declare_parameter<int>("cmd_mode", 2);
    stand_mode_ = declare_parameter<int>("stand_mode", 1);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);

    control_mode_ = declare_parameter<std::string>("control_mode", "holonomic");
    kp_x_ = declare_parameter<double>("kp_x", 0.8);
    kp_y_ = declare_parameter<double>("kp_y", 0.6);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.2);
    max_vx_ = declare_parameter<double>("max_vx", 0.40);
    max_vy_ = declare_parameter<double>("max_vy", 0.25);
    max_wz_ = declare_parameter<double>("max_wz", 0.50);
    allow_lateral_ = declare_parameter<bool>("allow_lateral", true);
    yaw_align_band_ = declare_parameter<double>("yaw_align_band", 0.35);
    const double smooth_tau = declare_parameter<double>("smooth_tau", 0.10);
    min_velocity_ = declare_parameter<double>("min_velocity", 0.02);
    minimum_tracking_speed_ = declare_parameter<double>("minimum_tracking_speed", 0.0);

    pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.05);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);
    const double hold_time = declare_parameter<double>("hold_time", 0.30);
    lost_timeout_ = declare_parameter<double>("lost_timeout", 0.5);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    resume_on_reacquire_ = declare_parameter<bool>("resume_on_reacquire", true);
    max_tilt_rad_ = pm::deg2rad(declare_parameter<double>("max_tilt_deg", 30.0));
    publish_debug_poses_ = declare_parameter<bool>("publish_debug_poses", true);
    latch_goal_ = declare_parameter<bool>("latch_goal", false);
    track_after_arrival_ = declare_parameter<bool>("track_after_arrival", false);
    tracking_resume_distance_ = declare_parameter<double>("tracking_resume_distance", 0.08);
    tracking_resume_yaw_ = pm::deg2rad(
      declare_parameter<double>("tracking_resume_yaw_deg", 8.0));
    if (track_after_arrival_ && (tracking_resume_distance_ < pos_tolerance_ || tracking_resume_yaw_ < yaw_tolerance_)) {
      throw std::runtime_error("Tracking resume thresholds must exceed arrival tolerances");
    }
    const auto stand_topic = declare_parameter<std::string>("stand_topic", "");
    if (!stand_topic.empty()) {
      stand_pub_ = create_publisher<std_msgs::msg::Bool>(stand_topic, 1);
    }

    if (tag_id_ < -1) {
      throw std::runtime_error("tag_id 只能是 -1(接受任意 ID) 或 >= 0 的具体 ID");
    }
    if (image_source_ != "auto" && image_source_ != "realsense" &&
        image_source_ != "uvc" && image_source_ != "ros")
    {
      throw std::runtime_error(
        "image_source 只能是 auto / realsense / uvc / ros，当前为 " + image_source_);
    }
    if (distortion_.size() != 5U) {
      throw std::runtime_error("distortion 必须是 5 个数 k1,k2,p1,p2,k3");
    }
    if (hfov_deg_ <= 0.0 || hfov_deg_ >= 180.0) {
      throw std::runtime_error("hfov_deg 必须在 (0, 180) 之间");
    }
    if (cmd_output_ != "robot_move_cmd" && cmd_output_ != "cmd_vel" &&
        cmd_output_ != "none")
    {
      throw std::runtime_error(
        "cmd_output 只能是 robot_move_cmd / cmd_vel / none，当前为 " + cmd_output_);
    }
    if (control_mode_ != "holonomic" && control_mode_ != "differential") {
      throw std::runtime_error(
        "control_mode 只能是 holonomic / differential，当前为 " + control_mode_);
    }
    if (control_rate_ <= 0.0) {
      throw std::runtime_error("control_rate 必须为正数");
    }

    // 频率对齐后，凡是「按周期数」定义的参数都要换算成时间制，
    // 否则把 control_rate 从 20 Hz 提到 50 Hz 会悄悄改变滤波强度和到位判定时间。
    const double period = 1.0 / control_rate_;
    // 一阶低通 alpha = 1 - exp(-T/tau)，与频率解耦
    smooth_alpha_ = (smooth_tau <= 0.0)
      ? 1.0
      : (1.0 - std::exp(-period / smooth_tau));
    // 到位保持时间 -> 控制周期数
    const double hold = std::max(hold_time, period);
    hold_ticks_ = std::max(1, static_cast<int>(std::ceil(hold * control_rate_)));

    RCLCPP_INFO(get_logger(),
      "控制频率 %.1f Hz (周期 %.2f ms)，对齐 rl_real_atdog2 的 dt*decimation 策略周期"
      " | smooth_alpha=%.3f hold_ticks=%d",
      control_rate_, 1000.0 / control_rate_, smooth_alpha_, hold_ticks_);
  }

  // ------------------------------------------------------------ ROS 接口
  void setupRosInterfaces()
  {
    cmd_pub_ = create_publisher<robot_msgs::msg::Cmd>(cmd_topic_, 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
    tag_pose_body_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/tag_pose_body", 1);
    goal_pose_body_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/goal_pose_body", 1);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        hold_count_ = 0;
        reached_ = false;
        resetFilters();
        active_.store(true);
        resp->success = true;
        resp->message = "开始视觉趋近；丢失 tag 会自动停车";
        RCLCPP_INFO(get_logger(), "~/start：开始视觉趋近");
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        active_.store(false);
        resetFilters();
        publishCommand(0.0, 0.0, 0.0, true);
        resp->success = true;
        resp->message = "已停车并交还控制权";
        RCLCPP_WARN(get_logger(), "~/stop：已停车（不再发布速度，交还控制权）");
      });

    record_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/record_goal",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        record_requested_.store(true);
        resp->success = true;
        resp->message = "将在下一帧检测时记录目标位姿并打印 4x4 矩阵";
        RCLCPP_INFO(get_logger(), "~/record_goal：请把机器人摆在期望位姿并让相机看到 tag");
      });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(period, [this]() { controlTick(); });
  }

  // ------------------------------------------------------------- 相机初始化
  void setupCamera()
  {
    if (image_source_ == "ros") {
      const auto image_topic = declare_parameter<std::string>(
        "image_topic", "/mujoco/camera/image_raw");
      const auto info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/mujoco/camera/camera_info");
      info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {return;}
          std::lock_guard<std::mutex> lock(image_mutex_);
          ros_info_ = msg;
        });
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          if ((msg->encoding != "rgb8" && msg->encoding != "bgr8") ||
            msg->width == 0 || msg->height == 0 || msg->step < msg->width * 3 ||
            msg->data.size() < static_cast<size_t>(msg->step) * msg->height)
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "ROS 图像需要有效的 rgb8/bgr8 数据");
            return;
          }
          std::lock_guard<std::mutex> lock(image_mutex_);
          ros_image_ = msg;
          image_ready_.notify_one();
        });
      camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
      RCLCPP_INFO(get_logger(), "等待 ROS 仿真图像 %s 和内参 %s",
        image_topic.c_str(), info_topic.c_str());
    }
    std::string source = image_source_;
    if (source == "auto") {
      // 没有 D435 时自动退回普通 UVC 摄像头，测试机无需 RealSense。
      // 探测本身也可能抛异常（udev 规则缺失导致无访问权限），此时按无设备处理。
      bool has_realsense = false;
      try {
        rs2::context ctx;
        has_realsense = ctx.query_devices().size() > 0U;
      } catch (const rs2::error & e) {
        RCLCPP_WARN(get_logger(), "探测 RealSense 失败 (%s)，按 UVC 处理", e.what());
      }
      source = has_realsense ? "realsense" : "uvc";
      RCLCPP_INFO(get_logger(), "image_source=auto -> %s", source.c_str());
    }
    use_realsense_ = (source == "realsense");

    if (use_realsense_) {
      setupRealsense();
    } else if (source != "ros") {
      setupUvc();
    }

    const float s = tag_size_ / 2.0f;
    // apriltag 的 p[] 从 tag 右上角开始、绕 tag 逆时针：
    //   p[0]=右上 p[1]=左上 p[2]=左下 p[3]=右下
    // （用 tag36h11 正立合成图实测确认，写错会让 T_C_tag 的 yaw 恒定差 180°）
    // tag 坐标系：+x 由 p[1] 指向 p[0]（图像右），+y 由 p[3] 指向 p[0]（图像上），
    // +z = +x × +y，指向码正面外侧。
    object_points_ = {
      cv::Point3f(s, s, 0.0f),
      cv::Point3f(-s, s, 0.0f),
      cv::Point3f(-s, -s, 0.0f),
      cv::Point3f(s, -s, 0.0f),
    };

    const std::string source_label = use_realsense_
      ? "RealSense D435" : (source == "ros" ? "ROS MuJoCo" : "UVC " + uvc_device_);
    RCLCPP_INFO(get_logger(),
      "图像源=%s | %dx%d@%d | fx=%.1f fy=%.1f cx=%.1f cy=%.1f | tag_size=%.4f m",
      source_label.c_str(), image_width_, image_height_, fps_,
      camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
      camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2),
      tag_size_);
  }

  void setupRealsense()
  {
    rs2::context ctx;
    if (ctx.query_devices().size() == 0U) {
      throw std::runtime_error(
        "image_source 指定 realsense 但未检测到 D435；改用 auto 或 uvc");
    }

    rs2::config cfg;
    cfg.enable_stream(
      RS2_STREAM_COLOR, image_width_, image_height_, RS2_FORMAT_BGR8, fps_);
    pipe_.start(cfg);

    const auto profile = pipe_.get_active_profile();
    const auto color_stream =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto intr = color_stream.get_intrinsics();

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      intr.fx, 0.0, intr.ppx,
      0.0, intr.fy, intr.ppy,
      0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    if (intr.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY ||
        intr.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
      for (int i = 0; i < 5; ++i) {
        dist_coeffs_.at<double>(i) = intr.coeffs[i];
      }
    }

    applyRealsenseTuning();
  }

  void setupUvc()
  {
    // 纯数字当 OpenCV 索引，否则当 /dev/videoX 路径
    const bool numeric = !uvc_device_.empty() &&
      std::all_of(uvc_device_.begin(), uvc_device_.end(),
        [](unsigned char c) { return std::isdigit(c) != 0; });
    const bool opened = numeric
      ? capture_.open(std::stoi(uvc_device_), cv::CAP_V4L2)
      : capture_.open(uvc_device_, cv::CAP_V4L2);
    if (!opened) {
      throw std::runtime_error(
        "无法打开 UVC 摄像头 '" + uvc_device_ +
        "'，用 v4l2-ctl --list-devices 确认设备路径");
    }

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    capture_.set(cv::CAP_PROP_FPS, fps_);

    // 驱动可能不接受请求的分辨率，以内参必须和实际图像尺寸一致为准
    double w = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
    double h = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (w <= 0.0 || h <= 0.0) {
      w = image_width_;
      h = image_height_;
      RCLCPP_WARN(get_logger(), "读不到 UVC 实际分辨率，按参数 %dx%d 计算内参",
        image_width_, image_height_);
    } else if (static_cast<int>(w) != image_width_ ||
      static_cast<int>(h) != image_height_)
    {
      RCLCPP_WARN(get_logger(),
        "UVC 实际分辨率 %dx%d 与请求的 %dx%d 不同，按实际分辨率计算内参",
        static_cast<int>(w), static_cast<int>(h), image_width_, image_height_);
      image_width_ = static_cast<int>(w);
      image_height_ = static_cast<int>(h);
    }

    double fx = camera_fx_, fy = camera_fy_;
    double cx = camera_cx_, cy = camera_cy_;
    if (fx <= 0.0 || fy <= 0.0) {
      const double half_hfov = pm::deg2rad(hfov_deg_) / 2.0;
      fx = (w / 2.0) / std::tan(half_hfov);
      fy = fx;
      cx = w / 2.0;
      cy = h / 2.0;
      RCLCPP_WARN(get_logger(),
        "UVC 未标定内参：按 hfov_deg=%.1f 估算 fx=fy=%.1f，"
        "PnP 距离会有系统性偏差，请棋盘格标定后填 camera_fx/fy/cx/cy",
        hfov_deg_, fx);
    }
    if (cx <= 0.0) {
      cx = w / 2.0;
    }
    if (cy <= 0.0) {
      cy = h / 2.0;
    }

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    for (int i = 0; i < 5; ++i) {
      dist_coeffs_.at<double>(i) = distortion_[i];
    }

    // 放在分辨率设置之后：部分驱动切分辨率会把控制项重置回默认值
    applyUvcTuning();
  }

  // 画面参数只走 OpenCV 的 V4L2 后端（CAP_PROP_* 直接映射到 V4L2 CID，
  // 数值不做归一化），所以这里用到的范围就是 v4l2-ctl --list-ctrls 打印的范围。
  void applyUvcTuning()
  {
    // 曝光必须先落「手动」再写绝对值，否则驱动会把手动值丢掉
    if (auto_exposure_) {
      // 3 = V4L2_EXPOSURE_APERTURE_PRIORITY；UVC 上通常只有这个才是真自动
      setUvcControl(cv::CAP_PROP_AUTO_EXPOSURE, 3.0, "auto_exposure(auto)");
    } else {
      setUvcControl(cv::CAP_PROP_AUTO_EXPOSURE, 1.0, "auto_exposure(manual)");
      if (exposure_ > 0) {
        setUvcControl(cv::CAP_PROP_EXPOSURE, exposure_, "exposure[100us]");
      }
    }

    const struct Item
    {
      int value;
      cv::VideoCaptureProperties prop;
      const char * name;
    } items[] = {
      {gain_, cv::CAP_PROP_GAIN, "gain"},
      {brightness_, cv::CAP_PROP_BRIGHTNESS, "brightness"},
      {contrast_, cv::CAP_PROP_CONTRAST, "contrast"},
      {saturation_, cv::CAP_PROP_SATURATION, "saturation"},
      {gamma_, cv::CAP_PROP_GAMMA, "gamma"},
      {sharpness_, cv::CAP_PROP_SHARPNESS, "sharpness"},
      {backlight_compensation_, cv::CAP_PROP_BACKLIGHT, "backlight_compensation"},
    };
    for (const auto & item : items) {
      if (item.value >= 0) {
        setUvcControl(item.prop, item.value, item.name);
      }
    }

    setUvcControl(cv::CAP_PROP_AUTO_WB, white_balance_auto_ ? 1.0 : 0.0,
      "auto_white_balance");
    if (!white_balance_auto_ && white_balance_temperature_ > 0) {
      setUvcControl(cv::CAP_PROP_WB_TEMPERATURE, white_balance_temperature_,
        "white_balance_temperature[K]");
    }
    // 工频（日光灯横条纹）OpenCV 没有映射，只能在启动前用
    //   v4l2-ctl -d /dev/video2 --set-ctrl=power_line_frequency=1   # 国内 50Hz
    // 设一次；节点不去动它。
  }

  void setUvcControl(int prop, double value, const char * name)
  {
    if (!capture_.set(prop, value)) {
      RCLCPP_WARN(get_logger(),
        "UVC 设置 %s=%g 失败（驱动不支持该控制项，见 v4l2-ctl --list-ctrls）",
        name, value);
      return;
    }
    RCLCPP_INFO(get_logger(), "UVC %s = %g", name, capture_.get(prop));
  }

  // RealSense 走 rs2 option：曝光单位是 us（参数按 100 us 书写，乘 100），
  // gain/brightness 的量纲与 UVC 完全不同，不要照抄 UVC 的数值。
  void applyRealsenseTuning()
  {
    rs2::sensor color_sensor;
    try {
      color_sensor =
        pipe_.get_active_profile().get_device().first<rs2::color_sensor>();
    } catch (const rs2::error & e) {
      RCLCPP_WARN(get_logger(), "取不到 D435 彩色传感器，跳过画面参数: %s", e.what());
      return;
    }

    setRsOption(color_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE,
      auto_exposure_ ? 1.0f : 0.0f, "auto_exposure");
    if (!auto_exposure_ && exposure_ > 0) {
      setRsOption(color_sensor, RS2_OPTION_EXPOSURE,
        static_cast<float>(exposure_) * 100.0f, "exposure[us]");
    }

    const struct Item
    {
      int value;
      rs2_option option;
      const char * name;
    } items[] = {
      {gain_, RS2_OPTION_GAIN, "gain"},
      {brightness_, RS2_OPTION_BRIGHTNESS, "brightness"},
      {contrast_, RS2_OPTION_CONTRAST, "contrast"},
      {saturation_, RS2_OPTION_SATURATION, "saturation"},
      {gamma_, RS2_OPTION_GAMMA, "gamma"},
      {sharpness_, RS2_OPTION_SHARPNESS, "sharpness"},
      {backlight_compensation_, RS2_OPTION_BACKLIGHT_COMPENSATION,
        "backlight_compensation"},
    };
    for (const auto & item : items) {
      if (item.value >= 0) {
        setRsOption(color_sensor, item.option, static_cast<float>(item.value),
          item.name);
      }
    }

    setRsOption(color_sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
      white_balance_auto_ ? 1.0f : 0.0f, "auto_white_balance");
    if (!white_balance_auto_ && white_balance_temperature_ > 0) {
      setRsOption(color_sensor, RS2_OPTION_WHITE_BALANCE,
        static_cast<float>(white_balance_temperature_), "white_balance[K]");
    }
    // 工频同理不在节点里设：RealSense 用 rs-enumerate-devices 或
    // realsense-viewer 的 Power Line Frequency 选项改成 50Hz 即可。
  }

  void setRsOption(
    rs2::sensor & sensor, rs2_option option, float value, const char * name)
  {
    if (!sensor.supports(option)) {
      RCLCPP_DEBUG(get_logger(), "RealSense 不支持 %s，跳过", name);
      return;
    }
    try {
      sensor.set_option(option, value);
      RCLCPP_INFO(get_logger(), "RealSense %s = %.0f", name,
        sensor.get_option(option));
    } catch (const rs2::error & e) {
      RCLCPP_WARN(get_logger(), "RealSense 设置 %s 失败: %s", name, e.what());
    }
  }

  // 两种图像源统一产出一张 BGR 图；失败时返回空 Mat
  cv::Mat grabFrame()
  {
    if (image_source_ == "ros") {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_ready_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
        return !running_.load() || (ros_image_ && ros_info_);
      });
      if (!running_.load() || !ros_image_ || !ros_info_) {return {};}
      const auto msg = std::move(ros_image_);
      const auto info = ros_info_;
      lock.unlock();
      if (info->width != msg->width || info->height != msg->height) {return {};}
      camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        info->k[0], 0, info->k[2], 0, info->k[4], info->k[5], 0, 0, 1);
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
      for (size_t i = 0; i < std::min<size_t>(5, info->d.size()); ++i) {
        dist_coeffs_.at<double>(0, i) = info->d[i];
      }
      cv::Mat raw(msg->height, msg->width, CV_8UC3,
        const_cast<uint8_t *>(msg->data.data()), msg->step);
      cv::Mat bgr;
      if (msg->encoding == "rgb8") {cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);}
      else {bgr = raw.clone();}
      return bgr;
    }
    if (use_realsense_) {
      rs2::frameset frames;
      try {
        frames = pipe_.wait_for_frames(1000U);
      } catch (const rs2::error & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "等待 D435 图像失败: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return {};
      }
      const rs2::video_frame color_frame = frames.get_color_frame();
      if (!color_frame) {
        return {};
      }
      cv::Mat bgr(
        cv::Size(color_frame.get_width(), color_frame.get_height()),
        CV_8UC3,
        const_cast<void *>(color_frame.get_data()),
        cv::Mat::AUTO_STEP);
      return bgr.clone();
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "UVC 读帧失败 (%s)", uvc_device_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return {};
    }
    return frame;
  }

  void setupAprilTag()
  {
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = static_cast<float>(quad_decimate_);
    td_->quad_sigma = static_cast<float>(quad_sigma_);
    td_->nthreads = nthreads_;
    td_->debug = 0;
    td_->refine_edges = refine_edges_ ? 1 : 0;
  }

  // ------------------------------------------------------------- 采集线程
  // OpenCV 的 imshow 在无图形界面时会直接 abort，先看环境变量再决定开不开窗口
  static bool hasDisplay()
  {
    const char * display = std::getenv("DISPLAY");
    if (display != nullptr && display[0] != '\0') {
      return true;
    }
    const char * wayland = std::getenv("WAYLAND_DISPLAY");
    return wayland != nullptr && wayland[0] != '\0';
  }

  void captureLoop()
  {
    bool window_ready = false;
    if (show_image_ && !hasDisplay()) {
      RCLCPP_WARN(get_logger(),
        "show_image=true 但没有图形界面 (DISPLAY / WAYLAND_DISPLAY 均为空)，"
        "已自动关闭相机预览窗口；想在桌面看画面请在本机会话里启动，或 ssh -X 转发");
      show_image_ = false;
    }
    RCLCPP_INFO(get_logger(), "相机预览窗口：%s",
      show_image_ ? "开（叠加 tag 检测框与坐标轴，ESC 关闭）" : "关");

    while (running_.load() && rclcpp::ok()) {
      cv::Mat image = grabFrame();
      if (image.empty()) {
        continue;
      }
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

      // AprilTag 3.2.0 的 image_u8 前三个成员是 const，必须聚合初始化
      image_u8_t img = {
        static_cast<int32_t>(gray.cols),
        static_cast<int32_t>(gray.rows),
        static_cast<int32_t>(gray.step[0]),
        gray.data,
      };

      zarray_t * detections = apriltag_detector_detect(td_, &img);
      const int n_det = zarray_size(detections);
      bool found_target = false;
      std::string seen_ids;
      for (int i = 0; i < n_det; ++i) {
        apriltag_detection_t * det = nullptr;
        zarray_get(detections, i, &det);
        if (i != 0) {
          seen_ids += ", ";
        }
        seen_ids += std::to_string(det->id);

        std::vector<cv::Point2f> image_points;
        image_points.reserve(4);
        for (int j = 0; j < 4; ++j) {
          image_points.emplace_back(
            static_cast<float>(det->p[j][0]), static_cast<float>(det->p[j][1]));
        }

        const cv::Point center(
          static_cast<int>(det->c[0]), static_cast<int>(det->c[1]));
        if (show_image_) {
          for (int j = 0; j < 4; ++j) {
            cv::line(image, image_points[j], image_points[(j + 1) % 4],
              cv::Scalar(0, 255, 0), 2);
          }
          cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);
          cv::putText(image, "ID: " + std::to_string(det->id),
            center + cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX,
            0.6, cv::Scalar(0, 255, 255), 2);
        }

        // tag_id < 0 时接受任意 ID（只取第一个），方便临时换码测试
        if (found_target || (tag_id_ >= 0 && det->id != tag_id_)) {
          continue;
        }

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(
            object_points_, image_points, camera_matrix_, dist_coeffs_,
            rvec, tvec))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "tag %d 的 solvePnP 失败（角点退化或内参异常），本帧忽略", det->id);
          continue;
        }
        if (tvec.at<double>(2) <= 0.0 || tvec.at<double>(2) > 10.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "tag %d 解出 z=%.2f m 不在 (0, 10] 内，本帧忽略；"
            "检查 tag_size=%.3f m 与相机内参",
            det->id, tvec.at<double>(2), tag_size_);
          continue;
        }

        // ① PnP -> ② tag 在当前机体系 -> ③ 目标机体在当前机体系
        const tf2::Transform T_C_tag = pm::fromSolvePnP(rvec, tvec);

        // tag 中心在相机光学系（x 右 / y 下 / z 前）下的 6 个自由度量
        double c_roll = 0.0, c_pitch = 0.0, c_yaw = 0.0;
        tf2::Matrix3x3(T_C_tag.getRotation()).getRPY(c_roll, c_pitch, c_yaw);
        const auto & c_o = T_C_tag.getOrigin();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "T_C_tag | x=%+.4f y=%+.4f z=%+.4f m | "
          "roll=%+.2f pitch=%+.2f yaw=%+.2f deg",
          c_o.x(), c_o.y(), c_o.z(),
          pm::rad2deg(c_roll), pm::rad2deg(c_pitch), pm::rad2deg(c_yaw));

        const tf2::Transform T_B_tag = T_B_C_ * T_C_tag;

        // 示教：把机器人摆在期望位姿时，当前机体在 tag 系中的位姿就是目标，
        // 即 T_tag_B_goal = (T_B_tag)^-1。只在采集线程访问，无需加锁。
        if (record_requested_.exchange(false)) {
          T_tag_B_goal_ = T_B_tag.inverse();
          RCLCPP_INFO(get_logger(),
            "已示教 T_tag_B_goal，请把下面的 4x4 矩阵抄回 config/approach.yaml:\n%s",
            pm::matrixToString(pm::toMatrix4x4(T_tag_B_goal_)).c_str());
        }

        const tf2::Transform T_goal_in_body = T_B_tag * T_tag_B_goal_;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(T_goal_in_body.getRotation()).getRPY(roll, pitch, yaw);
        if (std::fabs(roll) > max_tilt_rad_ || std::fabs(pitch) > max_tilt_rad_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "目标相对当前机体倾角异常 roll=%.1f pitch=%.1f deg，"
            "请检查 T_B_C / T_tag_B_goal 是否写反，本帧忽略",
            pm::rad2deg(roll), pm::rad2deg(pitch));
          continue;
        }

        ApproachError e;
        const auto & o = T_goal_in_body.getOrigin();
        e.dx = o.x();
        e.dy = o.y();
        e.dz = o.z();
        e.yaw = yaw;
        e.stamp = now().seconds();
        {
          std::lock_guard<std::mutex> lock(error_mutex_);
          latest_error_ = e;
        }

        publishDebugPose(tag_pose_body_pub_, T_B_tag);
        publishDebugPose(goal_pose_body_pub_, T_goal_in_body);

        if (show_image_) {
          cv::drawFrameAxes(image, camera_matrix_, dist_coeffs_, rvec, tvec, 0.05);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "tag %d | 机体系误差 dx=%+.3f dy=%+.3f dz=%+.3f yaw=%+.1f deg",
          det->id, e.dx, e.dy, e.dz, pm::rad2deg(e.yaw));
        found_target = true;
      }
      // 没有位姿输出时也要说清楚原因，避免「什么都不打印」无从排查
      if (n_det == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "本帧未检测到任何 AprilTag：确认预览窗口里能看到完整 tag、"
          "亮度正常、tag_size=%.3f m 与实物一致", tag_size_);
      } else if (!found_target) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "本帧检测到 %d 个 tag (id=%s) 但没有有效位姿："
          "目标 tag_id=%d，把 config/approach.yaml 的 tag_id 改成画面里的 ID"
          "（或设 -1 接受任意 ID）；ID 已匹配时看上面的 PnP/距离/倾角 WARN",
          n_det, seen_ids.c_str(), tag_id_);
      }
      apriltag_detections_destroy(detections);

      if (show_image_) {
        if (!window_ready) {
          // WINDOW_NORMAL 让窗口可拖动缩放，画面按窗口大小自适应
          cv::namedWindow(kWindowTitle, cv::WINDOW_NORMAL);
          cv::resizeWindow(kWindowTitle, 960, 720);
          window_ready = true;
        }
        cv::imshow(kWindowTitle, image);
        // waitKey 同时负责 GUI 事件循环，ESC(27) 关闭窗口但不影响节点运行
        if (cv::waitKey(1) == 27) {
          show_image_ = false;
          cv::destroyWindow(kWindowTitle);
          window_ready = false;
          RCLCPP_INFO(get_logger(), "已关闭相机预览窗口（按 ESC），节点继续运行");
        }
      }
    }
  }

  void publishDebugPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    const tf2::Transform & T)
  {
    if (!publish_debug_poses_ || !pub) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";   // 测试模式无 TF 树，仅作 rviz 显示用
    msg.pose = pm::toPose(T);
    pub->publish(msg);
  }

  // ------------------------------------------------------------- 控制定时
  void controlTick()
  {
    // 未激活时完全不发布，避免抢占遥控器/其它节点的控制权
    if (!active_.load()) {
      return;
    }

    if (latch_goal_ && !track_after_arrival_ && reached_) {
      publishCommand(0.0, 0.0, 0.0, true);
      return;
    }
    std::optional<ApproachError> e;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      e = latest_error_;
    }
    const double now_s = now().seconds();

    if (!e || (now_s - e->stamp) > lost_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "tag 丢失或数据超时 (> %.2f s)，发零速停车", lost_timeout_);
      publishCommand(0.0, 0.0, 0.0, true);
      resetFilters();
      hold_count_ = 0;
      if (!resume_on_reacquire_) {
        active_.store(false);
        RCLCPP_WARN(get_logger(), "已停机，需重新调用 ~/start");
      }
      return;
    }

    const double dist = std::hypot(e->dx, e->dy);
    if (track_after_arrival_ && reached_) {
      if (dist <= tracking_resume_distance_ && std::fabs(e->yaw) <= tracking_resume_yaw_) {
        publishCommand(0.0, 0.0, 0.0, true);
        return;
      }
      reached_ = false;
      hold_count_ = 0;
      resetFilters();
      RCLCPP_INFO(get_logger(), "目标已移动或机体偏离，恢复视觉跟踪行走");
    }
    // Once braking has begun, allow the small pose shift caused by changing
    // from the walking policy to position-controlled stand. This hysteresis
    // prevents repeatedly reloading the policy at the arrival boundary.
    const double arrival_band = (latch_goal_ || track_after_arrival_) && hold_count_ > 0 ? 2.0 : 1.0;
    const bool within =
      dist <= pos_tolerance_ * arrival_band &&
      std::fabs(e->yaw) <= yaw_tolerance_ * arrival_band;
    if (within) {
      ++hold_count_;
      publishCommand(0.0, 0.0, 0.0, true);
      resetFilters();
      if (hold_count_ >= hold_ticks_ && !reached_) {
        reached_ = true;
        RCLCPP_INFO(get_logger(),
          "已到达目标位姿 (dist=%.3f m, yaw=%.2f deg)，保持零速站立；"
          "%s", dist, pm::rad2deg(e->yaw),
          track_after_arrival_ ? "继续追踪 tag，偏离目标时自动恢复行走" : "调用 ~/stop 可交还控制权");
      }
      return;
    }
    hold_count_ = 0;
    reached_ = false;

    double vx = 0.0, vy = 0.0, wz = 0.0;
    if (control_mode_ == "differential") {
      const double heading = std::atan2(e->dy, e->dx);
      wz = kp_yaw_ * heading;
      if (std::fabs(heading) <= yaw_align_band_) {
        vx = kp_x_ * dist * std::cos(heading);
      }
    } else {
      wz = kp_yaw_ * e->yaw;
      if (std::fabs(e->yaw) <= yaw_align_band_) {
        vx = kp_x_ * e->dx;
        if (allow_lateral_) {
          vy = kp_y_ * e->dy;
        }
      }
    }

    vx = clampAbs(vx, max_vx_);
    vy = clampAbs(vy, max_vy_);
    wz = clampAbs(wz, max_wz_);
    vx = (std::fabs(vx) < min_velocity_) ? 0.0 : vx;
    vy = (std::fabs(vy) < min_velocity_) ? 0.0 : vy;
    wz = (std::fabs(wz) < min_velocity_) ? 0.0 : wz;

    // The command-driven gait clock cannot restart below its entry threshold.
    // Use a configured minimum only while correcting a position error.
    const double speed = std::hypot(vx, vy);
    if (speed > 1e-6 && speed < minimum_tracking_speed_) {
      vx *= minimum_tracking_speed_ / speed;
      vy *= minimum_tracking_speed_ / speed;
      vx = clampAbs(vx, max_vx_);
      vy = clampAbs(vy, max_vy_);
    }
    vx = smooth_alpha_ * vx + (1.0 - smooth_alpha_) * vx_prev_;
    vy = smooth_alpha_ * vy + (1.0 - smooth_alpha_) * vy_prev_;
    wz = smooth_alpha_ * wz + (1.0 - smooth_alpha_) * wz_prev_;
    vx_prev_ = vx;
    vy_prev_ = vy;
    wz_prev_ = wz;

    publishCommand(vx, vy, wz, false);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "cmd vx=%+.3f vy=%+.3f wz=%+.3f | err d=%.3f yaw=%.1f deg",
      vx, vy, wz, dist, pm::rad2deg(e->yaw));
  }

  static double clampAbs(double v, double limit)
  {
    return std::max(-limit, std::min(limit, v));
  }

  void resetFilters()
  {
    vx_prev_ = 0.0;
    vy_prev_ = 0.0;
    wz_prev_ = 0.0;
  }

  void publishCommand(double vx, double vy, double wz, bool stand)
  {
    if (stand_pub_) {
      std_msgs::msg::Bool state;
      state.data = stand;
      stand_pub_->publish(state);
    }
    if (cmd_output_ == "robot_move_cmd") {
      robot_msgs::msg::Cmd cmd;
      cmd.mode = stand ? stand_mode_ : cmd_mode_;
      cmd.vx = static_cast<float>(vx);
      cmd.vy = static_cast<float>(vy);
      // 本工作区约定：Cmd.vz 是偏航角速度，不是 z 向线速度
      cmd.vz = static_cast<float>(wz);
      cmd.wheel_vel = 0.0f;
      cmd_pub_->publish(cmd);
    } else if (cmd_output_ == "cmd_vel") {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = vx;
      twist.linear.y = vy;
      twist.angular.z = wz;
      twist_pub_->publish(twist);
    }
    // cmd_output == "none"：干跑，只靠日志观察，不发任何速度
  }

  // ---------------------------------------------------------------- 成员
  tf2::Transform T_B_C_;
  tf2::Transform T_tag_B_goal_;   // 只由采集线程读写

  int tag_id_{0}, image_width_{640}, image_height_{480}, fps_{30};
  int nthreads_{4}, cmd_mode_{2}, stand_mode_{1}, hold_ticks_{10};
  float tag_size_{0.118f};
  double quad_decimate_{1.0}, quad_sigma_{0.0};
  double control_rate_{20.0}, kp_x_{0.8}, kp_y_{0.6}, kp_yaw_{1.2};
  double max_vx_{0.4}, max_vy_{0.25}, max_wz_{0.5};
  double yaw_align_band_{0.35}, smooth_alpha_{0.4}, min_velocity_{0.02};
  double pos_tolerance_{0.05}, yaw_tolerance_{0.05}, lost_timeout_{0.5};
  double minimum_tracking_speed_{0.0};
  double max_tilt_rad_{0.52};
  double vx_prev_{0.0}, vy_prev_{0.0}, wz_prev_{0.0};
  bool refine_edges_{true}, show_image_{false}, allow_lateral_{true};
  bool auto_start_{false}, resume_on_reacquire_{true}, latch_goal_{false};
  bool track_after_arrival_{false};
  double tracking_resume_distance_{0.08}, tracking_resume_yaw_{0.14};
  bool publish_debug_poses_{true}, reached_{false};
  int hold_count_{0};
  std::string cmd_output_, cmd_topic_, twist_topic_, control_mode_;

  std::vector<cv::Point3f> object_points_;
  cv::Mat camera_matrix_, dist_coeffs_;

  // 图像源选择：realsense -> librealsense2；uvc -> OpenCV VideoCapture。
  // auto 时先探 D435，没有就退回 UVC，测试机不必插 RealSense。
  std::string image_source_{"auto"};
  std::string uvc_device_{"/dev/video2"};
  bool use_realsense_{false};
  double camera_fx_{0.0}, camera_fy_{0.0}, camera_cx_{0.0}, camera_cy_{0.0};
  std::vector<double> distortion_{std::vector<double>(5, 0.0)};
  double hfov_deg_{60.0};
  cv::VideoCapture capture_;
  std::mutex image_mutex_;
  std::condition_variable image_ready_;
  sensor_msgs::msg::Image::ConstSharedPtr ros_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr ros_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  // 画面参数（曝光/增益/亮度等），<0 表示不动驱动默认值
  bool auto_exposure_{true}, white_balance_auto_{true};
  int exposure_{-1}, gain_{-1}, brightness_{-1}, contrast_{-1}, saturation_{-1};
  int gamma_{-1}, sharpness_{-1}, backlight_compensation_{-1};
  int white_balance_temperature_{-1};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stand_pub_;
  rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    tag_pose_body_pub_, goal_pose_body_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    start_srv_, stop_srv_, record_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rs2::pipeline pipe_;
  apriltag_family_t * tf_{nullptr};
  apriltag_detector_t * td_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> record_requested_{false};
  std::mutex error_mutex_;
  std::optional<ApproachError> latest_error_;
  std::thread capture_thread_;
};

}  // namespace apriltag_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<apriltag_localization::TagApproach>(
      rclcpp::NodeOptions());
    node->start();
    rclcpp::spin(node);
  } catch (const rs2::error & e) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_approach"),
      "RealSense 初始化失败: %s（可把 image_source 设为 uvc 改用普通摄像头）",
      e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_approach"),
      "节点初始化失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
