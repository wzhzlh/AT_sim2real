// AprilTag 绝对地标重定位：M=理想地图，O=雷达现实 odom，B=机器人。
// 每帧按图像时间配对：T_M_O = T_M_B(tag) * inverse(T_O_B)。
// 持续广播 M -> O，并发布 T_M_child = T_M_O * T_O_child 的 Odometry。
// 不发布 O -> child（由雷达负责），不依赖 AMCL。

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <algorithm>
#include <iomanip>
#include <map>
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "apriltag_localization/pose_math.hpp"

namespace pm = apriltag_localization::pose_math;

namespace apriltag_localization
{

class AprilTagLocalizer : public rclcpp::Node
{
public:
  explicit AprilTagLocalizer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("apriltag_localizer", options)
  {
    declareParameters();
    setupRosInterfaces();
    setupCamera();       // 可能抛 rs2::error，由 main 捕获
    setupAprilTag();
  }

  // 必须由 main 在 shared_ptr 构造完成之后调用：
  // 静态 TF broadcaster 需要 shared_from_this()，不能在构造函数里创建。
  void start()
  {
    if (publish_camera_tf_) {
      publishCameraStaticTf();
    }

    need_publish_.store(trigger_mode_ != "manual");
    running_.store(true);
    capture_thread_ = std::thread(&AprilTagLocalizer::captureLoop, this);

    RCLCPP_INFO(get_logger(),
      "Tag 重定位：%s -> %s；输入 %s，校正输出 %s；模式 %s，%zu 个地标",
      map_frame_.c_str(), odom_frame_.c_str(), odom_topic_.c_str(),
      corrected_odom_topic_.c_str(), trigger_mode_.c_str(), tag_poses_.size());
  }

  ~AprilTagLocalizer() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (td_ != nullptr) {
      apriltag_detector_destroy(td_);
      td_ = nullptr;
    }
    if (tf_ != nullptr) {
      tag36h11_destroy(tf_);
      tf_ = nullptr;
    }
    try {
      pipe_.stop();
    } catch (const std::exception &) {
      // 析构阶段忽略设备已经断开之类的错误
    }
  }

private:
  // ---------------------------------------------------------------- 参数
  void declareParameters()
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "laser");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_color_optical_frame");

    tag_size_ = static_cast<float>(declare_parameter<double>("tag_size", 0.118));
    target_tag_id_ = declare_parameter<int>("target_tag_id", -1);
    min_samples_ = declare_parameter<int>("min_samples", 10);
    max_spread_ = declare_parameter<double>("max_spread", 0.08);
    trigger_mode_ = declare_parameter<std::string>("trigger_mode", "auto_once");
    max_rotation_spread_ = pm::deg2rad(
      declare_parameter<double>("max_rotation_spread_deg", 5.0));
    max_sample_gap_ = declare_parameter<double>("max_sample_gap", 0.5);
    publish_debug_poses_ = declare_parameter<bool>("publish_debug_poses", true);

    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.2);
    max_tilt_rad_ = pm::deg2rad(declare_parameter<double>("max_tilt_deg", 30.0));
    show_image_ = declare_parameter<bool>("show_image", false);

    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    fps_ = declare_parameter<int>("fps", 30);

    quad_decimate_ = declare_parameter<double>("quad_decimate", 1.0);
    quad_sigma_ = declare_parameter<double>("quad_sigma", 0.0);
    nthreads_ = declare_parameter<int>("nthreads", 4);
    refine_edges_ = declare_parameter<bool>("refine_edges", true);

    publish_camera_tf_ = declare_parameter<bool>("publish_camera_tf", false);
    camera_tf_parent_ = declare_parameter<std::string>("camera_tf_parent", "base_link");
    camera_in_parent_ = declare_parameter<std::vector<double>>(
      "camera_in_parent", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    corrected_odom_topic_ = declare_parameter<std::string>(
      "corrected_odom_topic", "/odom_corrected");
    tf_update_hz_ = declare_parameter<double>("tf_update_hz", 30.0);
    odom_history_sec_ = declare_parameter<double>("odom_history_sec", 5.0);
    max_sync_error_ = declare_parameter<double>("max_sync_error", 0.10);
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
        map_frame_ == odom_frame_ || base_frame_ == map_frame_ ||
        base_frame_ == odom_frame_ || odom_topic_ == corrected_odom_topic_ ||
        tf_update_hz_ <= 0.0 || odom_history_sec_ <= 0.0 || max_sync_error_ < 0.0 ||
        max_sample_gap_ <= 0.0 || max_spread_ <= 0.0 || max_rotation_spread_ <= 0.0)
    {
      throw std::runtime_error("帧名、输入输出话题或采样/时间参数无效");
    }

    if (min_samples_ < 1) {
      min_samples_ = 1;
    }
    if (trigger_mode_ != "auto_once" && trigger_mode_ != "manual" &&
        trigger_mode_ != "continuous")
    {
      RCLCPP_WARN(get_logger(), "未知 trigger_mode '%s'，回退到 auto_once",
        trigger_mode_.c_str());
      trigger_mode_ = "auto_once";
    }

    const auto tag_pose_texts = declare_parameter<std::vector<std::string>>(
      "tag_poses", std::vector<std::string>{});
    for (const auto & text : tag_pose_texts) {
      pm::TagPoseConfig cfg;
      std::string error;
      if (!pm::parseTagPose(text, cfg, error)) {
        RCLCPP_ERROR(get_logger(), "tag_poses 项 '%s' 解析失败: %s",
          text.c_str(), error.c_str());
        continue;
      }
      tag_poses_[cfg.id] = cfg.T_M_tag;
      RCLCPP_INFO(get_logger(), "载入 tag %d 的 map 位姿: %s",
        cfg.id, text.c_str());
    }
    if (tag_poses_.empty()) {
      throw std::runtime_error("tag_poses 为空，请先填写理想地图中的 Tag 绝对位姿");
    }
  }

  // ------------------------------------------------------------ ROS 接口
  void setupRosInterfaces()
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // 带 timeout 的 lookupTransform 依赖 CreateTimerInterface，不设置会抛
    // tf2_ros::CreateTimerInterfaceException。静态外参刚发布时可用于短暂等待。
    const auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      corrected_odom_topic_, rclcpp::QoS(50));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdometry(*msg); });

    // 三个中间结果，供 rviz / 命令行核对位姿链每一步是否正确
    tag_pose_camera_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/tag_pose_camera", 1);
    tag_pose_lidar_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/tag_pose_lidar", 1);
    lidar_pose_map_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/lidar_pose_map", 1);

    // 重新触发一次重定位（例如机器人被搬动后）
    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/relocalize",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        {
          std::lock_guard<std::mutex> lock(samples_mutex_);
          samples_.clear();
          last_sample_ns_ = 0;
          need_publish_.store(true);
        }
        resp->success = true;
        resp->message = "已重新触发 AprilTag 重定位，正在累积检测样本";
        RCLCPP_INFO(get_logger(), "收到 ~/relocalize，重新采集样本");
      });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_update_hz_),
      [this]() { publishOdomCorrection(); });
  }

  void publishCameraStaticTf()
  {
    if (camera_in_parent_.size() != 6U) {
      RCLCPP_ERROR(get_logger(),
        "camera_in_parent 必须含 6 个数 [x,y,z,roll,pitch,yaw]，当前 %zu 个",
        camera_in_parent_.size());
      return;
    }
    const auto & v = camera_in_parent_;
    const auto T = pm::fromRpyDegrees(v[0], v[1], v[2], v[3], v[4], v[5]);

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = camera_tf_parent_;
    msg.child_frame_id = camera_frame_;
    msg.transform.translation.x = T.getOrigin().x();
    msg.transform.translation.y = T.getOrigin().y();
    msg.transform.translation.z = T.getOrigin().z();
    const auto & q = T.getRotation();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    static_broadcaster_->sendTransform(msg);

    RCLCPP_INFO(get_logger(), "发布静态外参 TF: %s -> %s",
      camera_tf_parent_.c_str(), camera_frame_.c_str());
  }

  // ------------------------------------------------------------- 相机初始化
  void setupCamera()
  {
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

    // D435 彩色流通常是已去畸变的 RS2_DISTORTION_NONE；
    // 若是 Brown-Conrady 模型则把 5 个畸变系数交给 solvePnP。
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    if (intr.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY ||
        intr.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
      for (int i = 0; i < 5; ++i) {
        dist_coeffs_.at<double>(i) = intr.coeffs[i];
      }
    }

    RCLCPP_INFO(get_logger(),
      "D435 彩色流 %dx%d@%d, fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      image_width_, image_height_, fps_, intr.fx, intr.fy, intr.ppx, intr.ppy);

    // apriltag 的 p[] 从 tag 右上角开始、绕 tag 逆时针：
    //   p[0]=右上 p[1]=左上 p[2]=左下 p[3]=右下
    // （用 tag36h11 正立合成图实测确认；若按 (-s,-s) 起头，解出的 yaw 会恒定差 180°）
    // 由此确定 tag 坐标系：+x 沿 p1->p0，+y 沿 p3->p0，+z 为码正面法向。
    const float s = tag_size_ / 2.0f;
    object_points_ = {
      cv::Point3f(s, s, 0.0f),
      cv::Point3f(-s, s, 0.0f),
      cv::Point3f(-s, -s, 0.0f),
      cv::Point3f(s, -s, 0.0f),
    };
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

  // ------------------------------------------------------------- 主采集循环
  void captureLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipe_.wait_for_frames(1000U);
      } catch (const rs2::error & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "等待 D435 图像失败: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      // 使用 RealSense 的系统/全局时间；硬件时间域只能以接收时间近似。
      const auto received_at = now();
      const rs2::video_frame color_frame = frames.get_color_frame();
      if (!color_frame) {
        continue;
      }

      rclcpp::Time stamp = received_at;
      if (!get_parameter("use_sim_time").as_bool() &&
          color_frame.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
      {
        stamp = rclcpp::Time(static_cast<int64_t>(color_frame.get_timestamp() * 1e6),
          get_clock()->get_clock_type());
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "相机使用硬件时间/仿真时钟，以图像接收时刻近似采样时间；运动时注意传输延迟");
      }
      cv::Mat bgr(
        cv::Size(color_frame.get_width(), color_frame.get_height()),
        CV_8UC3,
        const_cast<void *>(color_frame.get_data()),
        cv::Mat::AUTO_STEP);
      cv::Mat image = bgr.clone();

      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

      // AprilTag 3.2.0 的 image_u8 中 width/height/stride 是 const 成员，
      // 不能默认构造后逐个赋值，必须一次性聚合初始化（零拷贝包装 OpenCV 数据）。
      image_u8_t img = {
        static_cast<int32_t>(gray.cols),
        static_cast<int32_t>(gray.rows),
        static_cast<int32_t>(gray.step[0]),
        gray.data,
      };

      zarray_t * detections = apriltag_detector_detect(td_, &img);
      bool localized_this_frame = false;
      for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t * det = nullptr;
        zarray_get(detections, i, &det);
        // 一帧只用「第一个有效 tag」产生一个定位样本，避免多 tag 样本混在一起
        if (processDetection(det, image, !localized_this_frame, stamp)) {
          localized_this_frame = true;
        }
      }
      apriltag_detections_destroy(detections);

      if (show_image_) {
        cv::imshow("AprilTag Localization", image);
        if (cv::waitKey(1) == 27) {
          RCLCPP_INFO(get_logger(), "ESC 退出图像预览，节点继续运行");
          show_image_ = false;
          cv::destroyWindow("AprilTag Localization");
        }
      }
    }
  }

  // ------------------------------------------------------------- 单次检测
  // 返回 true 表示本帧已经用这个 tag 产生了一个定位样本
  bool processDetection(
    const apriltag_detection_t * det, cv::Mat & image, bool allow_localization,
    const rclcpp::Time & stamp)
  {
    std::vector<cv::Point2f> image_points;
    image_points.reserve(4);
    for (int j = 0; j < 4; ++j) {
      image_points.emplace_back(
        static_cast<float>(det->p[j][0]), static_cast<float>(det->p[j][1]));
    }

    const cv::Point center(
      static_cast<int>(det->c[0]), static_cast<int>(det->c[1]));

    // 可视化：沿用 D435_apriltag/tag.cpp 的绿色框 + 红色中心点 + ID
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

    if (!allow_localization) {
      return false;
    }

    // 只处理「现场已标定 map 位姿」的 tag
    const auto it = tag_poses_.find(det->id);
    if (it == tag_poses_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "检测到未标定的 tag %d，已忽略（请补进 tag_poses）", det->id);
      return false;
    }
    if (target_tag_id_ >= 0 && det->id != target_tag_id_) {
      return false;
    }

    // ① solvePnP -> T_C_tag：tag 中心相对相机（光学）坐标系的位姿
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(
        object_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec))
    {
      return false;
    }
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec)) {return false;}
    if (tvec.at<double>(2) <= 0.0 || tvec.at<double>(2) > 10.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "tag %d 的 PnP 深度异常 z=%.3f，跳过", det->id, tvec.at<double>(2));
      return false;
    }
    const tf2::Transform T_C_tag = pm::fromSolvePnP(rvec, tvec);
    publishDebugPose(tag_pose_camera_pub_, camera_frame_, T_C_tag);

    if (show_image_) {
      cv::drawFrameAxes(image, camera_matrix_, dist_coeffs_, rvec, tvec, 0.05);
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "X:" << tvec.at<double>(0)
         << " Y:" << tvec.at<double>(1)
         << " Z:" << tvec.at<double>(2);
      cv::putText(image, ss.str(), center + cv::Point(-30, -20),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }

    // ② TF: 相机 -> 雷达外参；以及 雷达 -> base 外参（都是静态 TF）
    tf2::Transform T_L_C, T_L_B;
    if (!lookupTf(lidar_frame_, camera_frame_, T_L_C)) {
      return false;
    }
    if (!lookupTf(lidar_frame_, base_frame_, T_L_B)) {
      return false;
    }

    // ③④⑤⑥⑦ 位姿链 -> T_L_tag / T_M_L / T_M_B
    const auto result = pm::relocalize(T_L_C, T_C_tag, it->second, T_L_B);
    publishDebugPose(tag_pose_lidar_pub_, lidar_frame_, result.T_L_tag);
    publishDebugPose(lidar_pose_map_pub_, map_frame_, result.T_M_L);

    // 防呆：机器人本体在 map 中应当是水平的。若解出很大的 roll/pitch，
    // 几乎总是 tag_poses 的 RPY（尤其贴墙的 ±90°）或相机外参旋转填反了，
    // 此时不更新校正。
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(result.T_M_B.getRotation()).getRPY(roll, pitch, yaw);
    if (std::fabs(roll) > max_tilt_rad_ || std::fabs(pitch) > max_tilt_rad_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "解出的 map->base 姿态异常 roll=%.1f deg pitch=%.1f deg，"
        "请检查 tag_poses 的 RPY 与相机外参旋转，本帧不用于初始化",
        pm::rad2deg(roll), pm::rad2deg(pitch));
      return false;
    }

    const pm::Pose2D pose2d{
      result.T_M_B.getOrigin().x(),
      result.T_M_B.getOrigin().y(),
      yaw,
    };

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "tag %d | 相机系 t=(%.3f,%.3f,%.3f) | 雷达系 t=(%.3f,%.3f,%.3f) | "
      "map 中 base=(%.3f,%.3f,%.1f deg)",
      det->id,
      T_C_tag.getOrigin().x(), T_C_tag.getOrigin().y(), T_C_tag.getOrigin().z(),
      result.T_L_tag.getOrigin().x(), result.T_L_tag.getOrigin().y(),
      result.T_L_tag.getOrigin().z(),
      pose2d.x, pose2d.y, pm::rad2deg(pose2d.yaw));

    if (!need_publish_.load()) {
      return true;
    }
    tf2::Transform T_O_child;
    std::string child;
    if (!odometryAt(stamp, T_O_child, child)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "无图像时刻对应的 %s 里程计，等待同步（检查话题、时间戳与帧名）", odom_topic_.c_str());
      return false;
    }
    tf2::Transform T_child_B;
    if (!lookupTf(child, base_frame_, T_child_B)) {
      return false;
    }
    // 先逐帧反解修正量再平均；机器人运动不会混入校正变换。
    pushSample(result.T_M_B * (T_O_child * T_child_B).inverse(), stamp);
    return true;
  }

  bool lookupTf(
    const std::string & target, const std::string & source,
    tf2::Transform & out)
  {
    // 同一个 frame 时 tf2 无需真实 TF 数据，直接给单位变换
    if (target == source) {
      out = pm::identity();
      return true;
    }
    try {
      const auto msg = tf_buffer_->lookupTransform(
        target, source, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      out = pm::fromTransformMsg(msg);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "TF %s -> %s 不可用: %s", target.c_str(), source.c_str(), ex.what());
      return false;
    }
  }

  void publishDebugPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    const std::string & frame, const tf2::Transform & T)
  {
    if (!publish_debug_poses_ || !pub) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame;
    msg.pose = pm::toPose(T);
    pub->publish(msg);
  }

  static tf2::Transform poseTransform(const geometry_msgs::msg::Pose & p)
  {
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    return tf2::Transform(q.normalized(), tf2::Vector3(p.position.x, p.position.y, p.position.z));
  }

  void onOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose;
    const auto & q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id.empty() ||
        msg.child_frame_id == map_frame_ || msg.child_frame_id == odom_frame_ ||
        !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z) || !std::isfinite(norm) || norm < 1e-12 ||
        rclcpp::Time(msg.header.stamp).nanoseconds() == 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "忽略无效 odom：需要 frame_id=%s、有效 child、非零时间戳及有限位姿",
        odom_frame_.c_str());
      return;
    }
    std::optional<tf2::Transform> correction;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_history_.empty() &&
          (msg.child_frame_id != odom_history_.back().child_frame_id ||
          rclcpp::Time(msg.header.stamp) <= rclcpp::Time(odom_history_.back().header.stamp)))
      {
        RCLCPP_WARN(get_logger(), "odom 时间倒退或 child 改变，忽略本帧；雷达重启后请重启重定位节点");
        return;
      }
      odom_history_.push_back(msg);
      while (odom_history_.size() > 2 &&
          (rclcpp::Time(msg.header.stamp) -
          rclcpp::Time(odom_history_.front().header.stamp)).seconds() > odom_history_sec_)
      {
        odom_history_.pop_front();
      }
      correction = current_M_O_;
    }
    if (!correction) {
      return;  // 未完成定位时不把原始 odom 冒充理想地图位姿。
    }
    auto out = msg;
    out.header.frame_id = map_frame_;
    out.pose.pose = pm::toPose(*correction * poseTransform(msg.pose.pose));
    // Odometry 的 twist 在 child_frame_id 中表达，保持不变。
    // pose 协方差在父系表达，按 diag(R,R) * C * diag(R,R)^T 旋转。
    const auto R = correction->getBasis();
    out.pose.covariance.fill(0.0);
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        for (int a = 0; a < 3; ++a) {
          for (int b = 0; b < 3; ++b) {
            out.pose.covariance[6*i+j] += R[i%3][a] *
              msg.pose.covariance[6*(3*(i/3)+a)+3*(j/3)+b] * R[j%3][b];
          }
        }
      }
    }
    corrected_odom_pub_->publish(out);
  }

  bool odometryAt(const rclcpp::Time & stamp, tf2::Transform & out, std::string & child)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_history_.empty()) {
      return false;
    }
    const auto t = stamp.nanoseconds();
    const auto next = std::lower_bound(odom_history_.begin(), odom_history_.end(), t,
      [](const nav_msgs::msg::Odometry & m, int64_t n) {
        return rclcpp::Time(m.header.stamp).nanoseconds() < n;
      });
    const auto nearest = next == odom_history_.end() ? std::prev(next) : next;
    child = nearest->child_frame_id;
    if (next == odom_history_.begin() || next == odom_history_.end()) {
      if (std::abs(t - rclcpp::Time(nearest->header.stamp).nanoseconds()) * 1e-9 >
          max_sync_error_) {return false;}
      out = poseTransform(nearest->pose.pose);
      return true;
    }
    const auto prev = std::prev(next);
    const auto t0 = rclcpp::Time(prev->header.stamp).nanoseconds();
    const auto t1 = rclcpp::Time(next->header.stamp).nanoseconds();
    if (std::max(t-t0, t1-t) * 1e-9 > max_sync_error_) {return false;}
    const double alpha = static_cast<double>(t-t0) / static_cast<double>(t1-t0);
    const auto A = poseTransform(prev->pose.pose);
    const auto B = poseTransform(next->pose.pose);
    out = tf2::Transform(A.getRotation().slerp(B.getRotation(), alpha),
      A.getOrigin().lerp(B.getOrigin(), alpha));
    return true;
  }

  void pushSample(const tf2::Transform & sample, const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(samples_mutex_);
    if (!need_publish_.load()) {return;}
    if (last_sample_ns_ != 0 &&
        (stamp.nanoseconds()-last_sample_ns_) * 1e-9 > max_sample_gap_)
    {
      samples_.clear();
    }
    last_sample_ns_ = stamp.nanoseconds();
    samples_.push_back(sample);
    if (static_cast<int>(samples_.size()) < min_samples_) {return;}
    tf2::Vector3 translation(0, 0, 0);
    tf2::Quaternion rotation(0, 0, 0, 0);
    const auto reference = samples_.front().getRotation();
    for (const auto & s : samples_) {
      translation += s.getOrigin();
      auto q = s.getRotation();
      if (q.dot(reference) < 0.0) {q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());}
      rotation += q;
    }
    translation /= static_cast<double>(samples_.size());
    if (rotation.length2() < 1e-12) {samples_.clear(); return;}
    rotation.normalize();
    bool valid = true;
    for (const auto & s : samples_) {
      valid = valid && (s.getOrigin()-translation).length() <= max_spread_ &&
        rotation.angleShortestPath(s.getRotation()) <= max_rotation_spread_;
    }
    samples_.clear();
    if (!valid) {
      RCLCPP_WARN(get_logger(), "校正样本位置/角度离散过大，丢弃本轮");
      return;
    }
    {
      std::lock_guard<std::mutex> odom_lock(odom_mutex_);
      current_M_O_ = tf2::Transform(rotation, translation);
    }
    if (trigger_mode_ != "continuous") {need_publish_.store(false);}
    RCLCPP_INFO(get_logger(), "已对齐 %s -> %s：平移=(%.3f, %.3f, %.3f)，yaw=%.2f deg",
      map_frame_.c_str(), odom_frame_.c_str(), translation.x(), translation.y(),
      translation.z(), pm::rad2deg(tf2::getYaw(rotation)));
  }

  void publishOdomCorrection()
  {
    std::optional<tf2::Transform> correction;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      correction = current_M_O_;
    }
    if (!correction) {return;}
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = odom_frame_;
    const auto p = pm::toPose(*correction);
    msg.transform.translation.x = p.position.x;
    msg.transform.translation.y = p.position.y;
    msg.transform.translation.z = p.position.z;
    msg.transform.rotation = p.orientation;
    tf_broadcaster_->sendTransform(msg);
  }

  // ---------------------------------------------------------------- 成员
  std::string map_frame_, base_frame_, lidar_frame_, camera_frame_;
  std::string trigger_mode_, camera_tf_parent_;
  std::string odom_frame_, odom_topic_, corrected_odom_topic_;
  double tf_update_hz_{30.0}, odom_history_sec_{5.0}, max_sync_error_{0.1};
  double max_rotation_spread_{0.1}, max_sample_gap_{0.5};
  int64_t last_sample_ns_{0};
  float tag_size_{0.118f};
  int target_tag_id_{-1}, min_samples_{10};
  int image_width_{640}, image_height_{480}, fps_{30}, nthreads_{4};
  double max_spread_{0.08}, tf_timeout_{0.2}, max_tilt_rad_{0.52};
  double quad_decimate_{1.0}, quad_sigma_{0.0};
  bool publish_debug_poses_{true};
  bool show_image_{false}, refine_edges_{true}, publish_camera_tf_{false};
  std::vector<double> camera_in_parent_;

  std::map<int, tf2::Transform> tag_poses_;   // id -> T_M_tag
  std::vector<cv::Point3f> object_points_;
  cv::Mat camera_matrix_, dist_coeffs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    tag_pose_camera_pub_, tag_pose_lidar_pub_, lidar_pose_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

  // 理想地图 -> 现实 odom 广播
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::mutex odom_mutex_;
  std::optional<tf2::Transform> current_M_O_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;

  rs2::pipeline pipe_;
  apriltag_family_t * tf_{nullptr};
  apriltag_detector_t * td_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<bool> need_publish_{false};
  std::mutex samples_mutex_;
  std::vector<tf2::Transform> samples_;
  std::thread capture_thread_;
};

}  // namespace apriltag_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<apriltag_localization::AprilTagLocalizer>(
      rclcpp::NodeOptions());
    node->start();
    rclcpp::spin(node);
  } catch (const rs2::error & e) {
    RCLCPP_FATAL(rclcpp::get_logger("apriltag_localization"),
      "D435 初始化失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("apriltag_localization"),
      "节点初始化失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
