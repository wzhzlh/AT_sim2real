// =====================================================================
//  apriltag_localization / apriltag_localizer
//
//  来源：~/qq_down/D435_apriltag/tag.cpp（C++ 版 D435 + AprilTag + solvePnP）
//  改造：把「打印 tag 相对相机位姿」升级为 ROS2 视觉重定位节点
//
//    1) RealSense D435 取图，apriltag 检测，cv::solvePnP 得到 T_C_tag
//    2) TF: camera -> lidar，得到雷达系下的目标位姿 T_L_tag
//    3) 结合人工标定的 tag 场地位置 T_M_tag，反算 T_M_L / T_M_B
//    4) 发布 geometry_msgs/PoseWithCovarianceStamped 到 /initialpose
//    5) AMCL 收到初值后用激光 scan 与地图匹配继续修正位姿
//
//  详细推导与操作步骤见本包 teach.md
// =====================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
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
      "AprilTag 重定位节点已启动 | map=%s base=%s lidar=%s camera=%s | "
      "tag_size=%.4f m | 已加载 %zu 个 tag 标定值 | trigger_mode=%s",
      map_frame_.c_str(), base_frame_.c_str(), lidar_frame_.c_str(),
      camera_frame_.c_str(), tag_size_, tag_poses_.size(), trigger_mode_.c_str());
    RCLCPP_INFO(get_logger(),
      "等待有效检测；%s 后把解算的 map 位姿发布到 /initialpose",
      trigger_mode_ == "manual" ? "调用 ~/relocalize 服务" : "累积到 min_samples 帧");
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
    require_subscriber_ = declare_parameter<bool>("require_subscriber", true);

    sigma_x_ = declare_parameter<double>("sigma_x", 0.15);
    sigma_y_ = declare_parameter<double>("sigma_y", 0.15);
    sigma_yaw_ = declare_parameter<double>("sigma_yaw", 0.10);

    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.2);
    max_tilt_rad_ = pm::deg2rad(declare_parameter<double>("max_tilt_deg", 30.0));
    publish_debug_poses_ = declare_parameter<bool>("publish_debug_poses", true);
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
      RCLCPP_WARN(get_logger(),
        "参数 tag_poses 为空，无法解算 map 位姿！"
        "请在 config/localizer.yaml 中按 id,x,y,z,roll,pitch,yaw 填写现场标定值");
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

    // /initialpose 是 nav2_amcl 的标准初值接口
    initial_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

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
        }
        need_publish_.store(true);
        resp->success = true;
        resp->message = "已重新触发 AprilTag 重定位，正在累积检测样本";
        RCLCPP_INFO(get_logger(), "收到 ~/relocalize，重新采集样本");
      });
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

    // 与 D435_apriltag/tag.cpp 完全一致的角点约定：
    // p[0]->(-s,-s), p[1]->(s,-s), p[2]->(s,s), p[3]->(-s,s)
    // 由此确定 tag 坐标系：+x 沿 p0->p1，+y 沿 p0->p3，+z 为码正面法向。
    const float s = tag_size_ / 2.0f;
    object_points_ = {
      cv::Point3f(-s, -s, 0.0f),
      cv::Point3f(s, -s, 0.0f),
      cv::Point3f(s, s, 0.0f),
      cv::Point3f(-s, s, 0.0f),
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

      const rs2::video_frame color_frame = frames.get_color_frame();
      if (!color_frame) {
        continue;
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
        if (processDetection(det, image, !localized_this_frame)) {
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
    const apriltag_detection_t * det, cv::Mat & image, bool allow_localization)
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
    // 此时绝不把错误初值发给 AMCL。
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

    pushSample(pose2d);
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

  // ------------------------------------------------------------- 样本与发布
  void pushSample(const pm::Pose2D & sample)
  {
    std::optional<pm::SampleResult> ready;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      samples_.push_back(sample);
      if (static_cast<int>(samples_.size()) >= min_samples_) {
        const auto res = pm::summarize(samples_, max_spread_);
        samples_.clear();
        if (res.valid) {
          ready = res;
        } else {
          RCLCPP_WARN(get_logger(),
            "样本离散度 %.3f m > max_spread %.3f m（机器人可能在动或检测抖动），本轮丢弃",
            res.spread, max_spread_);
        }
      }
    }

    if (ready && need_publish_.load()) {
      publishInitialPose(*ready);
    }
  }

  void publishInitialPose(const pm::SampleResult & res)
  {
    // AMCL 必须先启动并订阅 /initialpose，否则这条一次性消息会丢失
    if (require_subscriber_ && initial_pose_pub_->get_subscription_count() == 0U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "尚无节点订阅 /initialpose（AMCL 未启动？），暂不发布初值");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = res.mean.x;
    msg.pose.pose.position.y = res.mean.y;
    msg.pose.pose.position.z = 0.0;
    const auto q = pm::quaternionFromYaw(res.mean.yaw);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // 协方差是方差（标准差的平方）；AMCL 只用 x/y/yaw
    msg.pose.covariance = pm::makeCovariance(
      sigma_x_ * sigma_x_, sigma_y_ * sigma_y_, sigma_yaw_ * sigma_yaw_);

    initial_pose_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "已发布 /initialpose: x=%.3f y=%.3f yaw=%.2f deg (基于 %zu 帧, 离散度 %.3f m)"
      " -> 交给 AMCL 用激光 scan 与地图匹配继续修正",
      res.mean.x, res.mean.y, pm::rad2deg(res.mean.yaw),
      res.count, res.spread);

    // 只初始化一次，之后不再重置 AMCL，让它靠激光匹配自主收敛
    if (trigger_mode_ != "continuous") {
      need_publish_.store(false);
      RCLCPP_INFO(get_logger(),
        "自动初始化已完成；如需再次重定位：ros2 service call "
        "/apriltag_localizer/relocalize std_srvs/srv/Trigger");
    }
  }

  // ---------------------------------------------------------------- 成员
  std::string map_frame_, base_frame_, lidar_frame_, camera_frame_;
  std::string trigger_mode_, camera_tf_parent_;
  float tag_size_{0.118f};
  int target_tag_id_{-1}, min_samples_{10};
  int image_width_{640}, image_height_{480}, fps_{30}, nthreads_{4};
  double max_spread_{0.08}, tf_timeout_{0.2}, max_tilt_rad_{0.52};
  double sigma_x_{0.15}, sigma_y_{0.15}, sigma_yaw_{0.10};
  double quad_decimate_{1.0}, quad_sigma_{0.0};
  bool require_subscriber_{true}, publish_debug_poses_{true};
  bool show_image_{false}, refine_edges_{true}, publish_camera_tf_{false};
  std::vector<double> camera_in_parent_;

  std::map<int, tf2::Transform> tag_poses_;   // id -> T_M_tag
  std::vector<cv::Point3f> object_points_;
  cv::Mat camera_matrix_, dist_coeffs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    tag_pose_camera_pub_, tag_pose_lidar_pub_, lidar_pose_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

  rs2::pipeline pipe_;
  apriltag_family_t * tf_{nullptr};
  apriltag_detector_t * td_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<bool> need_publish_{false};
  std::mutex samples_mutex_;
  std::vector<pm::Pose2D> samples_;
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
