#ifndef APRILTAG_LOCALIZATION__POSE_MATH_HPP_
#define APRILTAG_LOCALIZATION__POSE_MATH_HPP_

// =====================================================================
//  AprilTag 视觉重定位的「纯数学」部分（不依赖 rclcpp，便于单独验算）
//
//  记号约定（全文一致）：
//    T_A_B 表示「把 B 坐标系下的坐标变换到 A 坐标系」的齐次变换，
//           即  p_A = T_A_B * p_B
//    组合规则： T_A_C = T_A_B * T_B_C
//    求逆：     T_B_A = T_A_B.inverse()
//
//  位姿链（详见 teach.md「3. 数学推导」）：
//    ① T_C_tag   : cv::solvePnP 直接给出，tag 相对「相机光学系」的位姿
//    ② T_L_C     : TF 查 lookupTransform(lidar_frame, camera_frame)
//    ③ T_L_tag   : = ② * ①              （雷达系下的目标位姿）
//    ④ T_M_tag   : 人工标定的 tag 在场地 map 中的位姿（参数 tag_poses）
//    ⑤ T_M_L     : = ④ * (③).inverse()  （雷达在 map 中的位姿，重定位结果）
//    ⑥ T_L_B     : TF 查 lookupTransform(lidar_frame, base_frame)
//    ⑦ T_M_B     : = ⑤ * ⑥              （AMCL 需要的 base 在 map 中的位姿）
//    ⑧ /initialpose : 从 ⑦ 取 x, y, yaw（AMCL 是 2D 定位，忽略 z/roll/pitch）
//
//  帧说明：
//    M = map, B = base_link, L = laser(雷达), C = camera_xxx_optical_frame,
//    tag = AprilTag 自身坐标系（原点在码中心，+z 为码正面法向）
// =====================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace apriltag_localization::pose_math
{

inline double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

inline double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

// ---- 基本构造 ---------------------------------------------------------

// ROS TF 消息 -> tf2::Transform。
// 注意 lookupTransform(target, source) 返回的正是 T_target_source。
inline tf2::Transform fromTransformMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  const auto & t = msg.transform.translation;
  const auto & r = msg.transform.rotation;
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(t.x, t.y, t.z));
  T.setRotation(tf2::Quaternion(r.x, r.y, r.z, r.w));
  return T;
}

// 由「位置 + RPY(角度制, 内旋 ZYX)」构造变换，用于填写 tag 在 map 中的位姿。
inline tf2::Transform fromRpyDegrees(
  double x, double y, double z,
  double roll_deg, double pitch_deg, double yaw_deg)
{
  tf2::Quaternion q;
  q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(x, y, z));
  T.setRotation(q);
  return T;
}

// cv::solvePnP 的 rvec/tvec -> T_C_tag。
// rvec 是旋转向量（Rodrigues），tvec 是 tag 原点在相机系下的平移。
inline tf2::Transform fromSolvePnP(const cv::Mat & rvec, const cv::Mat & tvec)
{
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  const tf2::Matrix3x3 rot(
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

  tf2::Quaternion q;
  rot.getRotation(q);
  q.normalize();

  tf2::Transform T;
  T.setOrigin(tf2::Vector3(
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
  T.setRotation(q);
  return T;
}

inline tf2::Transform identity()
{
  tf2::Transform T;
  T.setIdentity();
  return T;
}

inline tf2::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

inline double yawOf(const tf2::Transform & T)
{
  return tf2::getYaw(T.getRotation());
}

inline geometry_msgs::msg::Pose toPose(const tf2::Transform & T)
{
  geometry_msgs::msg::Pose p;
  const auto & o = T.getOrigin();
  p.position.x = o.x();
  p.position.y = o.y();
  p.position.z = o.z();
  const auto & q = T.getRotation();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// PoseWithCovarianceStamped.covariance 是 6x6 行主序数组，顺序 x,y,z,roll,pitch,yaw。
// AMCL 只用 x/y/yaw，其余轴给大方差表示「不提供信息」。
inline std::array<double, 36> makeCovariance(double var_x, double var_y, double var_yaw)
{
  std::array<double, 36> cov{};
  const double unknown = 1e6;
  cov[0] = var_x;      // (0,0) x
  cov[7] = var_y;      // (1,1) y
  cov[14] = unknown;   // (2,2) z
  cov[21] = unknown;   // (3,3) roll
  cov[28] = unknown;   // (4,4) pitch
  cov[35] = var_yaw;   // (5,5) yaw
  return cov;
}

// ---- 位姿链 -----------------------------------------------------------

// 输入：T_L_C（相机->雷达外参）、T_C_tag（PnP 结果）、
//       T_M_tag（tag 场地标定值）、T_L_B（雷达->base 外参）
// 输出：T_L_tag（雷达系下的目标位姿，中间结果）、
//       T_M_L（雷达 map 位姿）、T_M_B（base map 位姿，给 /initialpose）
struct RelocalizationResult
{
  tf2::Transform T_L_tag;
  tf2::Transform T_M_L;
  tf2::Transform T_M_B;
};

inline RelocalizationResult relocalize(
  const tf2::Transform & T_L_C,
  const tf2::Transform & T_C_tag,
  const tf2::Transform & T_M_tag,
  const tf2::Transform & T_L_B)
{
  RelocalizationResult out;
  out.T_L_tag = T_L_C * T_C_tag;                    // ③
  out.T_M_L = T_M_tag * out.T_L_tag.inverse();     // ⑤
  out.T_M_B = out.T_M_L * T_L_B;                   // ⑦
  return out;
}

// ---- 多样本统计 -------------------------------------------------------

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SampleResult
{
  bool valid{false};
  Pose2D mean;
  double spread{0.0};   // 平移样本相对均值的最大偏差 [m]
  size_t count{0};
};

// 对连续若干帧检测结果取均值；yaw 必须在圆上取平均（sin/cos），
// 否则 ±180° 跳变会算出错误朝向。样本离散过大说明机器人在动或检测抖动，
// 此时 valid=false，不发布初值。
inline SampleResult summarize(const std::vector<Pose2D> & samples, double max_spread)
{
  SampleResult out;
  out.count = samples.size();
  if (samples.empty()) {
    return out;
  }

  double sx = 0.0, sy = 0.0, sc = 0.0, ss = 0.0;
  for (const auto & s : samples) {
    sx += s.x;
    sy += s.y;
    sc += std::cos(s.yaw);
    ss += std::sin(s.yaw);
  }
  const double n = static_cast<double>(samples.size());
  out.mean.x = sx / n;
  out.mean.y = sy / n;
  out.mean.yaw = std::atan2(ss, sc);

  for (const auto & s : samples) {
    out.spread = std::max(
      out.spread, std::hypot(s.x - out.mean.x, s.y - out.mean.y));
  }
  out.valid = (max_spread <= 0.0) || (out.spread <= max_spread);
  return out;
}

// ---- tag 场地标定表解析 -----------------------------------------------

// 每个元素格式："id,x,y,z,roll,pitch,yaw"（角度制，单位 m / deg）
struct TagPoseConfig
{
  int id{-1};
  tf2::Transform T_M_tag;
};

inline bool parseTagPose(
  const std::string & text, TagPoseConfig & out, std::string & error)
{
  std::vector<double> v;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      v.push_back(std::stod(item));
    } catch (const std::exception &) {
      error = "无法解析数字 '" + item + "'";
      return false;
    }
  }
  if (v.size() != 7U) {
    error = "需要 7 个字段 id,x,y,z,roll,pitch,yaw，实际 " +
            std::to_string(v.size()) + " 个";
    return false;
  }
  out.id = static_cast<int>(v[0]);
  out.T_M_tag = fromRpyDegrees(v[1], v[2], v[3], v[4], v[5], v[6]);
  return true;
}

// ---- 4x4 齐次矩阵（行主序 16 个数）--------------------------------
//
// 测试阶段不依赖 TF 树：人手标定的常数位姿直接以 4x4 矩阵写进参数文件。
//   [ r00 r01 r02 tx ]
//   [ r10 r11 r12 ty ]
//   [ r20 r21 r22 tz ]
//   [  0   0   0   1 ]
// 解析时检查最后一行、旋转部分的正交性与 det=+1，尽早发现矩阵填反/含镜像。
inline bool fromMatrix4x4(
  const std::vector<double> & m, tf2::Transform & out, std::string & error)
{
  if (m.size() != 16U) {
    error = "4x4 矩阵需要 16 个数（行主序），实际 " +
            std::to_string(m.size()) + " 个";
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(m[12 + i]) > 1e-6) {
      error = "最后一行前三个数必须为 0";
      return false;
    }
  }
  if (std::fabs(m[15] - 1.0) > 1e-6) {
    error = "最后一行第四个数必须为 1";
    return false;
  }

  const tf2::Matrix3x3 R(
    m[0], m[1], m[2],
    m[4], m[5], m[6],
    m[8], m[9], m[10]);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double dot = R[0][i] * R[0][j] + R[1][i] * R[1][j] +
                         R[2][i] * R[2][j];
      if (std::fabs(dot - (i == j ? 1.0 : 0.0)) > 1e-3) {
        error = "左上 3x3 不是正交旋转矩阵（请检查是否写成平移/缩放/转置）";
        return false;
      }
    }
  }
  const double det =
    R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
  if (std::fabs(det - 1.0) > 1e-3) {
    error = "旋转部分行列式不为 +1（坐标系含镜像，请检查轴方向）";
    return false;
  }

  tf2::Quaternion q;
  R.getRotation(q);
  q.normalize();
  out.setOrigin(tf2::Vector3(m[3], m[7], m[11]));
  out.setRotation(q);
  return true;
}

// tf2::Transform -> 16 个数，用于 ~/record_goal 把示教结果打印回参数文件。
inline std::vector<double> toMatrix4x4(const tf2::Transform & T)
{
  const tf2::Matrix3x3 R(T.getRotation());
  const auto & o = T.getOrigin();
  return {
    R[0][0], R[0][1], R[0][2], o.x(),
    R[1][0], R[1][1], R[1][2], o.y(),
    R[2][0], R[2][1], R[2][2], o.z(),
    0.0, 0.0, 0.0, 1.0,
  };
}

inline std::string matrixToString(const std::vector<double> & m)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  for (int r = 0; r < 4; ++r) {
    ss << "  [";
    for (int c = 0; c < 4; ++c) {
      ss << std::setw(11) << m[r * 4 + c];
      if (c < 3) {
        ss << ",";
      }
    }
    ss << "]";
    if (r < 3) {
      ss << "\n";
    }
  }
  return ss.str();
}

}  // namespace apriltag_localization::pose_math

#endif  // APRILTAG_LOCALIZATION__POSE_MATH_HPP_
