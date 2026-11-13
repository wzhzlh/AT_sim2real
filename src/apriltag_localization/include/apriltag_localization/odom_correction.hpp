#ifndef APRILTAG_LOCALIZATION__ODOM_CORRECTION_HPP_
#define APRILTAG_LOCALIZATION__ODOM_CORRECTION_HPP_

// =====================================================================
//  output_mode=tf 时的 odom 校正数学（纯数学，不依赖 rclcpp）
//
//  记号同 pose_math.hpp：T_A_B 表示「把 B 系坐标变换到 A 系」的齐次变换。
//
//     T_M_B(视觉) : AprilTag 解算出的 base 在 map 中的绝对位姿
//     T_O_B(里程) : TF(odom -> base_link)，由底盘里程计给出（会漂移）
//     T_M_O       = T_M_B * (T_O_B)^-1
//
//  广播 map->odom 之后：
//     T_M_B(TF 合成) = T_M_O * T_O_B ≡ T_M_B(视觉)
//  即机器人被视觉钉在地图的绝对位置上，里程计只负责帧间增量，
//  它的漂移全部被 map->odom 吸收。
//
//  两个细节：
//    * planarize：机器人本体假设水平，map->odom 只保留 x/y/yaw，
//      避免里程计的 z / roll / pitch 把 base_link 拉歪。
//    * stepToward：单步限幅，修正量分多帧逼近目标，避免 TF 突跳。
// =====================================================================

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include "apriltag_localization/pose_math.hpp"

namespace apriltag_localization::pose_math
{

// 只保留平面分量：z=0，姿态只留 yaw
inline tf2::Transform planarize(const tf2::Transform & T)
{
  tf2::Transform out;
  out.setOrigin(tf2::Vector3(T.getOrigin().x(), T.getOrigin().y(), 0.0));
  out.setRotation(quaternionFromYaw(yawOf(T)));
  return out;
}

// 从 from 向 to 走一步，平移/旋转各自限幅；max_* <= 0 表示不限幅（一步到位）
inline tf2::Transform stepToward(
  const tf2::Transform & from, const tf2::Transform & to,
  double max_trans, double max_rot)
{
  tf2::Vector3 d = to.getOrigin() - from.getOrigin();
  d.setZ(0.0);
  if (max_trans > 0.0 && d.length() > max_trans) {
    d = d.normalized() * max_trans;
  }
  double dyaw = std::remainder(yawOf(to) - yawOf(from), 2.0 * M_PI);
  if (max_rot > 0.0 && std::fabs(dyaw) > max_rot) {
    dyaw = (dyaw > 0.0) ? max_rot : -max_rot;
  }
  tf2::Transform out;
  out.setOrigin(tf2::Vector3(
    from.getOrigin().x() + d.x(), from.getOrigin().y() + d.y(), 0.0));
  out.setRotation(quaternionFromYaw(yawOf(from) + dyaw));
  return out;
}

// 视觉绝对位姿 + 里程计位姿 -> 应该广播的 map->odom
inline tf2::Transform mapToOdom(
  const tf2::Transform & T_M_B, const tf2::Transform & T_O_B)
{
  return planarize(planarize(T_M_B) * planarize(T_O_B).inverse());
}

// 自检辅助：广播 map->odom 之后 TF 合成出来的 base 在 map 中的位姿
//   T_M_B' = T_M_O * T_O_B
// 正常情况下应当等于输入的 T_M_B(视觉)。
inline tf2::Transform composeBaseInMap(
  const tf2::Transform & T_M_O, const tf2::Transform & T_O_B)
{
  return T_M_O * T_O_B;
}

}  // namespace apriltag_localization::pose_math

#endif  // APRILTAG_LOCALIZATION__ODOM_CORRECTION_HPP_
