# MuJoCo AprilTag ID 5 闭环验收

验收日期：2026-11-12。机器人：`atdog2`；场景：`scene_tag`；标签：tag36h11 / ID 5。
使用项目现有策略、真实 MuJoCo 动力学、机身相机渲染、ROS 图像传输和 AprilTag + PnP。
仿真真值仅用于误差判定，不进入视觉控制器。

## 最终配置结果

| 测试 | 启动趋近至到位 / s | 最终平面误差 / cm | 最终偏航误差 / ° | 保持 10 s 的最大漂移 / mm |
| --- | ---: | ---: | ---: | ---: |
| nominal_1 | 5.56 | 3.89 | 0.67 | 0.50 |
| nominal_2 | 10.64 | 3.83 | -0.12 | 0.82 |
| interrupted | 13.63 | 2.67 | -0.33 | 6.15 |

三轮最终配置测试全部通过。`interrupted` 在前进中暂停识别进程 3 秒：
通信看门狗切换位控站立，最后一秒漂移 0.71 mm；恢复识别后自动继续并到位。
三轮均检查了 `/stop` 后继续保持位置，且最终速度为零、模式为站立。

验收门槛：最终平面误差 <5 cm，偏航误差 <5°，站立保持 10 秒最大漂移 <15 mm；
机体高度 >0.18 m，运动阶段倾角 <20°；识别进程中断后能站稳并恢复。
目标机体位置为世界系 `(1.4, 0)`，即距标签平面 0.6 m。
只验收平面站位和偏航，四足的实际站立高度由底层姿态决定；未声称六维位置均达到目标。

## 本轮修复

- MuJoCo 按键按事件消耗，消除恢复上一次按键导致视觉控制重复开关的问题。
- 相机渲染与像素读回不占用物理锁，减少控制延迟。
- 控制与推理按仿真时间调度，PD 力矩每个物理子步刷新。
- 标签场景改用 `implicitfast` + 1 ms 步长，消除高增益站立下出现的数值失稳。
- 到位、丢码、通信超时及 `/stop` 切换位控站立，避免零速策略漂移。
- 仿真到位设置制动滞回和目标锁定；调用 `/start` 重新启动下一次趋近。

早期测试曾出现重复切换、零速漂移与站立失稳，均未作为通过结果。
最终数据来自上述修复后连续两轮正常测试及一轮中断恢复测试。

## 复现

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
  python3 scripts/apriltag/validate_mujoco_approach.py --output /tmp/tag_check/normal
LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
  python3 scripts/apriltag/validate_mujoco_approach.py --interrupt --output /tmp/tag_check/interrupted
```

每轮目录包含 `result.json`（断言与完整轨迹）、`process.log`、`initial.png` 和 `final.png`。
`validation.png` 对比三轮的真值位置误差、偏航与输出速度。

本报告证明默认 `atdog2` 平地场景的以上行为；`atdog3` 仅保留接入，未做同等动力学验收。
复杂地形、遮挡后的主动搜索和真机行为不在本轮验收范围内。
