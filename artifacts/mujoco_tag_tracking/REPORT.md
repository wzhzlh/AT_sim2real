# MuJoCo 0/1 按键与持续 Tag 跟踪验收

验证对象：atdog2、当前 robot_lab 策略、tag36h11 / ID 5。识别和控制读取相机图像，机体真值仅用于验收。

## 使用

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch apriltag_localization tag_approach.launch.py use_mujoco:=true
```

点击 MuJoCo 窗口后，按 0 起身站稳，按 1 开始视觉趋近并自动切换 RL 行走。
到位后采用位控站立，视觉识别与误差评估持续运行；平面偏差超过 8 cm 或偏航超过 8° 时恢复 RL 调整。
按 0 关闭自动行走并站立，画面仍继续检测；再次按 1 恢复。无需按 N 或手动调用服务。

## 完整流程结果

`final_follow/result.json`：passed=true，15 项检查全部通过。

| 检查 | 实测 |
| --- | --- |
| 0 站立、无自动前进 | 通过 |
| 1 启动视觉控制和 RL 行走 | 通过 |
| 首次目标处平面误差 | 3.99 cm |
| 首次目标处偏航误差 | 0.53° |
| 首次到位后保持 10 秒最大漂移 | 0.31 mm |
| 标签前移 30 cm 后自动恢复 RL 并到位 | 通过 |
| 新目标处平面误差 | 3.57 cm |
| 新目标处保持 10 秒最大漂移 | 0.37 mm |
| 按 0 后再次移动标签，机器人保持站立 | 通过 |

到位进入阈值为 4 cm，制动保持阶段允许 8 cm，恢复调整阈值为 8 cm。
行走切换站立会产生机体位移，验收最终平面误差采用 7 cm。此前一轮中断测试的稳定误差为 6.15 cm，未满足当时 6 cm 验收线，不计为通过。
最终成功轮的误差均小于 4 cm。以上结果不代表 atdog3 的动力学已经验证。

复现命令（需要 X11 桌面）：

```bash
DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
python3 scripts/apriltag/validate_mujoco_approach.py --move-tag --output /tmp/tag_validation/following
```

完整轨迹、速度、站立指令与日志保存在 `final_follow/`，初始和最终相机图像也已保存。

## 额外故障注入的限制

`interrupted_failed/` 保留了强制 SIGSTOP 识别进程 3 秒再 SIGCONT 的失败记录。
看门狗正常切到站立，检测到的站立漂移为 1.80 mm；进程恢复后未再获得有效视觉结果，未完成后续趋近。
原因尚未确定，不能声称支持任意进程冻结后的自动恢复。正常运行中的持续跟随流程已通过；强制进程暂停恢复尚未通过。

MuJoCo 目标构建、apriltag_localization 构建、Python 语法检查和 git diff --check 已通过。
