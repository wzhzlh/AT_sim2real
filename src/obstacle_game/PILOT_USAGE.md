> 裁剪说明：keyboard、task_game、obstacle_game_ui 已移除。下文涉及这些模块及旧版 Record 的说明属于历史内容；当前 *_record.launch.py 仅启动手动控制和遥控节点，不提供路径录制。

# obstacle_game 自动导航与路径录制使用说明

## 1. 功能概述

`obstacle_game` 支持两种主要控制方式：

- **手动模式**：遥控器摇杆直接控制机器人速度，并可切换不同运动策略。
- **自动模式**：`Pilot` 读取 YAML 路径点，按照顺序自动导航，并发布 `robot_move_cmd`。

当前 `Pilot` 已适配更完整的轨迹算法，支持：

- 多路径点顺序执行
- 梯形 / 三角速度规划
- 前馈速度 + 位置误差反馈
- 最终位置微调
- 可选最终朝向约束
- 可选横移控制
- 可选三次多项式轨迹衔接
- 可选到点位控站立等待
- 旧版 YAML 路径文件兼容
- 新版 Record 录制字段兼容

---

## 2. 构建

当前工作区如果是 isolated install 布局，使用：

```bash
colcon build --symlink-install --packages-select obstacle_game
```

如果需要连同依赖一起构建：

```bash
colcon build --symlink-install --packages-select robot_msgs remote_node obstacle_game
```

构建完成后加载环境：

```bash
source install/setup.bash
```



可以直接这样调，atdog2 和 atdog3 的流程基本一样，只是
  机器人参数不同。

  先跑 MuJoCo：

  ./cmake_build/bin/rl_sim_mujoco atdog2 scene
  ./cmake_build/bin/rl_sim_mujoco atdog3 scene

  进入后用键盘调试最稳：

  - 0：站起来
  - R：重置仿真
  - Enter：暂停/继续
  - P：回 Passive

─ Worked for 3m 18s ────────────────────────────────────

 
› Summarize recent commits
 
  gpt-5.4 high · ~/RL/AT_robot-lab





---

## 3. 启动方式

### 3.1 使用自动路径选择启动

```bash
ros2 run obstacle_game robot_control
```

默认情况下，`scene_path` 为空，程序会自动选择要执行的 YAML 文件。

自动选择顺序：

1. 如果启动参数显式传入了 `scene_path`，直接使用该文件。
2. 如果没有传入 `scene_path`，优先查找录制目录下的：
   ```text
   obstacle_game.yaml
   ```
3. 如果没有 `obstacle_game.yaml`，则查找录制目录下最新的：
   ```text
   record*.yaml
   ```
4. 如果都没有找到，Pilot 轨迹为空，自动模式下触发开始会失败。

这里“最新”按文件名排序判断，因为 Record 生成的文件名格式是：

```text
recordYYYYMMDD_HHMMSS.yaml
```

文件名越大，时间越新。

### 3.2 推荐使用方式

平时录制会生成类似：

```text
record20260619_153000.yaml
```

测试后如果这条路径效果比较好，可以手动改名为：

```text
obstacle_game.yaml
```

之后直接启动：

```bash
ros2 run obstacle_game robot_control
```

程序会自动优先使用 `obstacle_game.yaml`。

### 3.3 指定路径 YAML 启动

如果临时想测试某个指定文件，可以显式传入：

```bash
ros2 run obstacle_game robot_control --ros-args \
  -p scene_path:=/home/qi/AT_DOG/AT_robot-lab/record20260619_142926.yaml
```

只要 `scene_path` 非空，就不会走自动查找逻辑。

### 3.4 指定录制输出前缀

```bash
ros2 run obstacle_game robot_control --ros-args \
  -p yaml_file_path:=/home/qi/AT_DOG/AT_robot-lab/record
```

录制生成的文件名类似：

```text
/home/qi/AT_DOG/AT_robot-lab/record20260619_153000.yaml
```

自动查找路径时，会根据 `yaml_file_path` 的父目录确定搜索目录。以上面的参数为例，搜索顺序是：

```text
/home/qi/AT_DOG/AT_robot-lab/obstacle_game.yaml
/home/qi/AT_DOG/AT_robot-lab/record*.yaml 中文件名最新的文件
```

如果 `yaml_file_path` 使用默认值：

```text
./trajectory/record
```

则搜索顺序是：

```text
./trajectory/obstacle_game.yaml
./trajectory/record*.yaml 中文件名最新的文件
```

---

## 4. 运行前检查

自动导航依赖 TF 位姿，运行前确认存在：

```text
map -> base_link
```

可用以下命令检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

同时确认遥控器节点正常发布：

```bash
ros2 topic echo /remote
```

确认控制命令输出：

```bash
ros2 topic echo /robot_move_cmd
```

---

## 5. 遥控器操作

### 5.1 模式位

| Bit | 功能 | 说明 |
|---:|---|---|
| `bit 1` | 自动模式 | 为 1 时进入自动模式；为 0 时为手动模式 |
| `bit 2` | 录制模式 | 为 1 时开始录制；松开后结束录制 |

### 5.2 手动模式操作

手动模式下，摇杆直接控制速度：

| 输入 | Cmd 字段 | 当前限幅 |
|---|---|---|
| `ly` | `cmd.vx` | `[-1.2, 1.2]` |
| `lx` | `cmd.vy` | `[-0.8, 0.8]` |
| `rx` | `cmd.vz` | `[-1.0, 1.0]` |

手动模式下的策略按键：

| Bit | 功能 | `cmd.mode` | 录制为 `policy_id` |
|---:|---|---:|---:|
| `bit 4` | 位控站立 | 1 | 1 |
| `bit 5` | 普通行走 | 2 | 2 |
| `bit 6` | 台阶策略 | 3 | 3 |
| `bit 3` | 沙地策略 | 4 | 4 |
| `bit 11` | 斜坡策略 | 5 | 5 |
| `bit 12` | 限高杆策略 | 6 | 6 |
| `bit 13` | 木桥策略 | 7 | 7 |
| `bit 10` | 翻墙策略 | 8 | 8 |

### 5.3 自动模式操作

进入自动模式后：

| Bit | 功能 |
|---:|---|
| `bit 4` | 复位并停止自动导航 |
| `bit 5` | 开始自动导航 |
| `bit 6` | 暂停自动导航 |

自动模式执行流程：

1. 将 `bit 1` 置为 1，进入自动模式。
2. 触发 `bit 5`，调用 `pilot->start()`。
3. `Pilot` 按 YAML 中 `paths` 顺序依次执行路径点。
4. 中间路径点只有在当前位置进入 `allow_final_pos_allow` 容差后，才会切换到下一个路径点，不再按规划时间超时自动切点。
5. 如果当前点配置了 `stand_at_target: true` 且 `stand_duration > 0`，到点后先进入位控站立等待。
6. 最后一个路径点完成后进入最终微调，然后停止。

---

## 6. 路径录制

### 6.1 录制流程

1. 进入手动模式。
2. 按住 `bit 2`，开始录制。
3. 移动到目标点。
4. 触发 `bit 14`，记录当前点。
5. 重复移动和记录。
6. 松开 `bit 2`，结束录制并关闭 YAML 文件。

### 6.2 记录点内容

每个记录点会保存：

- 当前 `map -> base_link` 的 `x/y`
- 当前机器人 yaw，写入 `target_yaw`
- 当前选择的 `policy_id`
- 速度 / 加速度 / 角速度限制
- PID 反馈参数
- 最终位置 / 朝向误差阈值
- 是否允许横移
- 轨迹衔接半径
- 到点后是否位控站立和站立时间

默认记录参数在 `src/obstacle_game/src/core/robot.cpp` 的记录点逻辑中设置。

---

## 7. YAML 路径格式

### 7.1 新版完整格式

```yaml
paths:
  - policy_id: 2
    target_pos:
      x: 1.0
      y: -1.0
    target_yaw: 0.0
    constraint_target_yaw: false
    target_vel: 0.0
    max_velocity: 0.7
    max_accelation: 0.4
    max_omega: 1.0
    kp:
      x: 0.2
      y: 0.2
      yaw: 0.4
    allow_start_dir_error: 0.2
    allow_final_dir_error: 0.2
    allow_final_pos_allow: 0.2
    adjust_min_vel: 0.2
    adjust_min_omega: 0.3
    allow_y_vel: false
    trajectory_connection_radius: 0.0
    stand_at_target: false
    stand_duration: 0.0
```

### 7.2 旧版兼容格式

旧字段仍可读取：

```yaml
paths:
  - policy_id: 2
    target_pos:
      x: 1.0
      y: -1.0
    target_vel: 0.0
    max_velocity: 0.7
    max_accelation: 0.4
    kp:
      x: 0.2
      y: 0.2
      yaw: 0.4
    allow_start_dir_error: 0.2
    err_allow: 0.2
    adjust_min_vel: 0.2
    min_omega: 0.3
```

旧字段映射关系：

| 旧字段 | 新 Pilot 内部含义 |
|---|---|
| `err_allow` | `allow_final_pos_allow` |
| `min_omega` | `adjust_min_omega` |
| `kp.yaw` | `kp.z()` |

缺失的新字段会使用默认值。

---

## 8. YAML 字段说明

### 8.1 基础字段

| 字段 | 说明 |
|---|---|
| `policy_id` | 当前路径段使用的运动策略 |
| `target_pos.x` | 目标点 x 坐标，地图系 |
| `target_pos.y` | 目标点 y 坐标，地图系 |
| `target_yaw` | 目标最终朝向，单位 rad |
| `constraint_target_yaw` | 是否约束最终朝向 |

### 8.2 速度限制

| 字段 | 说明 |
|---|---|
| `target_vel` | 到达该点后希望保留的速度 |
| `max_velocity` | 最大平移速度 |
| `max_accelation` | 最大加速度 |
| `max_omega` | 最大角速度 |

### 8.3 控制参数

| 字段 | 说明 |
|---|---|
| `kp.x` | x 方向位置反馈系数 |
| `kp.y` | y 方向位置反馈系数 |
| `kp.yaw` | yaw 方向反馈系数 |
| `adjust_min_vel` | 微调时最小平移速度 |
| `adjust_min_omega` | 微调时最小角速度 |

### 8.4 误差阈值

| 字段 | 说明 |
|---|---|
| `allow_start_dir_error` | 起步前允许的朝向误差 |
| `allow_final_dir_error` | 最终允许的朝向误差 |
| `allow_final_pos_allow` | 最终允许的位置误差 |

### 8.5 轨迹自由度

| 字段 | 说明 |
|---|---|
| `allow_y_vel` | 是否允许边横移边旋转 |
| `trajectory_connection_radius` | 多段路径衔接半径，>0 时启用三次曲线衔接 |
| `stand_at_target` | 到达该点后是否进入位控站立等待 |
| `stand_duration` | 位控站立等待时间，单位秒；为 0 时不等待 |

当 `trajectory_connection_radius > 0` 时，该点会走三次曲线衔接逻辑，即使 `stand_at_target: true` 也不会在该点末端位控站立。

---

## 9. `policy_id` 与 `Cmd.mode` 映射

| YAML `policy_id` | 含义 | 实际 `cmd.mode` |
|---:|---|---:|
| 1 | 位控站立 | 1 |
| 2 | 普通行走 | 2 |
| 3 | 台阶策略 | 3 |
| 4 | 沙地策略 | 4 |
| 5 | 斜坡策略 | 5 |
| 6 | 限高杆策略 | 6 |
| 7 | 木桥策略 | 7 |
| 8 | 翻墙策略 | 8 |

---

## 10. 调参建议

### 10.1 路径点之间停顿明显

可以给中间点设置：

```yaml
trajectory_connection_radius: 0.3
```

或：

```yaml
trajectory_connection_radius: 0.5
```

数值越大，越早进入下一段轨迹衔接。

### 10.2 需要边走边转

```yaml
allow_y_vel: true
```

适合空间足够、允许横移的路径段。

### 10.3 需要最终对准方向

```yaml
target_yaw: 1.57
constraint_target_yaw: true
allow_final_dir_error: 0.15
```

### 10.4 到点附近卡住

可适当增大：

```yaml
adjust_min_vel: 0.25
adjust_min_omega: 0.35
```

### 10.5 需要到点后位控站立

```yaml
stand_at_target: true
stand_duration: 1.0
trajectory_connection_radius: 0.0
```

只有实际到达该点并进入 `allow_final_pos_allow` 容差后才会站立等待；如果 `trajectory_connection_radius > 0`，该点不会站立等待。

### 10.6 路径跟踪晃动

可适当降低反馈增益：

```yaml
kp:
  x: 0.15
  y: 0.15
  yaw: 0.3
```

---

## 11. 安全行为

当控制周期内 TF 获取失败时：

- 自动驾驶停止。
- `cmd.mode` 切到 `1`。
- `cmd.vx / cmd.vy / cmd.vz` 清零。
- 控制模式回到手动安全态。

因此运行前必须确认 TF 稳定，否则自动导航不会继续执行。

---

## 12. 相关源码位置

| 功能 | 文件 |
|---|---|
| 主控制与遥控逻辑 | `src/obstacle_game/src/core/robot.cpp` |
| 自动导航算法 | `src/obstacle_game/src/core/pilot.cpp` |
| Pilot 接口与内部状态 | `src/obstacle_game/include/core/pilot.hpp` |
| 路径录制接口 | `src/obstacle_game/include/core/record.hpp` |
| YAML 写出逻辑 | `src/obstacle_game/src/core/record.cpp` |
| 遥控器消息 | `src/robot_msgs/msg/Remote.msg` |
| 控制命令消息 | `src/robot_msgs/msg/Cmd.msg` |
