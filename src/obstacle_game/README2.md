> 裁剪说明：keyboard、task_game、obstacle_game_ui 已移除。下文涉及这些模块及旧版 Record 的说明属于历史内容；当前 *_record.launch.py 仅启动手动控制和遥控节点，不提供路径录制。

遥控器与键盘操作摘要
=====================

概述
----
本项目提供基于遥控器和键盘的远程/手动控制方案，相关实现分布在以下模块：
- `src/remote_node`：串口遥控器驱动节点（发布 `robot_msgs/msg/Remote`）。
- `src/keyboard`：键盘输入映射节点（用于调试或无遥控器场景）。
- `src/obstacle_game` / `src/task_game`：机器人控制层，订阅遥控器/键盘并发布执行命令。
- `src/robot_msgs`：定义了 `Remote`、`Cmd` 等消息类型。

遥控器（Remote）
-----------------
- 节点：`remote_node`（位于 `src/remote_node`）。
- 发布：`robot_msgs/msg/Remote` 到话题 `/remote`。
- 行为：从串口读取遥控器协议并发布按键/摇杆状态。协议负载为 `float[4]` 摇杆值加 `uint32_t key`，对应 `lx`、`ly`、`rx`、`ry`、`key`。
- 模式：遥控器按键用于在 手动 / 自动 / 录制 模式间切换。
  - 手动模式：遥控器摇杆直接填充 `Cmd`（线速度/角速度等），由 `Robot` 节点读取并转发到 `/robot_move_cmd`。
  - 自动模式：由 `Pilot` 依据 `map -> base_link` TF 与路径 YAML 生成控制命令，`Robot` 发布为 `Cmd`。
- 录制模式：记录当前位置到 YAML（用于路径重放），并可给下一录制点附加站立或航向约束选项。
- 断连处理：串口断开或 1 秒无数据时，`remote_node` 发布一次清零的 `Remote` 消息并持续尝试重连；重连后的第一帧会设置 `just_reconnected=true`。
- 数据流：`/remote` -> `obstacle_game::Robot` / `task_game::Robot` -> 发布 `robot_msgs/msg/Cmd` 到 `/robot_move_cmd`。

遥控器实机操作速查
-----------------
- 模式拨杆：
  - `bit 1 = 0`：手动模式。此时摇杆直接控制底盘速度，策略按键切换 `Cmd.mode`。
  - `bit 1 = 1`：自动模式。此时 `Pilot` 接管运动控制，遥控器只负责开始、暂停、复位。刚从自动切回手动时，需要连续 3 帧检测到 `bit 1 = 0` 才生效，切回后进入位控站立 `mode=1` 并停止 `Pilot`。
- 录制拨杆：
  - `bit 2 = 1`：开始录制路径，首次进入时创建 `yaml_file_path + 时间戳 + .yaml`。
  - `bit 2 = 0`：结束录制并写出 YAML。
- 录制修饰键：
  - `bit 9 + bit 13`：录制状态下触发，下一次记录点写入 `stand_at_target=true`、`stand_duration=2`。
  - `bit 9 + bit 11`：录制状态下触发，下一次记录点写入 `constraint_target_yaw=true`、`allow_y_vel=true`。
  - 上面两个组合键只影响下一次 `bit 14` 记录的点位，写入后自动清除。
- 摇杆控制：
  - 左摇杆前后（`ly`）：控制前进/后退，对应 `cmd.vx`。
  - 左摇杆左右（`lx`）：控制横移，对应 `cmd.vy`。
  - 右摇杆左右（`rx`）：控制转向，对应 `cmd.vz`。
  - `ry`：当前 `obstacle_game` 和 `task_game` 未使用。
- 手动模式下的常用按键：
  - `bit 4`：位控站立，`mode=1`。
  - `bit 5`：普通行走，`mode=2`。
  - `bit 6`：台阶策略，`mode=3`。
  - `bit 3`：沙地策略，`mode=4`。
  - `bit 11`：斜坡策略，`mode=5`。
  - `bit 12`：限高杆策略，`mode=6`。
  - `bit 13`：木桥策略，`mode=7`。
  - `bit 10`：翻墙策略，`mode=8`。
  - 手动模式下若同时处于录制状态并按住 `bit 9`，`bit 11` 与 `bit 13` 不再切换策略，而是作为下一录制点的修饰键。
- 自动模式下的常用按键：
  - `bit 4`：复位并停止自动导航。
  - `bit 5`：开始自动导航。
  - `bit 6`：暂停自动导航。
- 录制操作：
  - 先把 `bit 2` 置 1 进入录制状态。
  - 需要特殊点位时，先触发 `bit 9 + bit 13` 或 `bit 9 + bit 11`。
  - 录制过程中触发 `bit 14`，记录当前点位。
  - 记录点会带上当前策略编号：位控站立 `1`、普通 `2`、台阶 `3`、沙地 `4`、斜坡 `5`、限高杆 `6`、木桥 `7`、翻墙 `8`。
  - 若当前没有有效 `map -> base_link` 位姿，`bit 14` 会跳过本次录点。

> 说明：源码里遥控器按键最终都映射成 `Remote.key` 的 bit 位。若您的实体遥控器面板标识与这里不同，请以实际发出的 bit 位为准。

键盘（Keyboard）
-----------------
- 节点：`keyboard`（位于 `src/keyboard`）。
- 功能：将键盘按键映射为控制命令或模式切换，常用于开发调试或无遥控器环境下的人工操控。
- 输出：实现可以直接发布 `robot_msgs/msg/Cmd`，或模拟 `Remote` 的按键事件（具体实现请参阅 `src/keyboard` 源码以确认行为与按键映射）。
- 常见映射（实现内可配置）：前进/后退/左转/右转、速度档位上下、启停、切换手动/自动/录制。

消息说明（简要）
-----------------
- `robot_msgs/msg/Remote`：遥控器原始数据结构，包含按键位、摇杆数值、模式标志等。
- `robot_msgs/msg/Cmd`：运动控制命令，通常包含线速度、角速度、爪/臂等动作控制字段，以及控制模式信息。

调试与常用操作
-----------------
- 先加载环境：
```bash
source install/setup.bash
```
- 启动遥控器节点（示例）：
```bash
ros2 run remote_node remote_node
```
- 启动机器人控制节点（示例）：
```bash
ros2 run obstacle_game robot_control
```
- 查看遥控器/命令话题：
```bash
ros2 topic echo /remote
ros2 topic echo /robot_move_cmd
```

实现注意事项
-----------------
- 模式与优先级：在手动模式下应优先响应摇杆/键盘输入，自动模式由 Pilot 决定输出；确保各来源发布频率与 TF 更新频率兼容。
- 串口参数：`remote_node` 默认读取 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，可通过 `--ros-args -p remote_dev_port:=/dev/ttyUSB0` 指定其他设备。
- 修改映射：若需修改按键或摇杆到 `Cmd` 的映射，请编辑 `src/keyboard` 或 `src/remote_node` 中对应逻辑并通过 `ros2 topic echo` 验证输出。

参考位置
-----------------
- `src/remote_node`
- `src/keyboard`
- `src/obstacle_game`
- `src/task_game`
- `src/robot_msgs`

键位映射（源码摘录）
--------------------
下面键位与按键位（bit 索引）来自 `src/keyboard/src/keyboard_node.cpp`，机器人侧对键位的使用见 `src/obstacle_game/src/core/robot.cpp` 和 `src/task_game/src/core/robot.cpp`。

- 运动轴（发布为 `robot_msgs/msg/Remote` 的摇杆值）：
  - `w`：前进 — 设置 `ly = +1200`（在 `task_game` 中映射为 `cmd.vx = ly/1200`）。
  - `s`：后退 — 设置 `ly = -1200`。
  - `a`：左移 — 设置 `lx = -1200`（在 `task_game` 中映射为 `cmd.vy = -lx/1200`）。
  - `d`：右移 — 设置 `lx = +1200`。
  - `q`：左转（自转负） — 设置 `rx = -1200`（在 `task_game` 中映射为 `cmd.vz = -rx/1200`）。
  - `e`：右转（自转正） — 设置 `rx = +1200`。
  - `Space`：清零所有轴（停止运动）。

- 模式切换（通过数字键修改 `Remote.key` 的模式位）：
  - `1`：切换到 Record 模式（`Remote.key` 的 bit 2 表示 Record）。
  - `2`：切换到 Manual 模式（本地实现把 Manual 视为默认、bit 清零）。
  - `3`：切换到 Auto 模式（`Remote.key` 的 bit 1 表示 Auto）。

- 策略/动作脉冲（按下后在短时间内将对应 bit 置位，用于触发一次性动作）：
  - `z`：位控站立（pulse bit 4）。
  - `x`：普通行走模式（pulse bit 5）。
  - `c`：台阶模式（pulse bit 6）。
  - `v`：沙地模式（pulse bit 3）。
  - `g`：翻墙模式（pulse bit 10）。
  - `b`：切换到 `robot_lab_bar` / 限高杆模式（pulse bit 12）。
  - `n`：切换到 `robot_lab_slope` / 斜坡模式（pulse bit 11）。
  - `h`：切换到 `robot_lab_bridge` / 木桥模式（pulse bit 13）。
  - `m`：记录当前点（pulse bit 14，用于路径记录）。

注释与行为说明：
- 键盘节点当前没有映射 `bit 9`，因此不能直接模拟实体遥控器上的录制修饰组合键 `bit 9 + bit 13` 和 `bit 9 + bit 11`。
- 在 `obstacle_game` 和 `task_game` 中，节点通过 `check_key_trigger(msg.key, index)` 判断某个动作按键是否被触发（该函数检测当前帧该 bit 从 0 -> 1）。
- `check_key_pressed(msg.key, index)` 用于检测某个模式位是否保持按下（例如自动模式 bit）。
- 遥控器/键盘发布的话题为 `/remote`，机器人控制命令发布在 `/robot_move_cmd`，可以通过 `ros2 topic echo` 验证实时数据。

遥控器 Bit 映射总表
--------------------

`robot_msgs/msg/Remote` 的 `key` 字段是一个 32 位无符号整数，每一位（bit）代表一个独立的按键/模式状态。
通过 `check_key_pressed(key, index)`（电平检测：该位当前为 1）和 `check_key_trigger(key, index)`（上升沿检测：该位从 0 变为 1）读取。

### 模式位（持续保持）

| Bit 索引 | 键盘按键 | 模式名称 | 说明 |
|---------|---------|---------|------|
| 1 | `3` | **Auto（自动模式）** | 该位为 1 时进入自动导航模式；为 0 时为手动模式 |
| 2 | `1` | **Record（路径录制模式）** | 该位为 1 时开始路径录制，松开后停止录制 |
| 9 | - | **Record option modifier（录制选项修饰）** | 仅 `obstacle_game` 使用；录制状态下和 bit 11 / bit 13 组合，给下一录制点增加 YAML 选项 |

> **注意**：Bit 1 和 Bit 2 独立设置，可以组合使用（例如自动模式下同时开启录制）。手动模式为 bit 1 清零的默认状态。

### 脉冲位（上升沿触发，一次性动作）

| Bit 索引 | 键盘按键 | 功能名称 | obstacle_game 行为 | task_game 行为 |
|---------|---------|---------|------------------|---------------|
| 3 | `v` | **sand（沙地策略）** | 切换到沙地策略 (mode=4) | 不支持 |
| 4 | `z` | **stand（位控站立/复位）** | 手动模式：切换到位控站立 (mode=1)；自动模式：复位并停止导航 | 手动模式：切换到位控站立 (mode=1)；自动调试模式：推进一次行为树阶段 |
| 5 | `x` | **walk（普通行走/开始导航）** | 手动模式：切换到普通行走 (mode=2)；自动模式：开始自动导航，启动前要求已有有效 `map -> base_link` 位姿 | 手动模式：切换到普通行走 (mode=2) |
| 6 | `c` | **stairs（台阶策略/暂停导航）** | 手动模式：切换到台阶策略 (mode=3)；自动模式：暂停自动导航 | 不支持 |
| 10 | `g` | **cross_wall（翻墙模式）** | 切换到翻墙策略 (mode=8) | 不支持 |
| 11 | `n` | **robot_lab_slope（斜坡模式）/yaw lock 录制选项** | 普通手动：切换到斜坡策略 (mode=5)；录制状态且 bit 9 按下：下一录制点写入 `constraint_target_yaw=true`、`allow_y_vel=true` | 不支持 |
| 12 | `b` | **robot_lab_bar（限高杆模式）** | 切换到限高杆策略 (mode=6) | 切换到限高杆策略 (mode=6) |
| 13 | `h` | **robot_lab_bridge（木桥模式）/stand 录制选项** | 普通手动：切换到木桥策略 (mode=7)；录制状态且 bit 9 按下：下一录制点写入 `stand_at_target=true`、`stand_duration=2` | 不支持 |
| 14 | `m` | **record_point（记录路径点）** | 记录一个当前路径点到 YAML | 不支持 |

### 摇杆轴映射（填充 Remote 的模拟量字段）

| Remote 字段 | 键盘按键 | 物理方向 | 映射到 Cmd 字段 | 取值范围 | 缩放公式 |
|------------|---------|---------|---------------|---------|---------|
| `lx` | a / d | 左右平移 | `cmd.vy` | ±1200 | `vy = -lx / 1200` |
| `ly` | w / s | 前后移动 | `cmd.vx` | ±1200 | `vx = +ly / 1200` |
| `rx` | q / e | 自转 | `cmd.vz` | ±1200 | `vz = -rx / 1200` |
| `ry` | - | （未使用） | - | - | - |

速度限幅：
- `obstacle_game`：`vx` 限幅 `[-1.2, 1.2]`，`vy` 限幅 `[-0.8, 0.8]`，`vz` 限幅 `[-1.0, 1.0]`。
- `task_game`：`vx`、`vy` 限幅 `[-1.2, 1.2]`，`vz` 限幅 `[-1.0, 1.0]`。

### 策略模式与 Cmd.mode 值

| Cmd.mode | 名称 | 触发 Bit | 说明 |
|---------|------|---------|------|
| 1 | 位控站立 | bit 4 | 保持站立姿态，不移动 |
| 2 | 普通行走 | bit 5 | 标准步态行走策略 |
| 3 | 台阶策略 | bit 6 | 适用于跨越台阶（仅 obstacle_game） |
| 4 | 沙地策略 | bit 3 | 适用于松软地面（仅 obstacle_game） |
| 5 | 斜坡策略 | bit 11 | 适用于斜坡地形 |
| 6 | 限高杆策略 | bit 12 | 低矮障碍物通过模式 |
| 7 | 木桥策略 | bit 13 | 适用于木桥地形 |
| 8 | 翻墙策略 | bit 10 | 适用于翻墙动作（仅 obstacle_game） |

### YAML 中的 policy_id 编码

| policy_id | 对应策略 | 实际运行时 Cmd.mode |
|----------|---------|-------------------|
| 1 | 位控站立 | 1 |
| 2 | 普通行走 | 2 |
| 3 | 台阶 | 3 |
| 4 | 沙地 | 4 |
| 5 | 斜坡 | 5 |
| 6 | 限高杆 | 6 |
| 7 | 木桥 | 7 |
| 8 | 翻墙 | 8 |

### 两个检测函数说明

```cpp
// 检测上升沿：当前帧该 bit=1 且上一帧该 bit=0（用于一次性动作触发）
bool check_key_trigger(uint32_t current_key, int index) {
    bool current_is_true = ((current_key >> index) & 0x0001);
    bool last_is_false   = !((last_key >> index) & 0x0001);
    return current_is_true && last_is_false;
}

// 检测电平：当前帧该 bit=1（用于模式保持判断）
bool check_key_pressed(uint32_t current_key, int index) {
    return ((current_key >> index) & 0x0001);
}
```

### obstacle_game 与 task_game 的 bit 使用差异

| Bit | obstacle_game | task_game |
|-----|--------------|-----------|
| 3   | 沙地策略 | 不支持 |
| 4   | 手动位控站立；自动复位并停止导航 | 手动位控站立；自动调试模式推进行为树阶段 |
| 5   | 手动普通行走；自动开始导航 | 手动普通行走 |
| 6   | 台阶策略 / 暂停导航 | 不支持 |
| 9   | 录制选项修饰键，需要和 bit 11 / bit 13 组合 | 不支持 |
| 10  | 翻墙策略 | 不支持 |
| 11  | 斜坡策略；录制状态下配合 bit 9 设置下一点航向锁定 | 不支持 |
| 12  | robot_lab_bar（限高杆策略） | 支持 |
| 13  | 木桥策略；录制状态下配合 bit 9 设置下一点站立等待 | 不支持 |
| 14  | 记录路径点 | 不支持 |

`task_game` 专注于搬运箱子的行为树流程，因此不需要沙地、台阶等地面策略和路径记录功能。

# obstacle_game - 机器人障碍赛导航系统

## 1 项目概述

`obstacle_game` 是一个基于 ROS2 的四足机器人自动导航障碍赛系统。该系统支持手动遥控和自主导航两种模式，能够沿着预设的路径点自动行进，适用于障碍赛、巡检等应用场景。

### 1.1 主要特性

- **双模式控制**：支持手动遥控模式和自动导航模式的无缝切换
- **路径点记录**：支持通过遥控器实时记录路径点，并导出为 YAML 配置文件
- **多策略支持**：内置多种运动策略（站立、行走、沙地、台阶等）
- **前馈+反馈控制**：结合前馈速度和反馈误差的复合控制算法
- **TF坐标转换**：基于 ROS2 TF2 的实时位姿估计

---

## 2 系统架构

### 2.1 架构图

```
┌─────────────────────────────────────────────────────────────┐
│                     Robot (主控制器)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ 遥控器处理   │  │ 模式切换    │  │ TF坐标监听           │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└──────────────────────────┬──────────────────────────────────┘
                           │
           ┌───────────────┴───────────────┐
           │                               │
           ▼                               ▼
┌─────────────────────┐       ┌─────────────────────────┐
│   Pilot (导航引擎)   │       │   Record (路径记录)      │
│  ┌───────────────┐  │       │  ┌───────────────────┐  │
│  │ 路径加载       │  │       │  │ YAML 文件写入      │  │
│  │ 轨迹跟踪控制   │  │       │  │ 路径点记录         │  │
│  │ 速度规划       │  │       │  └───────────────────┘  │
│  └───────────────┘  │       └─────────────────────────┘
└─────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│                   robot_move_cmd (发布)                      │
│                   robot_msgs/msg/Cmd                         │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 模块职责

| 模块 | 职责 | 核心文件 |
|------|------|---------|
| **Robot** | 主控制器，负责遥控器输入、模式切换、TF监听、指令发布 | `robot.cpp` |
| **Pilot** | 自动导航引擎，路径加载、速度规划、前馈+反馈控制 | `pilot.cpp` |
| **Record** | 路径记录器，YAML配置文件写入、路径点采集 | `record.cpp` |

---

## 3 模块详细说明

### 3.1 Robot (主控制器)

`Robot` 类是系统的核心入口，负责整合所有子模块并协调工作。

#### 控制流程

```
遥控器输入 → 模式判断 → 模式处理 → 指令发布
              ↓
         1. 手动模式: 直接解析摇杆值
         2. 自动模式: 调用 Pilot 计算指令
```

#### 模式定义

| 模式值 | 名称 | 说明 |
|--------|------|------|
| 1 | 位控站立 | 机器人保持站立姿态，不移动 |
| 2 | 普通行走 | 标准步态行走策略 |
| 3 | 台阶策略 | 适用于跨越台阶的策略 |
| 4 | 沙地策略 | 适用于松软地面的行走策略 |
| 5 | 斜坡策略 | 适用于斜坡地形 |
| 6 | 限高杆策略 | 适用于低矮障碍通过 |
| 7 | 木桥策略 | 适用于木桥地形 |
| 8 | 翻墙策略 | 适用于翻墙动作 |

#### 遥控器按键映射

| Bit 索引 | 检测方式 | obstacle_game 功能 | 说明 |
|----------|---------|-------------------|------|
| 1 | `check_key_pressed` | 自动/手动切换 | 该位为 1 时进入自动模式，为 0 时手动模式 |
| 2 | `check_key_pressed` | 路径录制开关 | 按下后开始录制路径 YAML，松开后停止 |
| 3 | `check_key_trigger` | 沙地策略 (mode=4) | 手动模式下切换到沙地策略 |
| 4 | `check_key_trigger` | 手动: 位控站立 (mode=1)；自动: 复位并停止导航 | 上升沿触发 |
| 5 | `check_key_trigger` | 手动: 普通行走 (mode=2)；自动: 开始自动导航 | 上升沿触发 |
| 6 | `check_key_trigger` | 手动: 台阶策略 (mode=3)；自动: 暂停自动导航 | 上升沿触发 |
| 9 | `check_key_pressed` | 录制选项修饰键 | 录制模式下配合 bit 11 / bit 13 使用 |
| 10 | `check_key_trigger` | 翻墙策略 (mode=8) | 上升沿触发 |
| 11 | `check_key_trigger` | 斜坡策略 (mode=5) / 下一录制点锁航向 | 普通手动为斜坡；录制模式且 bit 9 按下时写入 `constraint_target_yaw=true`、`allow_y_vel=true` |
| 12 | `check_key_trigger` | 限高杆策略 (mode=6) | 上升沿触发 |
| 13 | `check_key_trigger` | 木桥策略 (mode=7) / 下一录制点站立等待 | 普通手动为木桥；录制模式且 bit 9 按下时写入 `stand_at_target=true`、`stand_duration=2` |
| 14 | `check_key_trigger` | 记录当前路径点 | 上升沿触发，需先开启录制模式 (bit 2) |

> 完整 Bit 映射总表见上方「遥控器 Bit 映射总表」章节。

### 3.2 Pilot (导航引擎)

`Pilot` 类实现了自主导航的核心控制算法。

#### 控制算法

采用**前馈+反馈**的复合控制策略：

```
desired_velocity = feedforward_velocity + feedback_velocity

其中:
  feedforward_velocity: 基于路径曲率和目标速度的前馈控制
  feedback_velocity:    基于位置误差的 P 控制
```

#### 速度规划

- **加速度限制**：限制速度变化率，防止急加减速
- **停止距离规划**：根据目标速度和加速度计算安全停止距离
- **最小调整速度**：在接近目标点时保持最小速度确保平滑过渡

#### 航向控制

- 允许航向误差阈值 (`allow_start_dir_error`)
- 当航向偏差过大时，机器人原地旋转对齐后再行进

### 3.3 Record (路径记录)

`Record` 类负责采集和保存导航路径点。

#### 工作流程

```
开始记录 → 设置输出文件 → 逐点记录 → 结束记录
              ↓
         生成 YAML 配置文件
```

#### 输出格式

```yaml
paths:
  - policy_id: 2
    target_pos:
      x: 1.0
      y: 0.5
    target_vel: 0.0
    max_velocity: 0.6
    max_accelation: 0.4
    kp:
      x: 0.1
      y: 0.1
      yaw: 0.1
    allow_start_dir_error: 0.2
    err_allow: 0.2
    adjust_min_vel: 0.2
```

---

## 4 依赖说明

### 4.1 系统依赖

- **ROS2** (Humble 或更高版本)
- **Eigen3** - 线性代数库
- **yaml-cpp** - YAML 配置文件解析库

### 4.2 ROS2 依赖包

| 包名 | 说明 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `geometry_msgs` | 几何消息类型 |
| `sensor_msgs` | 传感器消息类型 |
| `visualization_msgs` | 可视化消息类型 |
| `tf2` / `tf2_ros` | 坐标变换库 |
| `robot_msgs` | 自定义机器人消息 |

---

## 5 编译与运行

### 5.1 编译

```bash
# 进入工作空间
cd ~/AT_robot-lab

# 编译 obstacle_game 包
colcon build --packages-select obstacle_game

# 或编译整个工作空间
colcon build
```

### 5.2 运行

```bash
# 加载环境
source install/setup.bash

# 启动主控制节点
ros2 run obstacle_game robot_control
```

### 5.3 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `scene_path` | string | `/home/dog/Desktop/AT_robot-lab/record20260515_211359.yaml` | 导航路径配置文件 |
| `yaml_file_path` | string | `./trajectory/record` | 记录输出路径前缀 |

---

## 6 配置说明

### 6.1 路径配置文件格式

详细格式见上方 `Record` 模块说明。

### 6.2 参数调优指南

| 参数 | 建议范围 | 调优说明 |
|------|----------|----------|
| `max_velocity` | 0.3 ~ 1.0 | 速度上限，越大越快但越难控制 |
| `max_accelation` | 0.2 ~ 0.8 | 加速度上限，影响启停平滑度 |
| `kp.x` / `kp.y` | 0.05 ~ 0.3 | 位置误差增益，越大响应越快但可能振荡 |
| `kp.yaw` | 0.1 ~ 0.5 | 航向误差增益 |
| `err_allow` | 0.1 ~ 0.3 | 到达判定阈值，到达此距离认为目标已到达 |
| `allow_start_dir_error` | 0.1 ~ 0.5 | 允许的航向误差（弧度） |
| `adjust_min_vel` | 0.1 ~ 0.3 | 接近目标时的最小调整速度 |

---

## 7 调试与故障排除

### 7.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| TF获取失败 | 定位系统未启动 | 检查定位节点是否运行 |
| 导航无法启动 | 路径文件为空或格式错误 | 检查 YAML 文件内容 |
| 机器人原地抖动 | `kp` 值过大 | 减小 kp.x / kp.y |
| 机器人到达目标后不停 | `err_allow` 过小 | 适当增大 err_allow |

### 7.2 日志级别

- 默认日志级别为 INFO
- 使用 RCLCPP 提供的日志函数输出状态信息
- 关键日志包括：模式切换、路径点到达、TF转换状态

---

## 8 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0.0 | 2025-05-15 | 初始版本，支持基本导航功能 |

---

## 9 许可证

本项目遵循 TODO: License Declaration
