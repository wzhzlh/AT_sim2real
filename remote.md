# 遥控器数据传递链路（`remote_test.launch.py` → `rl_real_atdog2` → 下位机）

本文档说明在终端分别执行：

```bash
ros2 run rl_sar rl_real_atdog2
ros2 launch obstacle_game remote_test.launch.py
```

之后，物理遥控器的数据如何经过 ROS 2 话题层层传递、被哪些节点处理、转换成什么控制量，以及最终如何到达 STM32 下位机和电机。

## 1. 总览

```text
物理遥控器
  │  USB 串口，私有协议，115200
  ▼
/remote_node
  │  /remote            robot_msgs/msg/Remote
  ▼
/robot_calc_node（obstacle_game）
  │  /robot_move_cmd    robot_msgs/msg/Cmd
  ▼
/robot_controller_node（rl_real_atdog2）
  │  FSM / 策略推理 / 关节目标生成
  ▼
LegDriver::set_leg_target()
  │  USB CDC，libusb bulk OUT，pack_type=4
  ▼
STM32 下位机 → 485 总线电机 → 机器人运动
```

关键区分：

- **遥控器原始数据**：`Remote.msg` 中的 `lx/ly/rx/ry/key`，来自遥控器串口包，只做解包，不含控制语义。
- **上位机控制指令**：`Cmd.msg` 中的 `mode/vx/vy/vz/wheel_vel`，由 `obstacle_game` 解析遥控器后生成。
- **电机目标**：`LegTarget_t`/`DogTargetPack_t` 中的关节 `rad/omega/torque/kp/kd` 和轮子 `omega/kd`，由策略/FSM 生成，经 USB CDC 发给下位机。
- 控制指令的**终点不是 ROS 话题**，而是 USB CDC 写往 STM32；状态反馈再由 STM32 通过 USB CDC 回到 `LegDriver`。

## 2. 启动后存在的节点和话题

### 2.1 节点

| 启动命令 | 可执行文件 | ROS 2 节点名 | 主要职责 |
| --- | --- | --- | --- |
| `ros2 launch obstacle_game remote_test.launch.py` | `remote_node` | `/remote_node` | 读取遥控器串口，发布 `/remote` |
| 同上 | `robot_control` | `/robot_calc_node` | 订阅 `/remote`，生成并发布 `/robot_move_cmd` |
| `ros2 run rl_sar rl_real_atdog2` | `rl_real_atdog2` | `/robot_controller_node` | 订阅 `/robot_move_cmd`，运行 FSM/策略，下发电机目标 |

依据：

- `src/obstacle_game/launch/remote_test.launch.py:5-21` 只启动 `robot_control` 和 `remote_node`。
- `src/remote_node/src/remote_node.cpp:15-16` 节点名为 `remote_node`。
- `src/obstacle_game/src/main.cpp:6-11` 节点名为 `robot_calc_node`。
- `src/rl_sar/src/rl_real_atdog2.cpp:284-287` 节点名为 `robot_controller_node`。

该 launch 文件**没有启动 `joy` 节点，也没有 remapping 或 namespace**。这里使用的是遥控器私有串口协议，不是 `sensor_msgs/msg/Joy`。若看到 `/joy`，那属于仿真或其它链路，不属于本流程。

### 2.2 话题

| 话题 | 类型 | 方向 | 作用 | QoS |
| --- | --- | --- | --- | --- |
| `/remote` | `robot_msgs/msg/Remote` | `/remote_node` → `/robot_calc_node` | 传输原始摇杆值和按键 bit 掩码 | 深度 10，默认 RELIABLE / VOLATILE |
| `/robot_move_cmd` | `robot_msgs/msg/Cmd` | `/robot_calc_node` → `/robot_controller_node` | 传输模式与速度指令 | 深度 10，默认 RELIABLE / VOLATILE |
| `/policy_done` | `robot_msgs/msg/Int` | 外部节点 → `/robot_calc_node` | 自动轨迹模式下的策略完成反馈，不是遥控必经链路 | 深度 10，默认 QoS |

代码使用相对话题名 `"remote"`、`"robot_move_cmd"`，没有 namespace，因此最终解析为根命名空间下的 `/remote`、`/robot_move_cmd`。

消息定义：

- `src/robot_msgs/msg/Remote.msg`

```text
float32 lx
float32 ly
float32 rx
float32 ry
uint32 key
bool just_reconnected
```

- `src/robot_msgs/msg/Cmd.msg`

```text
int32 mode
float32 vx
float32 vy
float32 vz
float32 wheel_vel
```

## 3. 第一层：`remote_node` 把串口数据发布到 `/remote`

### 3.1 串口连接

`remote_node` 在构造时：

- 声明参数 `remote_dev_port`，默认值为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`（`src/remote_node/src/remote_node.cpp:18`）。
- 波特率固定为 `115200`，读超时 `10 ms`（`remote_node.cpp:53-75`）。
- 可用如下方式覆盖端口：

```bash
ros2 launch obstacle_game remote_test.launch.py --ros-args \
  -p remote_dev_port:=/dev/ttyUSB0
```

- 启动串口接收线程 `serial_recv_task()` 和看门狗线程 `watchdog_task()`（`remote_node.cpp:44-50`）。

如果串口打开失败，节点不会退出，而是持续尝试重连；超过 1 秒没有数据也会重新初始化串口（`remote_node.cpp:159-246`）。

### 3.2 私有串口协议解析

遥控器数据走 `RemoteComm` 私有协议：

- 包头 `0x5A`。
- 遥控器数据命令 `CMD_REMOTE_CONTROL = 0x01`。
- 一帧遥控器 payload 为 20 字节：`float[4] + uint32_t key`。
- 串口接收线程逐字节调用 `remote_comm_->process_recv_byte()`，完整包通过注册回调进入 `on_remote_control_data()`（`remote_node.cpp:31-38, 208-218`）。

相关实现：

- `src/remote_node/include/remote_node/remote_comm.hpp`
- `src/remote_node/src/remote_comm.cpp`
- `src/remote_node/include/remote_node/remote_node.hpp`

### 3.3 生成 `Remote.msg`

`on_remote_control_data()` 只做内存拷贝和发布，不进行速度映射或限幅（`remote_node.cpp:250-273`）：

```cpp
memcpy(&remote.lx, data + 0, sizeof(float));
memcpy(&remote.ly, data + 4, sizeof(float));
memcpy(&remote.rx, data + 8, sizeof(float));
memcpy(&remote.ry, data + 12, sizeof(float));
memcpy(&remote.key, data + 16, sizeof(uint32_t));
remote.just_reconnected = node->should_mark_reconnect_.exchange(false);
node->remote_pub->publish(remote);
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `lx` | 左摇杆水平原始值 |
| `ly` | 左摇杆垂直原始值 |
| `rx` | 右摇杆水平原始值 |
| `ry` | 右摇杆垂直原始值，当前控制链路未使用 |
| `key` | `uint32_t` 按键位掩码 |
| `just_reconnected` | 非原始字段，仅用于标记重连后的首帧 |

发布频率等于遥控器串口包到达频率，节点没有额外节流。

### 3.4 断连与看门狗

- 串口断开或超过 1 秒无数据时，发布一次清零帧：`lx/ly/rx/ry=0`、`key=0`、`just_reconnected=false`（`remote_node.cpp:111-134, 219-228`）。
- 重连成功后第一帧 `just_reconnected=true`（`remote_node.cpp:183-187, 270`）。
- 看门狗线程每 2 秒检查一次心跳：2 秒无变化 WARN，6 秒无变化 ERROR，恢复后 INFO（`remote_node.cpp:297-335`）。
- 坏包目前只打印日志，不控制机器人（`remote_node.cpp:277-295`）。

## 4. 第二层：`robot_calc_node` 把 `/remote` 转换成 `/robot_move_cmd`

核心文件：

- `src/obstacle_game/src/core/robot.cpp`
- `src/obstacle_game/include/core/robot.hpp`

### 4.1 订阅和发布

```cpp
cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);
remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>(
    "remote", 10, [this](const robot_msgs::msg::Remote& msg) { ... });
```

对应 `robot.cpp:100-104`。

`Robot` 内部维护一个 `Cmd cmd` 成员变量，遥控器回调只更新该成员；另有 50 ms 周期定时器统一发布 `cmd`（`robot.cpp:265, 334`），因此 `/robot_move_cmd` 约为 **20 Hz**。

### 4.2 `key` 位读取方式

项目把 `key` 当作 bit 掩码：

- `check_key_pressed(key, index)`：当前 bit 为 1，用于持续状态。
- `check_key_trigger(key, index)`：当前帧为 1 且上一帧为 0，用于一次性触发。

实现见 `src/obstacle_game/src/core/robot.cpp:429-434`。每次遥控器回调末尾调用 `save_key_state(msg.key)` 保存上一帧状态（`robot.cpp:188`）。

### 4.3 自动/手动模式切换

`bit 1` 是自动/手动拨杆：

- `bit 1 = 0`：手动模式。
- `bit 1 = 1`：自动模式。

从自动切回手动时，有 3 帧防抖（`kManualSwitchDebounceFrames = 3`）。连续 3 帧满足后执行（`robot.cpp:106-127`）：

- `cmd.mode = 1`，强制进入位控站立，避免自动轨迹中途切回时继续运动。
- `pilot->reset()`：复位自动轨迹。
- `pilot->stop()`：停止自动轨迹。
- `current_control_mode = 0`。

### 4.4 手动模式：按键决定 `mode`

手动模式下，使用 trigger 语义读取按键，映射到 `Cmd.mode`（`robot.cpp:129-152`）：

| 遥控器 bit | 动作 | `cmd.mode` | at dog2 FSM 目标状态 |
| --- | --- | --- | --- |
| `4` | 位控站立 | `1` | `RLFSMStateGetUp` |
| `5` | 普通行走 | `2` | `RLFSMStateRLLocomotion` |
| `6` | 台阶 | `3` | `RLFSMStateRLStairs` |
| `3` | 沙地 | `4` | `RLFSMStateRLSand` |
| `11` | 斜坡 | `5` | `RLFSMStateRLSlope` |
| `12` | 限高杆 | `6` | `RLFSMStateRLBar` |
| `13` | 木桥 | `7` | `RLFSMStateRLBridge` |

`bit 2` 的录制保持、`bit 9` 的录点复用、`bit 14` 的录点触发属于 `obstacle_game` 的 YAML 轨迹录制功能，不改变上述 `mode` 映射。

### 4.5 手动模式：摇杆决定速度

`robot.cpp:154-157` 当前实际公式：

```cpp
cmd.vy = -(float)std::clamp((double)msg.lx / 1200.0, -0.8, 0.8);
cmd.vx =  (float)std::clamp((double)msg.ly / 800.0,  -1.8, 1.8);
cmd.vz = -(float)std::clamp((double)msg.rx / 1000.0, -1.5, 1.5);
```

即：

| `Cmd` 字段 | 来源 | 换算 | 限幅 | 含义 |
| --- | --- | --- | --- | --- |
| `vx` | `ly` | `ly / 800` | `[-1.8, 1.8]` | 前后速度 |
| `vy` | `lx` | `-lx / 1200` | `[-0.8, 0.8]` | 左右速度 |
| `vz` | `rx` | `-rx / 1000` | `[-1.5, 1.5]` | 偏航角速度 |
| `wheel_vel` | 无 | 未赋值 | — | at dog2 链路未使用 |

`ry` 当前未参与控制。

### 4.6 自动模式：由 Pilot 覆盖 `Cmd`

自动模式下，遥控器只负责启停/复位：

- `bit 4`：复位并停止 pilot（`robot.cpp:161-166`）。
- `bit 5`：先 `refresh_robot_pose()`，成功后再 `pilot->start()`（`robot.cpp:167-175`）。
- `bit 6`：`pilot->stop()`，暂停自动轨迹（`robot.cpp:176-180`）。

自动运行时，50 ms 定时器会用 `pilot->get_command()` 的结果覆盖 `cmd`（`robot.cpp:305-326`），其中包含 `mode`、速度和轨迹对应的策略 ID。位姿来自 TF `map → base_link`（`robot.cpp:338-347`）；如果 TF 失效，则停止 pilot，并强制 `cmd.mode=1`、`vx/vy/vz=0`、退回手动模式（`robot.cpp:279-294`）。

`Cmd.wheel_vel` 在该链路中没有被 `rl_real_atdog2` 读取，因此不会传到电机。

## 5. 第三层：`rl_real_atdog2` 订阅 `/robot_move_cmd`

### 5.1 初始化

`rl_real_atdog2` 构造时（`src/rl_sar/src/rl_real_atdog2.cpp:15-64`）：

- 读取 `policy/atdog2/base.yaml`：`dt=0.005`、`decimation=4`、`num_of_dofs=12`、`joint_mapping=[0..11]`。
- 创建 at dog2 FSM。
- 创建 `IMUDriver` 和 `LegDriver`。
- 注册电机错误回调：收到错误后 `rclcpp::shutdown()`（`rl_real_atdog2.cpp:31-42`）。
- 订阅 `/robot_move_cmd`，创建三个循环：
  - `loop_command`：0.05 s，`KeyboardInterface`，本地键盘调试。
  - `loop_control`：`dt=0.005 s`，即 200 Hz，`RobotControl`。
  - `loop_rl`：`dt * decimation = 0.02 s`，即 50 Hz，`RunModel`。

### 5.2 订阅回调只缓存消息

```cpp
cmd_sub = node_->create_subscription<robot_msgs::msg::Cmd>(
    "robot_move_cmd", 10, [this](const robot_msgs::msg::Cmd& msg) {
        remote_cmd = msg;
        if (msg.mode != last_logged_remote_mode_) {
            last_logged_remote_mode_ = msg.mode;
            RCLCPP_INFO(node_->get_logger(), "robot_move_cmd mode=%d", msg.mode);
        }
    });
```

见 `rl_real_atdog2.cpp:44-51`。

也就是说，订阅回调不直接控制电机，只更新 `remote_cmd`；真正消费它的是 200 Hz 的 `RobotControl()`。因此两个终端的启动顺序不影响最终连接：控制器启动后若还没收到 `Cmd`，`remote_cmd.mode` 为默认 0，进入“非 ROS 接管”状态。

### 5.3 `RobotControl()` 的操作

`rl_real_atdog2.cpp:131-154`：

```cpp
this->GetState(&this->robot_state);

if (remote_cmd.mode != 0) {
    this->control.setMode(remote_cmd.mode);
    const float vx = std::clamp(remote_cmd.vx, -2.5f, 2.5f);
    const float vy = std::clamp(remote_cmd.vy, -1.0f, 1.0f);
    const float vz = std::clamp(remote_cmd.vz, -2.5f, 2.5f);
    this->control.setVel(vx, vy, vz);
} else {
    this->control.setMode(0);
}

this->StateController(&this->robot_state, &this->robot_command);
this->control.ClearInput();
this->SetCommand(&this->robot_command);
```

处理顺序：

1. `GetState()`：读取 IMU 和 12 个关节状态，写入 `robot_state`。
2. 若 `remote_cmd.mode != 0`，认为 ROS 上层接管：
   - 设置 `control.mode`。
   - 对 `vx/vy/vz` 做第二次限幅。
   - 写入 `control.x/y/yaw`。
3. 若 `mode == 0`，只把内部 `control.mode` 设为 0，不再用 ROS 速度覆盖内部键盘/手柄输入。
4. `StateController()`：驱动 FSM 和策略。
5. `ClearInput()`：只清除键盘/手柄按键状态，不清空 `x/y/yaw/mode`（`src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:155-159`），所以速度会持续生效直到新的 `Cmd` 到来。
6. `SetCommand()`：把 `robot_command` 转换成电机目标并发送。

两级限幅汇总：

| 量 | `obstacle_game` 限幅 | `rl_real_atdog2` 限幅 | 实际生效范围 |
| --- | --- | --- | --- |
| `vx` | ±1.8 | ±2.5 | ±1.8 |
| `vy` | ±0.8 | ±1.0 | ±0.8 |
| `vz` | ±1.5 | ±2.5 | ±1.5 |

## 6. 第四层：FSM 与策略如何使用这些数据

### 6.1 `mode` 到 FSM 状态

`src/rl_sar/fsm_robot/fsm_atdog2.hpp:16-51` 的 `ResolveRemoteModeState()` 负责把 `control.mode` 翻译成目标状态：

| `mode` | FSM 状态 | 说明 |
| --- | --- | --- |
| `0` | 不切换 | 保留内部键盘/被动逻辑 |
| `1` | `RLFSMStateGetUp` | 位控站立 |
| `2` | `RLFSMStateRLLocomotion` | 普通行走策略 |
| `3` | `RLFSMStateRLStairs` | 台阶策略 |
| `4` | `RLFSMStateRLSand` | 沙地策略 |
| `5` | `RLFSMStateRLSlope` | 斜坡策略 |
| `6` | `RLFSMStateRLBar` | 限高杆策略 |
| `7` | `RLFSMStateRLBridge` | 木桥策略 |
| `9` | `RLFSMStateCheck` | 仅当前状态为 `GetUp` 时有效 |

`StateController()` 会把当前 `robot_state` 和 `robot_command` 注入所有 FSM 状态，然后调用 `fsm.Run()`（`src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:9-24`）。各策略状态的 `CheckChange()` 最终都会调用 `ResolveRemoteModeState(rl.control.mode, state_name_)`，例如 `fsm_atdog2.hpp:317-337`。

### 6.2 速度进入策略

只有进入 RL 策略状态后，速度指令才真正进入策略：

- `RLFSMStateRLLocomotion::Enter()` 加载 `policy/atdog2/robot_lab` 配置并初始化模型（`fsm_atdog2.hpp:280-299`）。
- `Run()` 打印当前 `control.x/y/yaw`，然后调用 `RLControl()`（`fsm_atdog2.hpp:301-309`）。
- `RunModel()` 中：

```cpp
this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
this->obs.ang_vel  = this->robot_state.imu.gyroscope;
this->obs.base_quat = this->robot_state.imu.quaternion;
this->obs.dof_pos   = this->robot_state.motor_state.q;
this->obs.dof_vel   = this->robot_state.motor_state.dq;
this->obs.actions   = this->Forward();
this->ComputeOutput(this->obs.actions,
                    this->output_dof_pos,
                    this->output_dof_vel,
                    this->output_dof_tau);
```

见 `rl_real_atdog2.cpp:214-235`。

其中：

- `obs.commands` 在 `ComputeObservation()` 中乘以 `commands_scale`，再进入模型输入（`rl_sdk.cpp:117-120`）。
- `obs.commands` 同时驱动 gait clock，更新步态相位。
- `Forward()` 调用推理后端得到动作，并按 `clip_actions_lower/upper` 限幅。
- `ComputeOutput()` 将动作乘以 `action_scale`，生成：
  - `output_dof_pos = default_dof_pos + 缩放后的位置动作`
  - `output_dof_vel`：轮式自由度速度动作；at dog2 的 `wheel_indices` 为空，因此为空/不参与
  - `output_dof_tau`：经 `torque_limits` 限幅的力矩（`rl_sdk.cpp:497-511`）
- `RLFSMState::RLControl()` 从队列取出结果，写入 `robot_command.motor_command.q/dq/kp/kd/tau`（`rl_sdk.cpp:836-856`）。

位控站立、起立、下蹲等状态通过插值生成关节目标，不消费速度指令；因此遥控器摇杆在 `mode=1` 时不会改变站立姿态。

## 7. 第五层：`SetCommand()` → USB CDC → STM32

### 7.1 生成 `LegTarget_t[4]`

`SetCommand()`（`rl_real_atdog2.cpp:156-212`）：

1. 清零 4 条腿 × 3 个关节的 `rad/omega/torque/kp/kd`。
2. 清零 4 个轮子的 `omega/kd`。at dog2 是 12 自由度配置，轮子不参与控制。
3. 按 `joint_mapping` 把策略 DOF 顺序映射到硬件腿/关节编号。
4. 从 `robot_command.motor_command` 拷贝：
   - `q → rad`
   - `dq → omega`
   - `tau → torque`
   - `kp → kp`
   - `kd → kd`
5. 调用 `leg_driver->set_leg_target(legs_target, ms)`，`ms` 为 `steady_clock` 毫秒时间戳。

需要注意：`Cmd.wheel_vel` 没有在这一层被读取，也不会发送给下位机。

### 7.2 打包为 USB 协议

`src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/src/leg_driver/leg_driver.cpp:160-185`：

- 组装 `DogTargetPack_t`：
  - `pack_type = 4`
  - `time = 上位机毫秒时间戳`
  - 4 条腿的目标值
  - 1 字节对齐，总长度 280 字节
- 正常模式直接拷贝 `legs_target`。
- `enable_control=false` 时进入安全阻尼：关节 `rad/omega/torque/kp=0`、`kd=8`，轮子 `omega=0`、`kd=8`。
- 最后调用 `cdc_trans->send_struct(target_pack)`。

`send_struct()` 最终调用 `libusb_bulk_transfer(handle, EP_OUT=0x01, ...)`，见：

- `src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/include/leg_driver/cdc_trans.hpp:10-35`
- `src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/src/leg_driver/cdc_trans.cpp:118-131`

VID/PID 默认为 `0x0483 / 0x5740`（`leg_driver.hpp:21`）。

### 7.3 下位机执行与状态回传

STM32 下位机收到 `pack_type=4` 后：

- 按 280 字节结构解析 12 个关节目标及 4 个轮子速度/速度增益。
- 将关节目标换算到电机输出轴。
- 通过两条 485 总线控制电机闭环。

下位机也会以约 200 Hz 回传 `pack_type=3` 状态包，当前为 252 字节，包括：

- 4 条腿的关节/轮子 `rad/omega/torque`
- 独立 `wheel[4]`
- `watch_dog`
- `timestamp`
- `motor_error`

`LegDriver` 收到后更新腿部状态；`watch_dog` 或 `motor_error` 非零时触发 `motor_has_error`，进而调用 `rl_real_atdog2` 注册的错误回调并 `rclcpp::shutdown()`（`rl_real_atdog2.cpp:31-42`，`leg_driver.cpp:25-94`）。

这条反馈链用于 `GetState()` 和策略观测，不经过 ROS 话题发布。

## 8. 端到端时序表（手动行走为例）

| 步骤 | 数据形态 | 所在位置 | 执行操作 |
| --- | --- | --- | --- |
| 1 | 串口字节流 | 遥控器 → `/remote_node` | `0x5A` 包头、拼包、命令 `0x01` 分发 |
| 2 | 20 字节 payload | `RemoteNode::on_remote_control_data()` | `memcpy` 到 `lx/ly/rx/ry/key` |
| 3 | `Remote.msg` | `/remote` | 按串口包频率发布 |
| 4 | `Remote` 回调 | `/robot_calc_node` | bit 1 判自动/手动；bit 5 trigger → `mode=2` |
| 5 | `Remote` 回调 | `/robot_calc_node` | `ly/rx/lx` 映射并限幅为 `vx/vy/vz` |
| 6 | `Cmd.msg` | `/robot_move_cmd` | 50 ms 周期发布，约 20 Hz |
| 7 | `remote_cmd` | `/robot_controller_node` | 订阅回调整包缓存 |
| 8 | `control.x/y/yaw/mode` | `RobotControl()` | 二次限幅、`setMode()`、`setVel()` |
| 9 | FSM 状态 | `fsm_atdog2` | `mode=2` → `RLFSMStateRLLocomotion`，加载 `robot_lab` 策略 |
| 10 | `obs.commands` | `RunModel()` | 速度进入模型输入和步态时钟 |
| 11 | `actions` | `Forward()` | 推理生成动作并限幅 |
| 12 | `motor_command` | `RLControl()` | 写入 12 个关节 `q/dq/kp/kd/tau` |
| 13 | `LegTarget_t[4]` | `SetCommand()` | `joint_mapping` 重排，填充关节目标 |
| 14 | `DogTargetPack_t` | `LegDriver` | `pack_type=4`，280 字节 |
| 15 | USB bulk OUT | `CDCTrans::send()` | `libusb_bulk_transfer(EP_OUT=0x01)` |
| 16 | 电机运动 | STM32 + 485 总线 | 下位机闭环执行，机器人按遥控速度运动 |

## 9. 排障要点

1. **没有 `/joy`**：本链路是串口遥控器，launch 只启动 `remote_node` 和 `robot_calc_node`。
2. **话题名不带命名空间**：实际名称就是 `/remote` 和 `/robot_move_cmd`。
3. **速度限幅有两层**：`obstacle_game` 先限，`rl_real_atdog2` 再限，最终取更严格者。
4. **频率不同**：遥控器包按到达频率发布，`/robot_move_cmd` 约 20 Hz，`RobotControl()` 200 Hz，`RunModel()` 50 Hz，电机目标随控制循环 200 Hz 下发。
5. **`mode=0` 表示 ROS 不接管**：此时 `rl_real_atdog2` 不再用 `/robot_move_cmd` 的速度覆盖 `control.x/y/yaw`。
6. **串口断连会让速度归零**：`obstacle_game` 手动模式收到清零 `Remote` 后会把 `vx/vy/vz` 写成 0，但不会清零 `mode`。
7. **`wheel_vel` 无效**：`Cmd.msg` 有该字段，但 at dog2 的 12 自由度控制链没有读取它。
8. **电机异常会导致节点退出**：下位机 `watch_dog` 或 `motor_error` 非零时，`rl_real_atdog2` 收到回调后执行 `rclcpp::shutdown()`。
9. **最终去向是硬件不是话题**：`/robot_move_cmd` 之后没有“电机控制话题”；指令经由 `LegDriver` 和 USB CDC 直接写入 STM32。
