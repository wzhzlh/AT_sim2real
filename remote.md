# 项目遥控器功能总表

这份文档汇总项目里所有和 `/remote` 有关的节点、按键 bit 语义、摇杆映射，以及 `obstacle_game` 中录制 YAML 时的按键复用逻辑。

## 1. 消息定义

`remote_node` 将物理遥控器数据解析后发布为 `robot_msgs/msg/Remote`：

```text
float32 lx
float32 ly
float32 rx
float32 ry
uint32 key
bool just_reconnected
```

- 原始串口包布局是 `float[4] + uint32_t`，总计 `20` 字节。
- `just_reconnected` 不是原始遥控器数据的一部分，而是 `remote_node` 在串口断开后重连成功时额外置位的一帧标记。
- 遥控器断开时，`remote_node` 会发布一次清零消息：`lx/ly/rx/ry=0`、`key=0`、`just_reconnected=false`。

相关文件：

- `src/robot_msgs/msg/Remote.msg`
- `src/remote_node/src/remote_node.cpp`

## 2. key 字段读取方式

项目里把 `key` 当作 `uint32_t` 位掩码使用。

- `check_key_pressed(key, index)`：检查某个 bit 当前是否为 `1`，用于持续状态判断。
- `check_key_trigger(key, index)`：检查某个 bit 当前帧是否从 `0 -> 1`，用于一次性触发。

对应实现：

```cpp
bool check_key_trigger(uint32_t current_key, int index)
{
    bool current_is_true = ((current_key >> index) & 0x0001);
    bool last_is_false = !((last_key >> index) & 0x0001);
    return current_is_true && last_is_false;
}

bool check_key_pressed(uint32_t current_key, int index)
{
    return ((current_key >> index) & 0x0001);
}
```

## 3. 项目内已使用的 bit

| bit | 用途 |
| --- | --- |
| `1` | 自动/手动模式拨杆 |
| `2` | 录制模式保持 |
| `3` | sand 策略 |
| `4` | stand 策略，或自动流程中的复位/调试触发 |
| `5` | walk 策略，或自动流程中的启动 |
| `6` | stairs 策略，或自动流程中的暂停 |
| `9` | `obstacle_game` 录点复用修饰键 |
| `11` | slope 策略，或在 `bit 9` 条件下作为录点选项 |
| `12` | bar 策略 |
| `13` | bridge 策略，或在 `bit 9` 条件下作为录点选项 |
| `14` | 录制一个路径点 |

## 4. 摇杆轴映射

### `obstacle_game`

手动模式下：

- `cmd.vy = -clamp(lx / 1200.0, -0.8, 0.8)`
- `cmd.vx =  clamp(ly / 1200.0, -1.2, 1.2)`
- `cmd.vz = -clamp(rx / 1200.0, -1.0, 1.0)`
- `ry` 当前未使用

### `task_game`

手动模式下：

- `cmd.vy = -clamp(lx / 1200.0, -1.2, 1.2)`
- `cmd.vx =  clamp(ly / 1200.0, -1.2, 1.2)`
- `cmd.vz = -clamp(rx / 1200.0, -1.0, 1.0)`
- `ry` 当前未使用

## 5. `obstacle_game` 遥控器功能

消费节点：

- `src/obstacle_game/src/core/robot.cpp`

### 5.1 模式切换

- `bit 1 = 0`：手动控制
- `bit 1 = 1`：自动控制

从自动切回手动时，节点会：

- 强制 `cmd.mode = 1`
- `pilot->reset()`
- `pilot->stop()`

### 5.2 手动模式下的策略切换

| bit | 行为 | `cmd.mode` | 录制时 `policy_id` |
| --- | --- | --- | --- |
| `4` | 位控站立 | `1` | `1` |
| `5` | 普通行走 | `2` | `2` |
| `6` | 台阶 | `3` | `3` |
| `3` | 沙地 | `4` | `4` |
| `11` | 斜坡 | `5` | `5` |
| `12` | 限高杆 | `6` | `6` |
| `13` | 木桥 | `7` | `7` |
| `10` | 翻墙 | `8` | `8` |

说明：

- `bit 11` 和 `bit 13` 在普通手动模式下仍然分别是 slope / bridge。
- 只有在“录制保持 + `bit 9` 修饰键”同时成立时，它们才会被复用成录点选项键。

### 5.3 自动模式下的控制

| bit | 行为 |
| --- | --- |
| `4` | 复位并停止自动 pilot |
| `5` | 启动自动 pilot |
| `6` | 暂停自动 pilot |

### 5.4 YAML 录点

录制保持由 `bit 2` 控制：

- 按住 `bit 2`：进入录制周期
- 首次进入时：创建一个新 YAML 文件
- 触发 `bit 14`：记录当前点位一次
- 松开 `bit 2`：结束录制并关闭文件

默认记录字段：

- `target_yaw = 当前机器人 yaw`
- `constraint_target_yaw = false`
- `allow_y_vel = false`
- `stand_at_target = false`
- `stand_duration = 0.0`
- `policy_id = current_record_policy_id`

### 5.5 `bit 9` 下的按键复用

当同时满足以下条件时进入复用逻辑：

- `bit 2` 处于按住状态，也就是正在录制
- `bit 9` 处于按住状态

这时：

- `bit 13` 不再切换 bridge 策略，而是给“下一次录点”挂一个一次性选项
- `bit 11` 不再切换 slope 策略，而是给“下一次录点”挂一个一次性选项

一次性选项内容如下：

| 复用键 | 下一次 `bit 14` 录点时附加写入的字段 |
| --- | --- |
| `bit 13` | `stand_at_target: true`、`stand_duration: 2` |
| `bit 11` | `constraint_target_yaw: true`、`allow_y_vel: true` |

行为细节：

- 这两个选项都只生效一次。
- 成功录下一个点后，对应待写入标记会立即清空。
- 录制结束时，这两个待写入标记也会被清空。

## 6. `task_game` 遥控器功能

消费节点：

- `src/task_game/src/core/robot.cpp`

### 6.1 模式切换

- `bit 1 = 0`：手动控制
- `bit 1 = 1`：自动控制

额外逻辑：

- 当 `msg.just_reconnected == true` 时，`task_game` 会忽略重连后的前几帧模式切换输入，避免串口刚恢复时误触发。

### 6.2 手动模式下的策略切换

| bit | 行为 | `cmd.mode` |
| --- | --- | --- |
| `4` | 位控站立 | `1` |
| `5` | 普通行走 | `2` |
| `12` | 限高杆 | `5` |

### 6.3 自动模式下的功能

- 当 `tree_debug_mode` 开启时，触发 `bit 4` 会推进一次行为树调试阶段。

## 7. `keyboard` 模拟遥控器功能

发布节点：

- `src/keyboard/src/keyboard_node.cpp`

这个节点发布同样的 `robot_msgs/msg/Remote`，主要用于本地联调。

### 7.1 模式键

| 键盘键 | 实际行为 |
| --- | --- |
| `2` | 手动模式，`key` 不置任何模式 bit |
| `3` | 自动模式，置 `bit 1` |
| `1` | 录制模式，置 `bit 2` |

### 7.2 摇杆模拟

| 键盘键 | 写入字段 |
| --- | --- |
| `w / s` | `ly +/-` |
| `a / d` | `lx -/+` |
| `q / e` | `rx -/+` |
| `space` | 清零 `lx/ly/rx` |

### 7.3 脉冲按键

| 键盘键 | 置位 bit |
| --- | --- |
| `z` | `4` |
| `x` | `5` |
| `c` | `6` |
| `v` | `3` |
| `g` | `10` |
| `n` | `11` |
| `b` | `12` |
| `h` | `13` |
| `m` | `14` |

限制：

- 当前键盘模拟器没有提供 `bit 9` 的按键，所以 `obstacle_game` 里“`bit 9` + `bit 11/13`”的录点复用逻辑只能通过物理遥控器触发，不能通过现有键盘映射直接触发。
