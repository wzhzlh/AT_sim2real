# Dog2 HIM 策略 Sim2Real 部署要求（修订版）

本文针对 `policy/atdog2/robot_lab/config.yaml` 对应的 Dog2 HIM 策略。部署时必须以同一份 checkpoint、导出物、配置和实机标定数据为准；不要把本文数值直接套用到其他策略。

## 1. 已核实接口

- 输入：`obs_history`，`float32`，`[1, 270]`；输出：`actions`，`float32`，`[1, 12]`。
- 单帧 45 维，历史长度 6；布局为 `[当前帧，上一帧，…，最旧帧]`，每帧连续 45 维。
- `policy_metadata.json`/history YAML 描述接口，不包含全部硬件标定参数。
- `policy.pt` 是否包含 estimator 必须通过导出脚本和 TorchScript graph 实际检查，不能仅凭扩展名推断。

## 2. 部署侧必须显式提供

关节名及策略到硬件的映射、`default_dof_pos`、`action_scale`、观测坐标系/单位/scale、策略 PD 增益、力矩/速度/位置限值、控制周期和超时阈值、命令来源及 heading 语义。上述参数应与 checkpoint 一起版本化，启动时检查长度、单位和范围。

## 3. 时序

`policy/atdog2/base.yaml` 核实 `dt=0.005 s`、`decimation=4`，策略周期为 `0.020 s`（50 Hz）。实机须用单调时钟实现并记录周期抖动、推理耗时和传感器年龄。仿真 actuator delay 不能宣称已在实机复现；应测量实机延迟，必要时重新训练。观测、历史推进、推理和发送必须绑定同一 tick；失败时不推进历史并进入安全模式。

## 4. 关节顺序与映射

策略顺序：`FR(hip,thigh,calf), FL(hip,thigh,calf), RR(hip,thigh,calf), RL(hip,thigh,calf)`。

当前 `base.yaml` 的 `joint_mapping` 实际值是恒等映射 `[0..11]`，但注释还保留了交换示例；这不是可自动推导的事实。必须按实际硬件验证映射，并在上电前逐关节小幅动作确认，禁止依赖 URDF/SDK 默认顺序。

## 5. 45 维观测

| 切片 | 内容 | scale |
|---|---|---:|
| `[0:3]` | 机体系角速度 `ang_vel` | 0.25 |
| `[3:6]` | 机体系重力方向 `gravity_vec` | 1.0 |
| `[6:9]` | 机体系命令 `[vx,vy,wz]` | 1.0 |
| `[9:21]` | `q - default_dof_pos` | 1.0 |
| `[21:33]` | 关节角速度 | 0.05 |
| `[33:45]` | 上一帧网络原始 action | 1.0 |

角速度、重力、命令必须转换到训练定义的 base 坐标系；IMU 外参、陀螺零偏和编码器零位先标定。部署不要加入训练噪声。是否存在 normalizer 以导出图和 metadata 为准，不要自行假设。启动时复制第一帧填满 6 帧，并用离线回放验证历史方向。

## 6. 动作与 PD

当前 `robot_lab/config.yaml` 核实值：

```python
default = [0,-0.8,0, 0,0.8,0, 0,-0.8,0, 0,0.8,0]
scale = [0.125,0.25,0.25] * 4
rl_kp = [25.0] * 12
rl_kd = [0.5] * 12
target = default + action * scale
torque = rl_kp * (target - q) - rl_kd * qd
```

配置中的 `fixed_kp=80`、`fixed_kd=3` 是另一套固定控制参数，不能与 `rl_kp/rl_kd` 混用；实际使用哪套必须由代码路径确认。发送前对 action、目标角和力矩做有限值检查及硬限幅。摩擦参数不是额外 PD 增益，除非标定后明确需要，否则不要重复补偿。

## 7. 命令

接口只有 `[vx, vy, wz]` 三维。当前仓库能确认维度和 scale，但 heading 混合比例、采样分布和控制器参数必须从该 checkpoint 对应训练环境再次核对，不能仅凭 `heading_command=True` 推断。命令须在机体系并限幅到训练范围；若实机摇杆直接给 `wz`，训练必须采用相同 direct-yaw 语义，否则重新训练或复现同一 heading 控制律。

## 8. 安全与验收

- 力矩上限以驱动器和实际 URDF 对应关节的较小值为准；`23.5 N·m` 仅是当前配置值。
- 位置、速度、温度限值来自实机手册和标定；左右镜像不能未经核对套用。
- NaN、过期传感器、推理超时、映射错误或超限时立即切阻尼/急停。
- 上电先零命令站立，逐步增加动作幅度，首次测试配机械支撑、急停和日志。

验收顺序：ONNX 与 TorchScript/训练框架同输入逐元素对比（建议 `max_abs_err < 1e-4`）；真实观测回放检查切片、scale、历史和首帧；静态检查零姿态输出与限幅；再做站立、低速直行/横移/转向，最后才提高速度和地形复杂度。

## 快速核对

```text
50 Hz；输入 [1,270] obs_history；输出 [1,12] actions
顺序 FR, FL, RR, RL × (hip, thigh, calf)
历史当前到最旧；启动复制填满；dof_pos=q-default
actions=上一帧原始输出；default=[0,-.8,0, 0,.8,0, 0,-.8,0, 0,.8,0]
scale=[.125,.25,.25]×4；rl_kp=25；rl_kd=.5
```
