# apriltag_localization 使用与位姿配置指南

本包有两套独立工作流程。本文按当前源码整理；依赖安装、数学推导和更多调参说明见 [teach.md](teach.md)。

| 需求 | 启动文件 | 参数文件 | 是否查询 TF 树 | 输出 |
| --- | --- | --- | --- | --- |
| 看 tag 中心六维位姿、示教站位、走到 tag 前 | `tag_approach.launch.py` | `config/approach.yaml` | 否，使用两个常数矩阵 | 相机位姿日志、机体系调试位姿、速度指令 |
| 用 tag 将现实雷达 odom 对齐理想地图 | `apriltag_localization.launch.py` | `config/localizer.yaml` | 是 | `map -> odom`、`/odom_corrected`、调试位姿 |
| 兼容启动入口（同上） | `full_localization.launch.py` | `config/localizer.yaml` | 是 | 同上 |

`tag_approach` 支持 RealSense 和普通 UVC 摄像头；`apriltag_localizer` 当前只通过 librealsense2 读取 RealSense 彩色流。两者均检测 `tag36h11`，自行打开相机，不订阅 ROS 图像话题，不需要另启相机驱动。两个节点不能同时占用同一相机。

## 1. 编译和参数文件生效方式

需要 ROS2 Humble、`robot_msgs`、OpenCV、librealsense2 和 AprilTag C 库；odom 重定位不需要 AMCL 或地图服务器。即使只用 UVC，当前编译也需要 librealsense2。

```bash
cd /home/zhangjiayi/RL/AT_sim2real
source /opt/ros/humble/setup.bash
colcon build --packages-up-to apriltag_localization
source install/setup.bash
ros2 pkg executables apriltag_localization
```

应看到 `tag_approach` 和 `apriltag_localizer`。缺第三方库时 CMake 会跳过节点编译，因此“构建成功”不一定代表有可执行文件，安装方法见 [teach.md 第 1 节](teach.md#1-依赖安装)。

launch 默认加载 **install 目录**中的配置。修改源码目录下 YAML 后，可重新构建，或直接传源码配置的绝对路径：

```bash
ros2 launch apriltag_localization tag_approach.launch.py \
  params_file:=/home/zhangjiayi/RL/AT_sim2real/src/apriltag_localization/config/approach.yaml
```

本文参数均在启动时读取；修改 YAML 后重启节点。不要依赖 `ros2 param set` 更新算法内部的参数或矩阵，当前没有动态参数同步回调。自定义 YAML 保留根键 `/tag_approach` 或 `/apriltag_localizer`，以及 `ros__parameters`。

## 2. 坐标、单位和变换方向

全文 `T_A_B` 表示 **B 系在 A 系中的位姿**，同时把 B 系坐标转换到 A 系：

```text
p_A = T_A_B * p_B
T_A_C = T_A_B * T_B_C
T_B_A = inverse(T_A_B)
```

| 坐标系 | 原点 / 轴方向 |
| --- | --- |
| 机体系 B（`base_link`） | 机器人定义的本体原点；X 前、Y 左、Z 上 |
| 相机光学系 C（`camera_color_optical_frame`） | 相机彩色光学原点；X 图像右、Y 图像下、Z 沿光轴向前 |
| tag 系 | 黑色检测外框中心；轴由解码角点定义，见下文 |
| map 系 M | 与建图地图一致的世界坐标；Z 向上，XY 为地面平面 |
| 雷达系 L | 驱动扫描消息的 `header.frame_id`，按实际安装定义 |

距离单位为米。`camera_in_parent`、`tag_poses` 的 RPY 和相机位姿日志为**度**；控制参数 `yaw_tolerance`、`yaw_align_band` 为**弧度**，`max_wz` 为 rad/s。RPY 统一采用 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`。ROS 位姿话题的 orientation 是四元数 `(x,y,z,w)`。

当前两个节点对检测角点采用同一物点对应关系，`s = tag_size / 2`：

```text
p[0] = (+s,+s,0)    p[1] = (-s,+s,0)
p[2] = (-s,-s,0)    p[3] = (+s,-s,0)
+x：p[1] -> p[0]    +y：p[3] -> p[0]    +z：+x × +y
```

角点编号跟随解码方向，不能把 `p[0]` 永久当作图像右上角。tag 转动时，轴的图像投影也会转动；当前约定下 +Z 为码正面外法向。预览中**红=X、绿=Y、蓝=Z**，画在 tag 中心的是 **tag 轴在图像中的投影**，不是相机轴。红轴向左、绿轴向下可以是合法姿态，不能单凭方向判断错误。

## 3. 纯视觉：识别、打印、示教和趋近

### 3.1 先只看位姿

编辑 `config/approach.yaml`：

```yaml
/tag_approach:
  ros__parameters:
    # 以下仅列需修改项，保留原文件中的矩阵等其它参数
    image_source: "uvc"       # D435 用 "realsense"；"auto" 优先 RealSense
    uvc_device: "/dev/video2"
    tag_id: 5                 # 按预览 ID 填；-1 接受任意 ID（一帧取首个有效码）
    tag_size: 0.118           # 实测黑色检测外框边长，不含白边和整张纸
    cmd_output: "none"        # 只观察，不发布运动指令
    auto_start: false
```

```bash
ros2 launch apriltag_localization tag_approach.launch.py \
  params_file:=/home/zhangjiayi/RL/AT_sim2real/src/apriltag_localization/config/approach.yaml
# 无桌面时在同一命令末尾加 show_image:=false
```

目标 ID 匹配且 PnP 有效时，每秒打印一次：

```text
T_C_tag | x=... y=... z=... m | roll=... pitch=... yaw=... deg
```

这是 **tag 中心在相机光学系中的六维位姿**，不需要调用 `/start`，也不经过 `T_B_C`。`x>0` 表示中心在主点右侧，`y>0` 表示在下方，`z>0` 表示在相机前方。`z` 是轴向深度，中心到相机原点的直线距离为 `sqrt(x*x+y*y+z*z)`；tag 倾斜时 z 也不等于相机到 tag 平面的垂直距离。

RPY 取决于 tag 轴向，正对相机也不必为 `(0,0,0)`。例如 tag +X 朝图像右、+Y 朝图像上时，正对姿态约为 `(±180°,0°,0°)`。一般姿态下不要把 Euler 各角直接理解成绕 tag 自身三轴的独立转角。

普通 UVC 必须在实际采集分辨率下标定并填写 `camera_fx/fy/cx/cy` 和 `distortion: [k1,k2,p1,p2,k3]`。当前示例内参全为 0，节点按 `hfov_deg: 60.0` 估算，**包括 z 在内的位姿都可能有系统误差**。RealSense 从 SDK 获取内参。更换分辨率、镜头或对焦后需重新核对标定；`tag_size` 错误会按比例影响位置尺度。

### 3.2 修改相机安装位置：`T_B_C`

只改 `config/approach.yaml` 的 `T_B_C`。它表示相机光学原点和三轴在机体系中的位置、方向，**不能拿 camera_link 的外参直接替代光学系外参**。

矩阵按行主序写 16 个浮点数：

```text
[ r00, r01, r02, tx,
  r10, r11, r12, ty,
  r20, r21, r22, tz,
  0.0, 0.0, 0.0, 1.0 ]
```

左上 3×3 的三列分别为“相机 X、Y、Z 轴在机体系中的方向”，最后一列是相机原点在机体系中的坐标。水平前视、安装在机体前 0.20 m、高 0.15 m 的示例：

```yaml
T_B_C: [ 0.0,  0.0,  1.0,  0.20,
        -1.0,  0.0,  0.0,  0.00,
         0.0, -1.0,  0.0,  0.15,
         0.0,  0.0,  0.0,  1.00]
```

安装姿态不变、改为前 0.30 m、机体左侧 0.02 m、高 0.25 m，则只把最后一列前三项改为 `0.30, 0.02, 0.25`。相机偏转或俯仰则同时修改旋转部分；绕哪个坐标系旋转决定矩阵乘法顺序，不能任意交换。

若已有机器人 TF，可用 `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 读取机体到光学系的外参，将其四元数转换为旋转矩阵再填入。`tag_approach` 本身不会读取这条 TF，改 URDF / 静态 TF 不会自动改变该节点的 `T_B_C`。

### 3.3 修改目标位置和朝向：示教 `T_tag_B_goal`

推荐用真实站位示教，避免手推码轴：

1. 校准 `T_B_C`、相机内参和 `tag_size`，保持 `auto_start: false`。
2. 把机器人摆在最终希望停止的位置和朝向，让相机看到目标 tag。
3. 另开终端加载工作区，调用：

```bash
ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger '{}'
```

4. 服务先确认“已请求”；等下一帧有效检测后，终端才打印实际矩阵。节点执行 `T_tag_B_goal = inverse(T_B_C * T_C_tag)`，并立即在本次运行使用新目标。
5. 将打印的 16 个数按行主序保存到 YAML 的 `T_tag_B_goal`，重启后才会保留。当前服务不会自动写文件。

该矩阵的平移是在 **tag 系**下的目标机体原点位置，不是相机到 tag 的距离。不能只把某个数改成“想停多远”而忽略相机安装偏移、tag 高度和轴向。若目标姿态不变，手动改最后一列是在 tag 坐标轴方向移动目标；改左上 3×3 是改变目标机体朝向。

相机重新安装、换码/旋转码、改变物点轴约定后应重新核对外参并示教。tag 在场地中移动后，纯视觉目标会随 tag 移动，它不是固定 map 点。

运行时位姿链为：

```text
T_B_tag  = T_B_C * T_C_tag
T_B_goal = T_B_tag * T_tag_B_goal
```

日志 `机体系误差 dx/dy/dz/yaw` 描述目标机体相对当前机体的位置和朝向；`dx>0` 是目标在前方，`dy>0` 是目标在左侧。到达示教位姿时 `T_B_goal` 应接近单位矩阵，平移与 yaw 误差接近 0。

### 3.4 趋近控制和调试接口

完成干跑后，将 `cmd_output` 改为 `robot_move_cmd` 或 `cmd_vel`，重启节点，再操作：

```bash
ros2 service call /tag_approach/start std_srvs/srv/Trigger '{}'
ros2 service call /tag_approach/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /tag_approach/tag_pose_body
ros2 topic echo /tag_approach/goal_pose_body
```

| 项目 | 修改项 / 行为 |
| --- | --- |
| 本工作区机器人指令 | `cmd_output: "robot_move_cmd"`，`cmd_topic` 指定话题；`Cmd.vz` 为偏航角速度，不是竖直速度；`cmd_mode: 2` 行走，`stand_mode: 1` 站立 |
| 通用底盘 | `cmd_output: "cmd_vel"`，`twist_topic` 指定话题，输出 `linear.x/y` 和 `angular.z` |
| 控制方式 | `holonomic` 控制 dx/dy/yaw；`differential` 朝目标点转向再前进，不输出横移，最终原地对齐目标朝向的逻辑有限，需另行验证 |
| 增益和限速 | `kp_x/y/yaw`、`max_vx/vy/wz`、`allow_lateral`、`yaw_align_band` |
| 频率和平滑 | `control_rate: 50.0`、`smooth_tau: 0.10`（秒）；视觉更新频率独立于控制频率 |
| 到位 | `pos_tolerance: 0.05`（XY 距离）、`yaw_tolerance: 0.05`（rad）、`hold_time: 0.30`（秒）；不控制高度 |
| 丢码 | 超过 `lost_timeout` 发零速；`resume_on_reacquire: true` 自动恢复，否则需重新 `/start` |
| 停止 | `/stop` 发一次零速并停止继续发布；到位后则持续发零速，直到停止服务交还控制权 |

`publish_debug_poses: true` 时，两个调试话题均为 `PoseStamped`，`frame_id` 固定为 `base_link`。当前趋近节点**没有** `/tag_approach/tag_pose_camera` 话题，也不广播 tag TF；相机六维位姿看 `T_C_tag` 日志。RViz 显示机体系调试位姿时可将 Fixed Frame 设为 `base_link`。

## 4. 全局重定位：修改 tag 场地位置和 TF

### 4.1 配置 tag 在 map 中的真实位姿

修改 `config/localizer.yaml` 的 `tag_poses`，一行一个 ID，位置取 tag 中心，角度按第 2 节的实际 tag 轴填写：

```yaml
tag_poses:
  - "5, 2.50, 1.20, 0.80, 90.0, 0.0, 0.0"
target_tag_id: 5
```

这行示例表示 tag 中心在 map `(2.50,1.20,0.80)`，tag +X 指向 map +X、+Y 指向 map +Z、+Z 指向 map −Y（竖直墙面）。仅适用于这些轴向，不能把所有墙面都填成相同 RPY。未配置的 ID 会被忽略，`target_tag_id: -1` 接受任意已标定 ID。更换物理码或移动 tag 后，在这里修改场地位姿；**趋近模式不读取这个表**。

若已知机器人真实六维位姿，可用完整矩阵标定：

```text
T_M_tag = T_M_B_known * T_B_C * T_C_tag_measured
```

从 `/apriltag_localizer/tag_pose_camera` 获取 `T_C_tag_measured`，从 TF 获取 `T_B_C`，把结果转成位置和 RPY 度数写回。仅比较最终 XY 误差并累加到 tag 坐标不能普遍修正旋转错误。全局节点没有 `/record_goal` 服务，也不会自动保存 tag 场地标定。

### 4.2 相机外参和雷达外参改在哪里

全局节点不读取 `approach.yaml` 中的 `T_B_C`。它查询：

```text
lookupTransform(lidar_frame, camera_frame) -> T_L_C
lookupTransform(lidar_frame, base_frame)   -> T_L_B
T_L_tag = T_L_C * T_C_tag
T_M_L   = T_M_tag * inverse(T_L_tag)
T_M_B   = T_M_L * T_L_B
```

TF 树应连通，例如：

```text
map --Tag 校正--> odom --里程计--> base_link
                               ├──静态外参──> laser
                               └──静态外参──> camera_color_optical_frame
```

`full_localization.launch.py` 不启动雷达驱动、里程计、机器人 URDF 或底层运动节点，这些需另行启动。相机可经过 `camera_link` 等中间帧，只要最终光学系连通即可。

| 要修改什么 | 修改位置 |
| --- | --- |
| TF 已由 URDF / robot_state_publisher 提供的相机安装位置 | 修改对应 fixed joint 的 `origin xyz/rpy`；URDF 的 rpy 是弧度，需包含到光学系的固定轴变换 |
| 相机 TF 由外部 static_transform_publisher 提供 | 修改该发布器的平移和旋转，保证 child 最终为正确光学系 |
| 希望由本包发布相机静态 TF | 在 `localizer.yaml` 设置下方参数 |
| 雷达安装外参 | 修改 URDF 或外部 `base_link -> laser` 静态发布器；本节点没有发布雷达外参的参数 |
| 坐标系名字变化 | 同步修改 localizer.yaml、实际消息和 TF 的帧名 |

本包发布相机外参的示例（只在没有其它发布者提供该 child TF 时使用）：

```yaml
publish_camera_tf: true
camera_tf_parent: "base_link"
camera_frame: "camera_color_optical_frame"
camera_in_parent: [0.20, 0.0, 0.15, -90.0, 0.0, -90.0]  # m / 度
```

此处同样表示水平前视、前 0.20 m、高 0.15 m，等价于第 3.2 节 `T_B_C` 示例。父帧改为 `laser` 后，六个数也必须改成光学系在雷达系中的位姿。已经有相机 TF 时保持 `publish_camera_tf: false`，避免同一 child 被重复发布或出现多个父帧。

帧名不会生成外参。`map_frame` 是理想地图/示教轨迹的坐标系；Tag 必须在这个坐标系下标定，不能直接填相对现实 odom 的位置。

### 4.3 重定位与校正里程计

雷达先启动并发布 `nav_msgs/Odometry`。设置 `odom_topic`（默认 `/odom`）、`odom_frame`（必须匹配输入 header.frame_id）和 `base_frame`。输入 child 可以是雷达或机体，需具备 child 到 base 的静态外参。然后启动：

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py
ros2 topic echo /odom_corrected
ros2 run tf2_ros tf2_echo map odom
```

逐帧按图像时间在里程计缓存中插值，并计算：

```text
T_M_O = T_M_B(Tag) * inverse(T_O_child(同一时刻) * T_child_B)
校正输出：T_M_child = T_M_O * T_O_child
```

多帧平均的是校正变换，不是机器人位置，因此允许运动中采样。保留完整 xyz 和旋转，平移/角度离散超限则丢弃本轮。RealSense 系统/全局时间直接匹配 ROS 系统时间；硬件时间域下以图像接收时刻近似，会受到传输延迟影响。输入和图像必须使用同一时钟。

`auto_once` 首次对齐后固定修正，Tag 离开视野后仍靠雷达增量运动；`continuous` 在检测到 Tag 时不断纠偏；`manual` 等待服务触发。重新采样期间继续使用旧修正：

```bash
ros2 service call /apriltag_localizer/relocalize std_srvs/srv/Trigger "{}"
```

`/odom_corrected` 与输入频率、时间戳和 child_frame_id 一致，header.frame_id 为理想地图；位姿左乘校正变换，pose 协方差旋转，child 系下的 twist 保留。未对齐前不发布校正 odom/TF。只广播 `map -> odom`，不重复广播雷达的 `odom -> child`，也不广播额外的 `map -> child`。不要让 AMCL 或其它节点同时发布同一条 `map -> odom`。

固定修正能消除启动坐标偏差；后续雷达漂移需要再次观察 Tag 才能修正。这里输出的是地图系下的估计 odom，精度依赖 Tag 尺寸、地图标定、相机外参与时间同步。雷达里程计重置后应重启本节点并重新对齐；无法仅凭 odom 消息可靠判断坐标原点是否被重置。

旧 `full_localization.launch.py` 现为相同功能的兼容入口，参数文件使用 `localizer_params`；不再启动 AMCL 和地图服务器。旧 `output_mode`、`require_subscriber`、`sigma_*`、`tf_max_step_*` 参数已移除。

## 5. 现场修改速查与排错

| 现象 / 需求 | 优先核对 |
| --- | --- |
| 只需识别中心和打印六维位姿 | 趋近模式 `cmd_output: "none"`、目标 ID、相机内参；不用 `/start` |
| 换 tag 尺寸或 ID | 两套 YAML 各自修改 `tag_size` 和 `tag_id` / `target_tag_id`；全局模式还需补 `tag_poses` |
| 改相机安装位置 | 趋近模式改 `T_B_C`；全局模式改真实 TF 或 `camera_in_parent`；两套不会自动同步 |
| 改机器人最终站位 / 朝向 | 重新 `/tag_approach/record_goal`，保存 `T_tag_B_goal` |
| tag 在地图里挪动或转动 | 全局模式重标 `tag_poses`；纯视觉站位随 tag 移动，按需求重新示教 |
| 距离整体成固定比例错误 | 黑色外框边长、UVC 焦距内参和分辨率 |
| 有框但没有 `T_C_tag` | 目标 ID 不符、PnP 失败或深度不在 `(0,10]` m |
| 有 `T_C_tag` 但无机体系误差 / 轴 | 趋近节点倾角检查未通过；核对 `T_B_C` 并重新示教，日志中的 PnP 位姿在倾角检查前已输出 |
| 目标倾角异常 | 检查变换方向、旋转是否填反、相机光学系和 tag 轴，不要靠调大 `max_tilt_deg` 掩盖错误 |
| TF 查不到 | 两个查询帧是否连通、帧名是否一致；`tag_poses` 和调试 PoseStamped 本身不会广播 TF |
| 改 YAML 不生效 | 检查加载的是 install 还是源码路径，重启节点，核对启动时打印的矩阵 |
| 无桌面 / 远程运行 | 趋近 launch 用 `show_image:=false`；全局模式在 `localizer.yaml` 设置 `show_image: false` |
| 启动待命但没有速度 | `auto_start: false` 是默认行为；核对输出模式、底层接收节点和 `/start` |

示例参数中的安装位置、目标矩阵和 map 坐标均需现场替换。先核对相机位姿，再核对机体系误差，最后接入运动或 odom 校正，便于定位是哪一段变换有误。

## 6. MuJoCo 中识别 ID 5 并趋近

支持 `atdog2`（默认）和 `atdog3`。新增的 `scene_tag` 是平地测试场景，
机器人前方世界坐标 `(2, 0, 0.5)` 放置 tag36h11 / ID 5，黑框边长 **0.24 m**。
标签使用 AprilTag 官方码表生成的黑白几何单元，不依赖外部纹理。
机身前视相机位置为 `(0.1948, 0, 0.1465)`，图像 `640×480`，垂直视场角 `60°`。
识别器从 ROS 图像和精确相机内参运行真实 AprilTag 检测与 PnP，不使用标签真值。

编译（工作区根目录）：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/rl_sar -B cmake_build \
  -DUSE_CMAKE=ON -DUSE_MUJOCO=ON -DMUJOCO_ROS2=ON
cmake --build cmake_build --target rl_sim_mujoco -j2
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select apriltag_localization
source install/setup.bash
ros2 launch apriltag_localization tag_approach.launch.py use_mujoco:=true
# 另一型号：追加 robot:=atdog3
# 关闭检测预览：追加 show_image:=false（MuJoCo 本身仍需要桌面）
```

启动后即可看到 ID 和位姿日志，默认待命，不发趋近速度。要测试行走：

1. 点击 MuJoCo 窗口，使其获得键盘焦点。
2. 确认仿真正在运行（`Enter` 切换暂停/继续），按 `0` 起身并等待站稳。
3. 按 `1`：自动调用视觉跟踪启动、启用 ROS 速度输入，并切入强化学习策略行走。
4. 到位后站住，但持续识别与跟踪 ID 5；标签移动或机体偏差超过 **8 cm / 8°** 时自动恢复行走，调整到新目标。
5. 随时按 `0` 停止自动跟踪行走并站立；检测画面继续更新。再次按 `1` 恢复自动跟踪。

不需要再按 `N` 或另开终端调用 `/start`。`N` 和 start/stop 服务仍保留给手动调试。
若标签丢失会站立停车，标签重新出现后继续跟踪。只有 `1` 开启自动行走，启动或按 `0` 均不会自动前进。

仿真参数 `config/approach_mujoco.yaml` 在 `approach.yaml` 后加载，覆盖图像源、
标签尺寸、目标矩阵及输出话题；目标为机体原点距标签平面 **0.6 m**，正对标签。
修改场景标签位置后目标随标签移动；修改相机安装位置时同步修改 `T_B_C`。
不同于终端直接运行，ROS launch 不转发标准输入，因此这些按键已接入 MuJoCo 窗口。

| 话题 | 类型 / 用途 |
| --- | --- |
| `/mujoco/camera/image_raw` | `sensor_msgs/Image`，RGB 仿真相机画面 |
| `/mujoco/camera/camera_info` | `sensor_msgs/CameraInfo`，由垂直视场角计算的无畸变内参 |
| `/mujoco/tag_pose` | `geometry_msgs/PoseStamped`，`frame_id: world`，移动仿真标签供跟踪实验 |
| `/mujoco/stand` | `std_msgs/Bool`，到位、丢码、停止请求位控站立 |
| `/mujoco/base_pose` | `geometry_msgs/PoseStamped`，世界系机体真值，仅用于验收 |
| `/mujoco/cmd_vel` | `geometry_msgs/Twist`，视觉趋近输出，按 `1` 启动自动跟踪后被策略采用 |
| `/tag_approach/goal_pose_body` | 当前机体系下的目标位姿，用于核对距离和方向 |

丢码沿用 `lost_timeout=0.5` 秒停车；仿真桥在速度或模式话题超过 0.5 秒未更新时切换位控站立。
到位、`0` 与 `/stop` 明确切到位控站立，避免零速策略漂移。
仿真默认 `track_after_arrival: true`、`latch_goal: false`：到位后继续评估标签位姿，
在 `tracking_resume_distance: 0.08` m 或 `tracking_resume_yaw_deg: 8.0` 以外重新行走。
仿真起始到位阈值为 4 cm，制动保持期间允许两倍阈值，给行走切换站立时的机体移动留出余量。
`minimum_tracking_speed: 0.09` m/s 高于策略步态时钟的 0.08 m/s 启动阈值，
避免小误差调整时有速度指令却不能恢复迈步。实机默认不启用该最小速度补偿。

标签场景采用 `implicitfast` 积分器和 1 ms 物理步长；电机控制与推理按仿真时间调度，
分别每 5 ms / 20 ms 更新，PD 力矩在每个物理子步重新计算。相机只在复制渲染场景时持有
物理锁，GPU 渲染和读回不阻塞电机控制。`/mujoco/base_pose` 仅发布仿真真值供验收使用，
识别与趋近控制不读取真值。
原有实机命令 `ros2 launch apriltag_localization tag_approach.launch.py` 保持原用法。
若二进制在其他位置，传 `mujoco_executable:=/绝对路径/rl_sim_mujoco`。


### 自动闭环验收

在有 X11 桌面的终端运行（依赖系统 `libXtst`、`xwininfo`、Python OpenCV / NumPy）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/apriltag/validate_mujoco_approach.py --output /tmp/tag_validation/normal
python3 scripts/apriltag/validate_mujoco_approach.py --interrupt --output /tmp/tag_validation/interrupted
python3 scripts/apriltag/validate_mujoco_approach.py --move-tag --output /tmp/tag_validation/following
```

脚本启动真实 MuJoCo 查看器与原有策略，仅按 `0` / `1`，校验识别、
前进、到位、最终平面误差 <7 cm（含切换站立后的位移）、偏航误差 <5°、站立保持 10 秒漂移 <1.5 cm，
以及按 `0` 后保持。`--move-tag` 在到位后移动标签 0.3 m，验证无需再次按键就能恢复 RL
行走并到达新目标，再按 `0` 并移动标签，验证停止状态不会自行跟随。`--interrupt` 会在运动中暂停识别进程 3 秒，验证通信看门狗停车，
然后恢复识别并完成趋近。每轮保存进程日志、初始/最终相机图像、完整轨迹和验收 JSON；
断言失败时返回非零。测试使用独立 `ROS_DOMAIN_ID=89`，不要同时运行两轮。
软件 OpenGL 环境可在命令前加 `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa`。
