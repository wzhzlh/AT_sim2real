# apriltag_localization 功能与原理文档

> 面向 `AT_sim2real` 工作区（ROS 2 Humble）。本文是本功能包的**唯一文档**，覆盖当前源码中的
> 两个节点、全部参数、数学推导、现场标定、启动命令、验证与排错。
>
> **当前实现（2026-11）**：`apriltag_localizer` 用 AprilTag 把「现实雷达 odom」对齐到
> 「理想地图 map」，持续广播 `map -> odom` 并发布 `/odom_corrected`；
> `tag_approach` 是不依赖地图/TF 的纯视觉趋近节点。
> 当前链路**不依赖 AMCL、`/initialpose`、`map_server`**。
> `config/amcl.yaml`、原 `full_localization.launch.py` 属于历史方案，见第 9 节。

---

## 0. 功能总览

### 0.1 两个节点、两套流程

| 需求 | 节点 / 启动文件 | 参数文件 | 是否查 TF | 主要输出 |
| --- | --- | --- | --- | --- |
| 用 Tag 把现实雷达 odom 对齐理想地图 | `apriltag_localizer` / `apriltag_localization.launch.py` | `config/localizer.yaml` | 是 | `map -> odom`（TF）、`/odom_corrected`（Odometry）、3 个调试位姿 |
| 看 Tag 六维位姿、示教站位、走到 Tag 前 | `tag_approach` / `tag_approach.launch.py` | `config/approach.yaml` | 否，用两个 4x4 常数矩阵 | 速度指令、2 个调试位姿、位姿日志 |
| 实机一键部署（底层控制 + 趋近 + GUI） | `real_deployment.launch.py` | `config/approach.yaml` | 否 | 同上 |

两套流程**互相独立**：`tag_approach` 不读 `tag_poses`，`apriltag_localizer` 不读 `T_B_C`、
`T_tag_B_goal`。两个节点都自行打开相机、不订阅 ROS 图像话题（`tag_approach` 的 `ros`
图像源除外），因此**不能同时占用同一台相机**。

- `apriltag_localizer` 只通过 `librealsense2` 读取 D435 彩色流，内参由 SDK 给出。
- `tag_approach` 支持 `auto` / `realsense` / `uvc` / `ros` 四种图像源。

### 0.2 文件结构

| 文件 | 作用 |
| --- | --- |
| `include/apriltag_localization/pose_math.hpp` | 纯数学：solvePnP → `tf2::Transform`、位姿链、4x4 矩阵解析/生成、tag 标定表解析 |
| `include/apriltag_localization/odom_correction.hpp` | 旧版 `output_mode=tf` 的平面化/限幅数学，**当前源码未引用**，仅历史遗留 |
| `src/apriltag_localizer.cpp` | 全局重定位节点：D435 取图、Tag 检测、查 TF、配对 odom 历史、广播 `map -> odom`、发布 `/odom_corrected` |
| `src/tag_approach.cpp` | 纯视觉趋近节点：4x4 常数位姿、多图像源、视觉伺服、示教与急停 |
| `config/localizer.yaml` | 全局节点参数（帧名、tag 场地位姿、odom 话题、采样/时间同步、外参） |
| `config/approach.yaml` | 趋近节点参数（两个 4x4 矩阵、图像源/内参、控制与安全） |
| `config/approach_mujoco.yaml` | `use_mujoco:=true` 时叠加在 `approach.yaml` 之上的仿真参数 |
| `config/amcl.yaml` | **历史文件**，AMCL 方案遗留，当前 launch 不加载 |
| `launch/apriltag_localization.launch.py` | 只启动 `apriltag_localizer`；雷达、里程计、外参 TF 需另外启动 |
| `launch/tag_approach.launch.py` | 只启动 `tag_approach`；可带 `use_mujoco:=true` |
| `launch/real_deployment.launch.py` | 实机入口：`rl_real_atdog2` / `rl_real_atdog3` + `tag_approach` |
| `CMakeLists.txt` / `package.xml` | ament_cmake 包定义 |

### 0.3 快速开始

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# ① 实机 Tag 趋近（默认待命，需 /tag_approach/start 才开始）
ros2 launch apriltag_localization real_deployment.launch.py
# atdog3：追加 robot:=atdog3；普通 USB 相机：追加 image_source:=uvc

# ② 全局重定位 / odom 校正（需先启动雷达 odom 与外参 TF）
ros2 launch apriltag_localization apriltag_localization.launch.py

# ③ 无雷达纯视觉测试（不需要 map / AMCL / TF 树）
ros2 launch apriltag_localization tag_approach.launch.py
```

趋近节点启动后默认待命：

```bash
ros2 service call /tag_approach/start std_srvs/srv/Trigger "{}"
ros2 service call /tag_approach/stop  std_srvs/srv/Trigger "{}"
```

`real_deployment.launch.py` 需要本机桌面或图形转发（OpenCV 预览窗口）；
不要同时另开机器人控制程序或占用同一相机的驱动。

---

## 1. 依赖与编译

### 1.1 依赖安装

需要 ROS 2 Humble、`rclcpp`、`tf2_ros`、`nav_msgs`、`robot_msgs`、OpenCV、
`librealsense2` 和 AprilTag C 库。**odom 校正不需要 AMCL 或地图服务器**；
即使 `tag_approach` 只用 UVC/ROS 图像，当前编译仍需要 librealsense2。

```bash
# OpenCV（PnP、图像处理）
sudo apt install libopencv-dev

# librealsense2（Intel 官方 apt 源，Ubuntu 22.04 / jammy）
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-dev librealsense2-utils
rs-enumerate-devices          # 检查 D435

# AprilTag C 库（Ubuntu 22.04 universe 自带）
sudo apt install libapriltag-dev

# 查 TF 时常用
sudo apt install ros-humble-tf2-tools
```

无 sudo 权限时可把 AprilTag 免 root 安装到用户目录：

```bash
cd /tmp
apt-get download libapriltag-dev libapriltag3
rm -rf /tmp/apriltag-extract && mkdir -p /tmp/apriltag-extract \
    ~/.local/include ~/.local/lib ~/.local/lib/pkgconfig
for f in libapriltag*.deb; do dpkg-deb -x "$f" /tmp/apriltag-extract; done
cp -a /tmp/apriltag-extract/usr/include/apriltag ~/.local/include/
cp -a /tmp/apriltag-extract/usr/lib/x86_64-linux-gnu/libapriltag* ~/.local/lib/
cp -a /tmp/apriltag-extract/usr/lib/x86_64-linux-gnu/pkgconfig/apriltag.pc \
    ~/.local/lib/pkgconfig/
```

也可源码安装到 `/usr/local`：

```bash
git clone https://github.com/AprilRobotics/apriltag.git /tmp/apriltag
cmake -S /tmp/apriltag -B /tmp/apriltag/build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/apriltag/build --target install && sudo ldconfig
```

> `CMakeLists.txt` 用 `find_path(apriltag/apriltag.h)` + `find_library(apriltag)` 搜索
> `/usr`、`/usr/local`、`~/.local`，不依赖 `pkg-config`（Ubuntu 的 `apriltag.pc` 不含
> `Cflags`）。库在用户目录时会写入 RUNPATH，`ros2 run` 无需设置 `LD_LIBRARY_PATH`。

检查清单：

```bash
pkg-config --modversion opencv4       # 4.5.x
pkg-config --modversion realsense2    # 2.5x.x
ls /usr/include/apriltag/apriltag.h ~/.local/include/apriltag/apriltag.h 2>/dev/null
```

### 1.2 编译与参数生效方式

```bash
cd /home/zhangjiayi/RL/AT_sim2real
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select apriltag_localization
source install/setup.bash
ros2 pkg executables apriltag_localization    # 应看到 apriltag_localizer 和 tag_approach
```

> 第三方库缺失时 CMake 只打印 `STATUS` 并**跳过节点编译**，所以「构建成功」不等于有可执行文件。
> 补齐依赖后 `rm -rf build/apriltag_localization` 重新编译。
> 本包启用 `-Wall -Wextra -Wpedantic`；第三方头文件以 `-isystem` 引入，避免
> AprilTag 的 C 柔性数组告警。

launch 默认加载 **install 目录中的配置**。修改源码目录下的 YAML 后，可重新构建，
或直接传源码配置的绝对路径：

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py \
  params_file:=/home/zhangjiayi/RL/AT_sim2real/src/apriltag_localization/config/localizer.yaml
ros2 launch apriltag_localization tag_approach.launch.py \
  params_file:=/home/zhangjiayi/RL/AT_sim2real/src/apriltag_localization/config/approach.yaml
```

参数均在启动时读取，修改 YAML 后必须重启节点。当前**没有动态参数回调**，
不要依赖 `ros2 param set` 更新算法内部的参数或 4x4 矩阵。自定义 YAML 要保留根键
`/apriltag_localizer` 或 `/tag_approach` 以及 `ros__parameters`。

---

## 2. 坐标、单位与变换方向

### 2.1 记号与帧

全文 `T_A_B` 表示 **B 系在 A 系中的位姿**，同时把 B 系坐标变换到 A 系：

```text
p_A = T_A_B * p_B
T_A_C = T_A_B * T_B_C
T_B_A = inverse(T_A_B)
```

| 记号 | 帧 | 说明 |
| --- | --- | --- |
| M | `map`（`map_frame`） | 理想地图 / 示教轨迹坐标系；Tag 必须在这个坐标系下标定 |
| O | `odom`（`odom_frame`） | 现实雷达里程计系，必须等于输入 `Odometry.header.frame_id` |
| B | `base_link`（`base_frame`） | 机器人本体原点 |
| L | `laser`（`lidar_frame`） | 雷达系，与 `base_link` 之间是固定外参 |
| C | `camera_color_optical_frame`（`camera_frame`） | D435 彩色光学系：X 右、Y 下、Z 前 |
| `tag` | 码自身 | 原点=黑色检测外框中心，+z 为码正面外法向 |

单位：距离一律为 **米**。`camera_in_parent`、`tag_poses` 的 RPY 和位姿日志为**度**；
`yaw_tolerance`、`yaw_align_band` 为**弧度**，`max_wz` 为 rad/s。
RPY 统一为 \(R = R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})\)。
ROS 位姿话题的 orientation 是四元数 `(x,y,z,w)`。

### 2.2 角点 / 物点约定

两个节点使用同一套物点对应关系，`s = tag_size / 2`：

```text
p[0] = (+s,+s,0)    p[1] = (-s,+s,0)
p[2] = (-s,-s,0)    p[3] = (+s,-s,0)
+x：p[1] -> p[0]    +y：p[3] -> p[0]    +z：+x × +y
```

角点编号跟随解码方向，不能把 `p[0]` 永久当作图像右上角。tag 转动时轴的图像投影
也会转动；当前约定下 +Z 为码正面外法向。预览中**红=X、绿=Y、蓝=Z**，画在 tag 中心的是
**tag 轴在图像中的投影**，不是相机轴；红轴向左、绿轴向下可以是合法姿态。

### 2.3 TF 树

```text
map --(本节点发布)--> odom --(雷达里程计)--> odom 的 child_frame_id
                                              │
                                              ├─静态外参─> base_link
                                              ├─静态外参─> laser
                                              └─静态外参─> camera_color_optical_frame
```

`apriltag_localizer` 内部需要成立的三类 TF 查询（都是静态外参，用
`lookupTransform(..., tf2::TimePointZero)` 取最新值）：

1. `lookupTransform(lidar_frame, camera_frame)` → `T_L_C`（相机外参）
2. `lookupTransform(lidar_frame, base_frame)` → `T_L_B`（雷达→本体外参）
3. `lookupTransform(odom_child, base_frame)` → `T_child_B`（里程计 child 到本体；
   `odom_child` 来自输入消息的 `child_frame_id`，可以是雷达或机体）

帧名相同时 `lookupTf()` 直接返回单位变换，无需真实 TF。检查命令：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo laser camera_color_optical_frame
ros2 run tf2_ros tf2_echo laser base_link
```

> 帧名不会生成外参：改 URDF / 静态 TF 发布器才是改外参。
> `map_frame` 是理想地图，不能把 `tag_poses` 填成相对现实 odom 的位置。

---

## 3. `apriltag_localizer`：用 Tag 对齐现实 odom 到理想地图

### 3.1 数据流与对外接口

```text
D435 彩色帧 ──AprilTag + solvePnP──> T_C_tag
        │                              │
        ├─ TF: lidar->camera, lidar->base
        ├─ tag_poses: T_M_tag ────────┴──> T_M_B(Tag 解算的机体绝对位姿)
        │
雷达 /odom (nav_msgs/Odometry) ──按图像时间插值──> T_O_child，child_frame_id
        │                                          │
        └─ TF: child->base ────────────────────────┴──> 样本 T_M_O = T_M_B * inverse(T_O_B)
                                                          │
                                           多帧平均 + 离散度检查
                                                          │
                        ┌─────────────────────────────────┴──────────────┐
                        ▼                                                ▼
        TF: map -> odom（tf_update_hz）        /odom_corrected：T_M_O * T_O_child
```

| 类型 | 名称 | 消息 / 说明 |
| --- | --- | --- |
| 订阅 | `odom_topic`（默认 `/odom`） | `nav_msgs/Odometry`，`SensorDataQoS` |
| 发布 | `corrected_odom_topic`（默认 `/odom_corrected`） | `nav_msgs/Odometry`，QoS depth 50 |
| 发布 TF | `map_frame -> odom_frame` | `geometry_msgs/TransformStamped`，`tf_update_hz` 定时广播 |
| 发布 | `~/tag_pose_camera` | `PoseStamped`(frame=camera)，PnP 结果，供校验 |
| 发布 | `~/tag_pose_lidar` | `PoseStamped`(frame=laser)，校验相机外参 |
| 发布 | `~/lidar_pose_map` | `PoseStamped`(frame=map)，雷达在 map 中的位姿，不是机体位姿 |
| 服务 | `~/relocalize` | `std_srvs/Trigger`，清空样本、重新对齐 |

三个调试位姿由 `publish_debug_poses: true` 控制，纯显示用，不参与计算。
节点**不发布** `odom -> child`（由雷达负责）、`map -> child`、`/initialpose`，
也不订阅或发布 `/scan`、图像话题。相机由 `librealsense2` 直连 D435。

启动参数校验：`map_frame` / `odom_frame` / `base_frame` 必须非空且两两不同，
`odom_topic != corrected_odom_topic`，`tf_update_hz`、`odom_history_sec`、
`max_sample_gap`、`max_spread`、`max_rotation_spread_deg` 必须为正，
`tag_poses` 不能为空；不满足时节点直接报 `节点初始化失败` 退出。

### 3.2 数学推导

位姿链（`pose_math::relocalize()`）：

\[
\begin{aligned}
T_{C\_tag} &= \text{solvePnP}(rvec, tvec) && \text{① tag 在相机光学系}\\
T_{L\_tag} &= T_{L\_C}\,T_{C\_tag} && \text{② tag 在雷达系}\\
T_{M\_L}   &= T_{M\_tag}\,T_{L\_tag}^{-1} && \text{③ 雷达在 map 中}\\
T_{M\_B}   &= T_{M\_L}\,T_{L\_B} && \text{④ 机体在 map 中（Tag 解算的绝对位姿）}
\end{aligned}
\]

里程计配对与校正量（**逐帧先算修正量再平均**）：

\[
\begin{aligned}
T_{O\_B} &= T_{O\_child}\,T_{child\_B} && \text{同一图像时刻的 odom 位姿}\\
T_{M\_O} &= T_{M\_B}\,T_{O\_B}^{-1} && \text{本帧样本：完整 6DoF 校正变换}\\
\overline{T_{M\_O}} &= \text{mean}(\text{samples}) && \text{平移取均值，四元数统一半球后归一化平均}
\end{aligned}
\]

输出：

```text
TF            map -> odom  = T_M_O
/odom_corrected.pose       = T_M_O * T_O_child       (即 T_M_child)
map -> child               = T_M_O * T_O_child       (由 TF 树自动合成)
```

因为平均的是**校正变换**而不是机器人位置，机器人在采样期间运动不会把位移混进
修正量里；这正是 `continuous` 模式允许运动中采样的原因。校正保留完整 xyz 与旋转，
不做平面化，也不做单步限幅。

两道防呆：

- PnP 结果必须有限，`z ∈ (0, 10]` m，否则本帧丢弃。
- 解出的 \(T_{M\_B}\) 的 roll/pitch 超过 `max_tilt_deg` 时打印
  `解出的 map->base 姿态异常 ...` 并丢弃本帧——通常是 `tag_poses` 的贴墙 RPY
  或相机外参旋转填反，不要靠调大阈值掩盖。

### 3.3 时间同步与 odom 插值

图像时间戳（`captureLoop()`）：

- D435 使用系统/全局时间域时，直接把帧时间戳作为采样时刻，与 ROS 系统时间一致；
- 硬件时间域或 `use_sim_time` 时，退化为「图像接收时刻」，并周期性打印
  `相机使用硬件时间/仿真时钟，以图像接收时刻近似采样时间`；此时传输延迟会直接影响精度。

输入 odom（`onOdometry()`）先做合法性检查：`header.frame_id == odom_frame`、
`child_frame_id` 非空且不等于 `map_frame`/`odom_frame`、时间戳非 0、位姿有限、
四元数模长非零；无效帧忽略。缓存是带 `odom_mutex_` 的 `std::deque`，按
`odom_history_sec`（默认 5 s）裁剪；时间戳倒退或 `child_frame_id` 改变时打印
`odom 时间倒退或 child 改变，忽略本帧；雷达重启后请重启重定位节点`。

`odometryAt()` 用 `std::lower_bound` 在缓存中找图像时刻两侧样本：

- 平移线性插值（`lerp`），旋转球面插值（`slerp`）；
- 两侧时间差都必须 ≤ `max_sync_error`（默认 0.10 s），落在缓存边界时也要求
  「到最近样本的时间差」≤ 该阈值；
- 找不到配对时打印 `无图像时刻对应的 /odom 里程计，等待同步（检查话题、时间戳与帧名）`，
  本帧不产生样本。

> 输入 odom 与图像必须使用同一时钟；雷达里程计重置（坐标原点跳变）后应重启本节点重新对齐，
> 节点无法仅凭 odom 消息可靠判断原点是否被重置。

### 3.4 采样、离散度检查与触发模式

`pushSample()` 累积的是 \(T_{M\_O}\)。相邻有效样本间隔超过 `max_sample_gap`
（默认 0.5 s）会清空重来；累积到 `min_samples`（默认 10）帧后：

- 平移：三维向量取算术平均；
- 旋转：以第一个样本为参考把四元数统一到同一半球，累加后归一化；
- 有效性：每个样本的平移到均值的距离 ≤ `max_spread`（0.08 m，三维），
  与平均旋转的最短夹角 ≤ `max_rotation_spread_deg`（5°）；
- 任一不满足 → 打印 `校正样本位置/角度离散过大，丢弃本轮`，清空样本重来；
- 通过 → 写入 `current_M_O_`，日志打印
  `已对齐 map -> odom：平移=(x, y, z)，yaw=... deg`。

| `trigger_mode` | 行为 |
| --- | --- |
| `auto_once`（默认） | 首次对齐后固定修正，不再重新采样；Tag 离开视野后仍靠雷达增量运动 |
| `manual` | 启动后不采样，等 `~/relocalize` 触发一次 |
| `continuous` | 检测到 Tag 时不断重新对齐，可持续修正后续雷达漂移 |

重新对齐（机器人被搬动、标定改动、里程计重启后）：

```bash
ros2 service call /apriltag_localizer/relocalize std_srvs/srv/Trigger "{}"
```

服务清空样本、重置 `last_sample_ns` 并把 `need_publish_` 置回 true，
重新累积 `min_samples` 帧；重新采样期间继续使用上一次的修正。

### 3.5 输出细节

**TF `map -> odom`**：由 `tf_update_hz`（默认 30 Hz）定时器持续广播完整 6DoF 变换，
未对齐前不广播。只广播这一条，不重复发布雷达的 `odom -> child`，也不发 `map -> child`。
**不要让 AMCL 或其它节点同时发布同一条 `map -> odom`**，否则 TF 树冲突、rviz 中机器人乱跳。

**`/odom_corrected`**：每收到一条输入 odom 就发一条（未对齐前不发），
保持输入的**时间戳与 `child_frame_id`**，`header.frame_id` 改为 `map_frame`；

- `pose.pose = T_M_O * 输入 pose`，位姿左乘校正变换；
- `pose.covariance` 在父系表达，按 \(diag(R,R)\,C\,diag(R,R)^T\) 旋转；
- `twist` 在 child 系表达，**保持不变**。

固定修正能消除启动坐标偏差；后续雷达漂移需要再次看到 Tag（`continuous` 或
`~/relocalize`）才能修正。输出精度取决于 Tag 尺寸、地图标定、相机外参与时间同步。

### 3.6 参数表（`config/localizer.yaml`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_link` | 理想地图 / 雷达 odom / 机体帧 |
| `lidar_frame` / `camera_frame` | `laser` / `camera_color_optical_frame` | 雷达 / 相机光学帧，必须与实际 TF 一致 |
| `tag_size` | `0.118` | Tag 黑色方块外边长 [m]，**必须实测** |
| `tag_poses` | 见 YAML | `"id,x,y,z,roll,pitch,yaw"`，单位 m / 度，Tag 在 map 中的位姿表 |
| `target_tag_id` | `-1` | `-1` 使用任意已标定 Tag；否则只用指定 ID |
| `odom_topic` | `/odom` | 输入雷达里程计话题 |
| `corrected_odom_topic` | `/odom_corrected` | 校正后里程计输出话题 |
| `tf_update_hz` | `30.0` | `map -> odom` 广播频率 [Hz]，必须 > 0 |
| `odom_history_sec` | `5.0` | odom 缓存时长 [s]，必须 > 0 |
| `max_sync_error` | `0.10` | 图像与 odom 最大时间差 [s]；插值时两侧均须满足 |
| `trigger_mode` | `auto_once` | `auto_once` / `manual` / `continuous`，见 3.4 |
| `min_samples` | `10` | 累积多少帧有效样本取平均 |
| `max_spread` | `0.08` | 校正变换三维平移离散上限 [m] |
| `max_rotation_spread_deg` | `5.0` | 校正旋转离散上限 [度] |
| `max_sample_gap` | `0.5` | 样本间隔超过该值，清空重新累积 [s] |
| `tf_timeout` | `0.2` | 查静态外参的超时 [s] |
| `max_tilt_deg` | `30.0` | 解出的 `map->base` roll/pitch 上限，超了判定标定错误 |
| `publish_debug_poses` | `true` | 是否发布 3 个调试位姿话题 |
| `show_image` | `false` | OpenCV 预览（检测框、中心点、坐标轴、XYZ）；无桌面必须 false |
| `image_width` / `image_height` / `fps` | `640` / `480` / `30` | D435 彩色流 |
| `quad_decimate` / `quad_sigma` / `nthreads` / `refine_edges` | `1.0` / `0.0` / `4` / `true` | AprilTag 检测器参数 |
| `publish_camera_tf` | `false` | 是否由本节点代发相机静态外参 TF |
| `camera_tf_parent` | `base_link` | 代发 TF 的父帧 |
| `camera_in_parent` | `[0.20, 0, 0.15, -90, 0, -90]` | `[x,y,z,roll,pitch,yaw]`，m / 度 |

### 3.7 运行与验证

前置条件：雷达在发 `nav_msgs/Odometry`；`odom_frame` 与消息 `header.frame_id` 一致；
`child -> base_link`、`lidar -> camera`、`lidar -> base_link` 三条静态外参可用；
`tag_poses` 已按第 4 节标定。

```bash
ros2 launch apriltag_localization apriltag_localization.launch.py

# 端口 A：输入 odom 的帧名和时间戳
ros2 topic echo /odom --once | head -20

# 端口 B：校正输出（header.frame_id 应为 map）
ros2 topic echo /odom_corrected --once | head -20

# 端口 C：校正变换本身
ros2 run tf2_ros tf2_echo map odom

# 位姿链逐级核对
ros2 topic echo /apriltag_localizer/tag_pose_camera
ros2 topic echo /apriltag_localizer/tag_pose_lidar
ros2 topic echo /apriltag_localizer/lidar_pose_map

# 重新对齐
ros2 service call /apriltag_localizer/relocalize std_srvs/srv/Trigger "{}"
```

启动日志应打印 `Tag 重定位：map -> odom；输入 /odom，校正输出 /odom_corrected；...`
以及每条 `载入 tag <id> 的 map 位姿`。节点构造时会打开 D435，
**没有相机时会以 `D435 初始化失败: No device connected` 退出**。

### 3.8 排错

| 现象 | 优先核对 |
| --- | --- |
| `D435 初始化失败` | 没接 D435 / 被别的节点占用；`rs-enumerate-devices` 检查；两个节点不能共用一台相机 |
| `忽略无效 odom` | `header.frame_id` 是否等于 `odom_frame`、`child_frame_id` 是否有效、时间戳是否为 0、四元数是否归一化 |
| `无图像时刻对应的 /odom 里程计` | 话题名、`max_sync_error`、图像与 odom 是否同一时钟、odom 频率是否过低 |
| `odom 时间倒退或 child 改变` | 雷达重启导致原点/帧变化；重启雷达与重定位节点重新对齐 |
| `TF laser -> ... 不可用` | 帧名写错、静态外参没发布；`view_frames` 对照 |
| `解出的 map->base 姿态异常` | `tag_poses` 的贴墙 RPY 或相机外参旋转填反，见 4.2 / 4.3 |
| `校正样本位置/角度离散过大` | 采样时机器人/相机抖动、时间同步误差；静止后再 `~/relocalize` |
| 一直没有 `/odom_corrected` / `map -> odom` | 尚未完成首次对齐；检查是否检测到已标定 Tag、odom 是否配对成功 |
| 检测不到 Tag | `tag36h11` 家族、`tag_size`、光照、`target_tag_id` |
| 解算距离成固定比例偏差 | `tag_size` 填错 |
| rviz 里机器人在两个位置间跳 | 有别的节点同时在发 `map -> odom`（AMCL 的 `tf_broadcast` 等） |

---

## 4. 现场标定

**定位精度主要取决于这三项，不取决于算法**：`tag_size` → 相机外参 → `tag_poses`。

### 4.1 `tag_size`

量的是 **AprilTag 黑色方块图案的外边长**（不含白边、不含打印纸），单位米。
`tag36h11` 图案为 10×10 单元。两个 YAML 各有一份 `tag_size`，改动一个不会同步另一个。
填错的典型症状是 `~/tag_pose_camera` 的 z 与真实距离成固定比例偏差。

### 4.2 相机外参：两套配置互不相通

需要的是「相机光学系在机体系中的位姿」，包括平移和
`base_link`(x前/y左/z上) → 光学系(x右/y下/z前) 的固定轴系旋转
（水平前视时 `rpy = (-90°, 0, -90°)`，等价于 `T_B_C` 的列
`(0,-1,0)` / `(0,0,-1)` / `(1,0,0)`）。相机朝下或朝侧面时还要叠加安装角。

| 节点 | 外参来源 | 修改位置 |
| --- | --- | --- |
| `apriltag_localizer` | 真实 TF 树（`lookupTransform(lidar_frame, camera_frame)`） | URDF 的 fixed joint，或外部 `static_transform_publisher`，或本节点代发 |
| `tag_approach` | `config/approach.yaml` 的 4x4 矩阵 `T_B_C` | 直接改矩阵；**不读 TF** |

`apriltag_localizer` 三种提供方式（任选其一）：

1. **推荐**：URDF / `robot_state_publisher` 发布 `base_link -> camera_color_optical_frame`，
   保持 `publish_camera_tf: false`。
2. 外部静态 TF：
   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x 0.20 --y 0.0 --z 0.15 --roll -1.5708 --pitch 0 --yaw -1.5708 \
     --frame-id base_link --child-frame-id camera_color_optical_frame
   ```
3. 本节点代发：`publish_camera_tf: true`，填 `camera_tf_parent` 与
   `camera_in_parent: [0.20, 0.0, 0.15, -90.0, 0.0, -90.0]`。

同一个 child TF 只能有一个发布者，方式 1/2 与方式 3 不要同时用。父帧改成 `laser` 时，
六个数值必须改成光学系在雷达系中的位姿。URDF 的 `rpy` 是弧度，
`camera_in_parent` 是度。

`apriltag_localization.launch.py` / `real_deployment.launch.py` **都不启动**雷达驱动、
里程计、URDF 或底层运动节点，这些需另行启动。

### 4.3 `tag_poses`（Tag 在理想地图中的位姿）

格式（每行一个 ID，单位 m / 度，RPY 为内旋 ZYX）：

```yaml
tag_poses:
  - "5, 2.50, 1.20, 0.80, 90.0, 0.0, 0.0"
target_tag_id: 5
```

Tag 坐标系由 2.2 的角点约定决定：+x 沿 `p[1] -> p[0]`，+y 沿 `p[3] -> p[0]`，
+z 为码正面外法向。常见贴法：

| 贴法 | RPY | 说明 |
| --- | --- | --- |
| 平放地面、图案 +x 指向 map +X | `0, 0, 0` | +z 朝天，最不容易出错 |
| 平放地面、图案旋转 θ | `0, 0, θ` | θ 量图案边与 map X 轴的夹角 |
| 竖直贴墙（法向水平） | 含 `±90` 的 roll/pitch | 需现场确认 +z 指向场地内部 |

例：tag 贴在 `y = y0` 墙上、正面朝 map 的 −Y，图案 +x 指向 map +X、+y 朝上，
则 tag 三轴在 map 中为 `+x=+X`、`+y=+Z`、`+z=−Y`，对应绕 X 轴 +90°，
填 `"id, x, y0, z, 90, 0, 0"`。**先写出三轴再推 RPY，不要死记角度**。

最省事的反算法（强烈推荐）：把机器人摆到已知真实位姿，测出 Tag 在相机系中的完整位姿：

```text
T_M_tag = T_M_B_known * T_B_C * T_C_tag_measured
```

`T_C_tag_measured` 可从 `/apriltag_localizer/tag_pose_camera` 读取（姿态是四元数，
不能把 x/y/z 当成 RPY），`T_B_C` 来自外参或 TF。结果转换成位置和 RPY 度数写回
`tag_poses`，并在不同机器人朝向下重复验证。只给 XY 累加最终位置误差不能普遍修复旋转错误；
`~/lidar_pose_map` 是**雷达**位姿而非机体位姿，原点不重合时不能直接与机体真值比较。

`tag_poses` 可配多个 ID；节点一帧只用「第一个有效 Tag」产生一个样本，
跨帧可能切换 ID，各 Tag 的 map 标定必须一致。`target_tag_id` 可强制只用某一个；
检测到未标定的 Tag 会忽略并打印 WARNING。

### 4.4 时间同步与帧名

- `odom_frame` 必须等于输入 `Odometry.header.frame_id`，`map_frame` 是理想地图；
  两者、以及 `base_frame` 必须两两不同。
- 输入 odom 的 child 可以是雷达或机体，但必须有 child → `base_frame` 的静态外参。
- 图像与 odom 必须同一时钟；硬件时间域/仿真时钟下只能用接收时刻近似，
  运动时传输延迟会直接变成定位误差。
- `max_sync_error` 是配对成功的硬门槛，不是「允许的延迟补偿量」：
  超出就丢帧，不会外推。

---

## 5. `tag_approach`：纯视觉趋近

> 本节完全独立于第 3 节：不需要雷达、`/scan`、AMCL、`map` 帧或 TF 树。
> 所有常数位姿以 4x4 矩阵写在 `config/approach.yaml`。

### 5.1 链路与对外接口

```text
相机彩色图 -> AprilTag + solvePnP -> T_C_tag                  (tag 在相机系)
                     │
                     ├─ × T_B_C        (4x4 常数，相机安装外参)
                     ▼
                  T_B_tag                                   (tag 在当前机体系)
                     │
                     ├─ × T_tag_B_goal (4x4 常数，目标站位)
                     ▼
                  T_Bnow_Bgoal                              (目标机体在当前机体系)
                     │
                     └─ (dx, dy, yaw) -> P 控制 -> robot_move_cmd / cmd_vel -> 到位停车
```

\[
T_{B\_tag}=T_{B\_C}T_{C\_tag},\qquad
T_{B_{now}\_B_{goal}}=T_{B\_tag}T_{tag\_B\_goal}
\]

到位自检：机器人已在目标位姿时 \(T_{B\_tag}=T_{tag\_B\_goal}^{-1}\)，
于是 \(T_{B_{now}\_B_{goal}}=I\)，误差全为 0。从该变换取 `dx, dy, yaw` 做 P 控制：
`dx>0` 表示目标在前方，`dy>0` 表示在左侧。

速度命令由独立定时器以 `control_rate`（默认 50 Hz）发布，相机采集在独立线程，
两者通过带 mutex 的最新误差通信；掉帧或遮挡时定时器仍能可靠超时停车。

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| 发布 | `cmd_topic`（默认 `robot_move_cmd`） | `robot_msgs/Cmd` |
| 发布 | `twist_topic`（默认 `cmd_vel`） | `geometry_msgs/Twist`，`cmd_output: cmd_vel` 时 |
| 发布 | `stand_topic`（默认空） | `std_msgs/Bool`，非空时到位/停车请求位控站立（MuJoCo 用） |
| 发布 | `~/tag_pose_body` / `~/goal_pose_body` | 调试 `PoseStamped`，frame 固定 `base_link`，由 `publish_debug_poses` 控制 |
| 服务 | `~/start` / `~/stop` | 开始趋近 / 急停并交还控制权 |
| 服务 | `~/record_goal` | 示教目标站位，见 5.3 |
| 订阅 | `image_topic` + `camera_info_topic` | 仅 `image_source: ros` 时使用 |

当前趋近节点**没有** `~/tag_pose_camera` 话题，也不广播 Tag TF；
相机系六维位姿看日志 `T_C_tag`。

### 5.2 两个 4x4 矩阵（最容易写反）

| 参数 | 定义 | 含义 |
| --- | --- | --- |
| `T_B_C` | \(p_B = T_{B\_C}\,p_C\) | 相机光学系在机体系中的位姿（安装外参） |
| `T_tag_B_goal` | \(p_{tag} = T_{tag\_B\_goal}\,p_{B_{goal}}\) | 目标机体系在 Tag 系中的位姿（最终站位） |

行主序 16 个数，YAML 里可排成 4 行：

```yaml
T_B_C: [ 0.0,  0.0,  1.0,  0.20,     # [ r00 r01 r02 tx ]
        -1.0,  0.0,  0.0,  0.00,     # [ r10 r11 r12 ty ]
         0.0, -1.0,  0.0,  0.15,     # [ r20 r21 r22 tz ]
         0.0,  0.0,  0.0,  1.00]     # [  0   0   0   1 ]
```

左上 3×3 的三列分别是相机 X/Y/Z 轴在机体系中的方向，最后一列是相机原点在机体系中的坐标。
安装姿态不变、只改安装位置时，只改最后一列前三项。

节点启动时用 `pose_math::fromMatrix4x4()` 校验：恰好 16 个数、最后一行 `(0,0,0,1)`、
左上 3×3 正交（\(R^TR=I\)）、det=+1；否则拒绝启动。
**矩阵转置写反仍是合法旋转，校验抓不出来**，只会让机器人朝反方向走，
所以 `T_tag_B_goal` 一律用示教生成。

### 5.3 示教 `~/record_goal`

1. 校准 `T_B_C`、相机内参和 `tag_size`，保持 `auto_start: false`。
2. 把机器人摆到最终希望停止的位置与朝向，让相机稳定看见 Tag。
3. 调用：
   ```bash
   ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger "{}"
   ```
4. 服务先确认「已请求」，下一帧有效检测后日志打印实际 4x4 矩阵
   （节点算 \(T_{tag\_B\_goal} = (T_{B\_C}T_{C\_tag})^{-1}\)），并立即在本次运行使用新目标。
5. 把 16 个数抄回 `config/approach.yaml` 的 `T_tag_B_goal`，**重启后才会保留**；
   服务不会自动写文件。

该矩阵的平移是**Tag 系**下的目标机体原点位置，不是相机到 Tag 的距离；
改最后一列是沿 Tag 坐标轴移动目标，改左上 3×3 是改变目标朝向。
相机重新安装、换码/旋转码后应重新核对外参并示教。
Tag 在场地中移动后，纯视觉目标会随 Tag 移动，它不是固定 map 点。

### 5.4 图像源与内参

| `image_source` | 行为 | 内参来源 |
| --- | --- | --- |
| `auto`（默认） | 先探 D435，没有就回退普通 UVC | 按实际选中的源 |
| `realsense` | 强制 `librealsense2` 读 D435 彩色流 | SDK `get_intrinsics()`，无需标定 |
| `uvc` | OpenCV `VideoCapture` 读 `uvc_device` | `camera_fx/fy/cx/cy` + `distortion`，必须自己标定 |
| `ros` | 订阅 `image_topic` + `camera_info_topic` | `CameraInfo` 消息（MuJoCo 仿真用） |

`uvc_device` 可写 `/dev/video2` 或数字索引；用 `v4l2-ctl --list-devices` 查。
UVC 未标定（`camera_fx/fy=0`）时按 `hfov_deg`（默认 60°）估算
\(f_x = (w/2)/\tan(\text{hfov}/2)\)，节点打印 WARN：**包括 z 在内的位姿都可能有系统误差**，
表现为「离 Tag 越远误差越大」。要求精度时先棋盘格标定：

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=<图像话题>
```

把 \(f_x,f_y,c_x,c_y,k_1,k_2,p_1,p_2,k_3\) 填入 `config/approach.yaml`，
并确认内参与**实际采集分辨率**一致。

画面太暗（UVC 常见）时调 `auto_exposure` / `exposure` / `gain`：
本机实测 `exposure: 330`（33 ms，100 µs 单位）+ `gain: 64` 可用且仍保持 30 fps；
曝光超过 `1/fps` 会掉帧。RealSense 走 `rs2_option`，同名参数量纲不同（彩色 gain 16~248），
不要照抄 UVC 数值。日光灯横条纹需手工设
`v4l2-ctl -d /dev/video2 --set-ctrl=power_line_frequency=1`。

### 5.5 控制律与安全

| 机制 | 说明 |
| --- | --- |
| `holonomic`（默认） | `vx = kp_x·dx`、`vy = kp_y·dy`、`wz = kp_yaw·yaw` |
| `differential` | 先转向对准目标点再前进，不发横移；最终原地对齐朝向的逻辑有限，需另行验证 |
| `yaw_align_band` | 朝向偏差超过该值 [rad] 时只转向、不前进 |
| 限速 / 死区 / 平滑 | `max_vx/vy/wz`、`min_velocity`、`smooth_tau`（一阶低通时间常数 [s]） |
| 到位 | `pos_tolerance`（XY 距离 [m]）、`yaw_tolerance`（rad）连续保持 `hold_time` 秒；不控制高度 |
| 丢码 | 超过 `lost_timeout` 发零速；`resume_on_reacquire` 控制是否自动恢复，否则需重新 `~/start` |
| 待命 | `auto_start: false` 时完全不发速度，控制权留给遥控器 |
| 到位/停止 | 持续发零速站立帧，直到 `~/stop` 交还控制权 |
| 到位后继续跟踪 | `track_after_arrival` + `tracking_resume_distance` / `tracking_resume_yaw_deg`，超阈值自动恢复行走 |
| 倾角防呆 | 解出的目标相对当前机体 roll/pitch > `max_tilt_deg` → 判定矩阵填反，本帧忽略 |

控制频率对齐（`rl_real_atdog2` 实例）：`loop_control` 200 Hz 读 `remote_cmd`，
`loop_rl` 按 `dt × decimation = 20 ms`（50 Hz）才把速度送进策略观测，
所以 `control_rate: 50.0`。原 20 Hz 与 50 Hz 不成整数倍会产生相位抖动。
时间制参数 `smooth_tau`、`hold_time`、`lost_timeout` 都按秒换算，
改 `control_rate` 不会悄悄改变控制行为。

> **安全**：`rl_real_atdog2` 对 `robot_move_cmd` 没有超时保护，若本节点被 `kill -9`
> 或异常崩溃，机器人会保持最后一条速度命令一直走。正常 Ctrl-C 退出会补发零速站立帧；
> 真机测试请随时准备 `~/stop` 或遥控器急停。
> 测试期间不要同时运行 `obstacle_game` / `remote_node`，它们也发 `robot_move_cmd`。

### 5.6 速度输出接口

| `cmd_output` | 消息 / 话题 | 适用 |
| --- | --- | --- |
| `robot_move_cmd`（默认） | `robot_msgs/Cmd` → `cmd_topic` | 本工作区 `rl_sar` 的 `rl_real_atdog2/3` |
| `cmd_vel` | `geometry_msgs/Twist` → `twist_topic` | 通用底盘 / 仿真 |
| `none` | 不发布，只打日志 | **首次干跑推荐** |

关键约定：

- `robot_msgs/Cmd.vz` 是**偏航角速度**，不是 z 向线速度；节点把 `wz` 填进去，
  Twist 模式则填 `angular.z`。
- `cmd_mode: 2` → RL 行走；`stand_mode: 1` → 位控站立（到位/停车）。
  `mode = 0` 时底层不调用 `setVel`、`control.x/y/yaw` 保持上次值，本节点不会发 0。
  不希望有姿态过渡可把 `stand_mode` 也设为 `2`，靠零速让策略原地站稳。

### 5.7 参数表（`config/approach.yaml`，节选）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `T_B_C` / `T_tag_B_goal` | 见 YAML | 两个 4x4 行主序矩阵，启动时校验正交性与 det=+1 |
| `image_source` / `uvc_device` | `auto` / `/dev/video2` | 图像源与 UVC 设备 |
| `image_topic` / `camera_info_topic` | `/mujoco/camera/image_raw` 等 | 仅 `image_source: ros` |
| `camera_fx/fy/cx/cy`、`distortion`、`hfov_deg` | `0`（未标定）/ `60.0` | UVC 内参 |
| `auto_exposure`、`exposure`、`gain` 等 | 见 YAML | 画面参数，`-1` 表示不动驱动默认值 |
| `tag_id` / `tag_size` | `5` / `0.118` | 目标 ID（-1 接受任意）、黑框外边长 [m] |
| `cmd_output` / `cmd_topic` / `twist_topic` | `robot_move_cmd` / `robot_move_cmd` / `cmd_vel` | 速度输出 |
| `cmd_mode` / `stand_mode` | `2` / `1` | 行走 / 站立模式 |
| `control_rate` | `50.0` | 控制定时器频率 [Hz] |
| `control_mode` | `holonomic` | `holonomic` / `differential` |
| `kp_x/y/yaw`、`max_vx/vy/wz`、`allow_lateral`、`yaw_align_band` | 见 YAML | 增益、限速、横移开关 |
| `smooth_tau` / `min_velocity` / `minimum_tracking_speed` | `0.10` / `0.02` / `0.0` | 平滑、死区、跟踪最小速度补偿 |
| `pos_tolerance` / `yaw_tolerance` / `hold_time` | `0.05` / `0.05` / `0.30` | 到位判定 |
| `lost_timeout` / `resume_on_reacquire` | `0.5` / `true` | 丢码停车与恢复 |
| `auto_start` / `max_tilt_deg` / `publish_debug_poses` | `false` / `30.0` / `true` | 安全与调试 |
| `latch_goal` / `track_after_arrival` / `tracking_resume_distance` / `tracking_resume_yaw_deg` | `false` / `false` / `0.08` / `8.0` | 到位后继续跟踪（仿真默认开启） |
| `stand_topic` | 空 | 非空时发布 `std_msgs/Bool` 请求位控站立 |

### 5.8 运行步骤

```bash
# ① 编译
colcon build --symlink-install --packages-select apriltag_localization
source install/setup.bash

# ② 第一次务必干跑：approach.yaml 里 cmd_output: "none"
ros2 launch apriltag_localization tag_approach.launch.py
# 无桌面时追加 show_image:=false

# ③ 摆好机器人 -> 示教 -> 把打印的 4x4 矩阵抄回 yaml -> 重启
ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger "{}"

# ④ 改回 cmd_output，挪开一点，开始趋近
ros2 service call /tag_approach/start std_srvs/srv/Trigger "{}"
ros2 topic echo /tag_approach/goal_pose_body
ros2 topic hz /robot_move_cmd          # 应约 50 Hz

# ⑤ 任何不对劲立即急停
ros2 service call /tag_approach/stop std_srvs/srv/Trigger "{}"
```

启动日志会打印选中的图像源、分辨率、内参与 `tag_size`，先核对再往下走。
日志每秒打印一次 `T_C_tag | x=... y=... z=... | roll=... pitch=... yaw=...`，
这是**未经 `T_B_C` 的 PnP 原始结果**；`机体系误差 dx/dy/dz/yaw` 才是控制用量。

常见排错：

| 现象 | 优先核对 |
| --- | --- |
| 启动后不打印 `T_C_tag` | `tag_id` 与视野里的码不符（看预览窗口的 `ID:`）；临时设 `tag_id: -1` |
| 有框但无位姿 | `solvePnP` 失败、深度不在 `(0,10]`、内参/分辨率不匹配 |
| 预览画面全黑 | UVC 曝光/增益，见 5.4 |
| `dx` 与实际距离差一个比例 | 用了估算内参或 `tag_size` 填错 |
| 参数矩阵无效 | 16 个数、最后一行、正交性、det=+1 |
| 越走越远 / 原地打转 | 矩阵转置写反或目标朝向写反；看 `~/goal_pose_body` 的 dx 符号，重新示教 |
| 发了命令机器人不动 | 未 `~/start`、`cmd_output: none`、底层未启动、`cmd_mode` 为 0 |
| 机器人被别的节点抢着动 | 关掉同时运行的 `obstacle_game` / `remote_node` |

---

## 6. 实机部署与 MuJoCo 验收

### 6.1 `real_deployment.launch.py`

实机入口同时启动 `rl_sar` 的 `rl_real_<robot>`（`atdog2` 默认 / `atdog3` 可选）
和 `tag_approach`，默认打开 OpenCV 预览窗口，固定 `cmd_output: robot_move_cmd`、
`cmd_topic: /robot_move_cmd`；`auto_start` 默认 `false`（可用启动参数改）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch apriltag_localization real_deployment.launch.py
# 可选：robot:=atdog3  image_source:=uvc  auto_start:=true
#      params_file:=/abs/path/to/approach.yaml
```

任一侧进程退出时 launch 自动关停另一侧。启动后默认待命，通过
`/tag_approach/start` / `/tag_approach/stop` 开始或停车。需要本机桌面或 X11 转发。

### 6.2 MuJoCo 场景 `scene_tag`

`scene_tag.xml` 是平地测试场景：世界坐标 `(2, 0, 0.5)` 处放 tag36h11 / ID 5，
黑框边长 **0.24 m**；机身前视相机在 `(0.1948, 0, 0.1465)`，图像 `640×480`、
垂直视场角 60°。识别器对 ROS 图像和精确内参运行真实 AprilTag + PnP，不用标签真值。

```bash
source /opt/ros/humble/setup.bash
cmake -S src/rl_sar -B cmake_build \
  -DUSE_CMAKE=ON -DUSE_MUJOCO=ON -DMUJOCO_ROS2=ON
cmake --build cmake_build --target rl_sim_mujoco -j2
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-select apriltag_localization
source install/setup.bash
ros2 launch apriltag_localization tag_approach.launch.py use_mujoco:=true
# 另一型号：追加 robot:=atdog3；无预览：追加 show_image:=false
```

启动后默认待命不发速度。点击 MuJoCo 窗口取得焦点，`Enter` 切换暂停/继续，
按 `0` 起身站稳，按 `1` 自动调用视觉跟踪启动并切入 RL 策略行走；
到位后站住但持续识别，标签移动或机体偏差超过 **8 cm / 8°** 时自动恢复行走。
按 `0` 停止自动跟踪并保持站立，再次按 `1` 恢复。ROS launch 不转发标准输入，
所以按键已接入 MuJoCo 窗口；`N` 与 start/stop 服务仍保留给手动调试。

`config/approach_mujoco.yaml` 在 `approach.yaml` 之后加载，覆盖图像源为 `ros`、
`tag_size: 0.24`、目标矩阵与输出话题；目标为机体原点距标签平面 **0.6 m** 正对标签。

| 仿真话题 | 用途 |
| --- | --- |
| `/mujoco/camera/image_raw` / `/mujoco/camera/camera_info` | 仿真 RGB 画面与无畸变内参 |
| `/mujoco/tag_pose` | 世界系标签位姿，供移动标签实验 |
| `/mujoco/stand` | 到位、丢码、停止时请求位控站立 |
| `/mujoco/base_pose` | 世界系机体真值，**仅用于验收**，识别与控制不读取 |
| `/mujoco/cmd_vel` | 视觉趋近输出，按 `1` 后被策略采用 |
| `/tag_approach/goal_pose_body` | 机体系目标位姿，核对距离方向 |

### 6.3 自动闭环验收

在有 X11 桌面的终端运行（依赖 `libXtst`、`xwininfo`、Python OpenCV / NumPy）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/apriltag/validate_mujoco_approach.py --output /tmp/tag_validation/normal
python3 scripts/apriltag/validate_mujoco_approach.py --interrupt --output /tmp/tag_validation/interrupted
python3 scripts/apriltag/validate_mujoco_approach.py --move-tag --output /tmp/tag_validation/following
```

脚本启动真实 MuJoCo 查看器与策略，仅按 `0` / `1`，校验识别、前进、
最终平面误差 < 7 cm、偏航误差 < 5°、站立保持 10 秒漂移 < 1.5 cm 以及按 `0` 后保持。
`--move-tag` 在到位后移动标签 0.3 m，验证无需再次按键即可恢复行走；
`--interrupt` 在运动中暂停识别进程 3 秒，验证通信看门狗停车后恢复。
每轮保存进程日志、初始/最终相机图像、轨迹与验收 JSON，断言失败返回非零。
测试使用独立 `ROS_DOMAIN_ID=89`，不要同时运行两轮。软件 OpenGL 环境可加
`LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa`。

---

## 7. 常见问题 FAQ

**Q：没有 D435，能验证 odom 校正链路吗？**
A：`apriltag_localizer` 构造时就打开 D435，没设备会以 `D435 初始化失败` 退出。
数学部分可以按 3.2 的公式单独验算；`tag_approach` 则可用 UVC 或 `ros` 图像源
（MuJoCo）跑完整视觉链路，但它不做 odom 校正。

**Q：Tag 必须平放在地上吗？**
A：不必，贴墙/贴柱子都行，只要 `tag_poses` 的 RPY 如实描述 Tag 坐标系且 +z 朝向
能看到码的一侧。当前校正保留完整 6DoF，不像 2D AMCL 那样只用 x/y/yaw，
但 Tag 安装姿态仍会通过 \(T_{M\_tag}\) 影响解算，必须填对。

**Q：精度能到多少？**
A：PnP 单帧在 1~2 m、10 cm 量级 Tag 时，平移约 1~3 cm、朝向约 1~3°；
外参、Tag 场地坐标与时间同步误差通常是主要误差源（可达 5~10 cm）。
`min_samples` 多帧平均 + 离散度剔除用于抑制抖动。

**Q：为什么用 `tf2::TimePointZero` 查外参？**
A：相机与雷达外参、`base_link -> laser` 都是静态 TF，取最新值即可，不涉及时间插值。
若相机装在会动的云台上，需改成按图像采集时刻查询，并配 message filter /
带时间戳的 `lookupTransform`。

**Q：`lidar_frame` 和 `base_frame` 填成同一个可以吗？**
A：可以，`lookupTf()` 检测到相同直接返回单位变换；但只有当雷达原点确实与
`base_link` 原点重合时才这么做。

**Q：为什么节点用了独立采集线程？**
A：`rs2::pipeline::wait_for_frames()` 阻塞，放进 executor 回调会卡住服务与定时器。
节点在 `start()` 里起 `std::thread` 跑 `captureLoop()`，主线程 `rclcpp::spin()`
处理 odom 订阅、`~/relocalize` 服务和 TF 定时器；样本与 odom 缓存分别用 mutex 保护。

**Q：相机外参改了 URDF，为什么 `tag_approach` 没变化？**
A：`tag_approach` 不读 TF，只认 `config/approach.yaml` 的 `T_B_C` 矩阵；
两套配置必须分别维护。

---

## 8. 代码导览

| 顺序 | 位置 | 看点 |
| --- | --- | --- |
| 1 | `pose_math.hpp` 顶部注释 | 记号约定与位姿链 |
| 2 | `pose_math.hpp::fromSolvePnP()` | Rodrigues → `tf2::Matrix3x3` → `tf2::Quaternion` |
| 3 | `pose_math.hpp::relocalize()` | 三行核心公式 ②③④ |
| 4 | `apriltag_localizer.cpp::setupCamera()` | D435 内参、角点物点约定 |
| 5 | `apriltag_localizer.cpp::processDetection()` | PnP → 查 TF → 配对 odom → 生成 \(T_{M\_O}\) 样本 |
| 6 | `apriltag_localizer.cpp::onOdometry()` | odom 合法性检查、历史缓存、协方差旋转、发布 `/odom_corrected` |
| 7 | `apriltag_localizer.cpp::odometryAt()` | 按图像时刻 lerp / slerp 插值、`max_sync_error` |
| 8 | `apriltag_localizer.cpp::pushSample()` | 校正变换平均、半球统一、离散度检查、触发模式 |
| 9 | `apriltag_localizer.cpp::publishOdomCorrection()` | 定时广播 `map -> odom` |
| 10 | `pose_math.hpp::fromMatrix4x4()` | 4x4 矩阵解析 + 正交性/det 校验 |
| 11 | `tag_approach.cpp::captureLoop()` | 多图像源、三段位姿链、示教、倾角防呆 |
| 12 | `tag_approach.cpp` 的控制定时器 | P 控制、限速平滑、到位判定、丢码急停 |

---

## 9. 历史方案与迁移说明

早期版本是「AprilTag 给 AMCL 发 `/initialpose`，AMCL 用 `/scan` 与地图持续精修」，
对应 `config/amcl.yaml` 与 `launch/full_localization.launch.py`。
当前实现已改为「Tag 直接校正雷达 odom，输出 `map -> odom` 与 `/odom_corrected`」，
因此：

| 已移除 / 不再生效 | 说明 |
| --- | --- |
| `/initialpose` 发布、`nav2_amcl`、`nav2_map_server`、`nav2_lifecycle_manager` | 当前链路不依赖 AMCL 或地图服务器 |
| `launch/full_localization.launch.py` | 已删除；用 `apriltag_localization.launch.py` |
| `output_mode` | 不再区分 `initialpose` / `tf`，当前固定输出 TF + `/odom_corrected` |
| `require_subscriber`、`sigma_x/y/yaw` | 初值协方差与订阅等待逻辑已废弃 |
| `tf_max_step_trans` / `tf_max_step_rot` | 不再做平面化与单步限幅，校正保留完整 6DoF |
| `pose_math::summarize()` 的 2D 样本统计 | 当前 `apriltag_localizer` 不再调用；改为校正变换的三维平移 + 四元数平均 |

`config/amcl.yaml` 仅作为历史参考保留，当前任何 launch 都不会加载它；
若仍要跑 AMCL 方案，需自行恢复对应 launch，并确保**只有一个节点发布 `map -> odom`**。

### 附：从原 `tag.cpp` 改了什么

| 原 demo | 本功能包 |
| --- | --- |
| `main()` 里死循环打印 `X/Y/Z` | ROS 2 节点，独立采集线程 + `rclcpp::spin` |
| 位姿只到 `tvec` | 补 `rvec` → 完整 `T_C_tag` |
| 无 TF 概念 | 查相机/雷达/本体静态外参，得到 `T_L_tag` |
| 无地图概念 | 引入人工标定的 `T_M_tag`，反算 `T_M_L / T_M_B` |
| 无定位输出 | 配对 odom 求 \(T_{M\_O}\)，广播 `map -> odom` 并发布 `/odom_corrected` |
| 只能用 `cv::imshow` 目视 | 调试 `PoseStamped` 话题 + 可关闭的图像预览 |
| 单一 RealSense 取图 | `tag_approach` 另支持 UVC / ROS 图像与 MuJoCo 仿真闭环 |
