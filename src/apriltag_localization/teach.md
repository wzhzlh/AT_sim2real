# AprilTag 视觉重定位教学文档

> 面向 `AT_sim2real` 工作区（ROS2 Humble）的功能包 `apriltag_localization`。
> 目标：把 `~/qq_down/D435_apriltag/tag.cpp` 里那个「打印 tag 在相机系下 XYZ」的 C++ demo，
> 改造成能给激光定位做**全局初值**的 ROS2 节点。

---

## 0. 这个包解决什么问题

激光定位（AMCL）有个经典问题：机器人开机时不知道自己在地图的哪里，粒子滤波要靠运动 + 观测慢慢收敛；
场地大、地形对称时容易收敛到错误的局部极值，甚至完全不收敛。dof_pos

AprilTag 是一个**绝对位置基准**：tag 贴在场地已知坐标处，相机看到它就能算出「我相对这个基准在哪里」，
一步得到全局位姿。但它精度有限（厘米~十几厘米、几度），且视野受限、会被遮挡，不能一直用。

所以工程上的标准做法是两段式：

```
D435 相机 ──AprilTag 检测──> T_C_tag (tag 在相机系的位姿)
        │
        ├─ TF: camera→lidar ─> T_L_tag (tag 在雷达系的位姿)
        │
        ├─ + tag 在 map 中的标定值 T_M_tag
        │        └─> 反算 T_M_L / T_M_B (雷达、本体在 map 中的位姿)
        │
        └─> 发布 /initialpose (geometry_msgs/PoseWithCovarianceStamped)
                    │
                    ▼
        nav2_amcl：激光 scan 与 map 做粒子滤波匹配
                    │
                    ▼
        持续发布 map -> odom，修正 map->base_link（等价于修正雷达的 map 位姿）
```

一句话：**AprilTag 负责「一次性把机器人放到正确的邻域」，AMCL 负责「之后靠激光持续精修」。**

### 文件结构

| 文件 | 作用 |
| --- | --- |
| `include/apriltag_localization/pose_math.hpp` | 纯数学：solvePnP 结果转 `tf2::Transform`、整条位姿链、协方差、样本统计、tag 标定表解析 |
| `src/apriltag_localizer.cpp` | ROS2 节点：D435 取图、AprilTag 检测、查 TF、发 `/initialpose`、重定位服务 |
| `config/localizer.yaml` | 节点参数（帧名、tag 尺寸、**tag 场地位姿**、外参、发布策略、协方差） |
| `config/amcl.yaml` | `nav2_amcl` 示例参数（激光 scan 与地图匹配） |
| `launch/apriltag_localization.launch.py` | 只启动视觉节点（导航栈已由别人启动） |
| `launch/full_localization.launch.py` | 一键启动 `map_server + amcl + lifecycle_manager + 视觉节点` |
| `package.xml` / `CMakeLists.txt` | ROS2 ament_cmake 包定义 |

---

## 1. 依赖安装

> 当前开发机已装齐并验证：`ROS2 Humble`、`OpenCV 4.5.4`、`librealsense2 2.59.1`、
> `AprilTag 3.2.0`（免 sudo 用户级安装在 `~/.local`）。`colcon build` 零警告零错误，
> 已生成 `apriltag_localizer` 可执行文件。实机按下面各项安装即可。

### 1.1 系统库

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

# 检查相机
rs-enumerate-devices
```

```bash
# AprilTag C 库（Ubuntu 22.04 universe 自带）
sudo apt install libapriltag-dev
```

没有 sudo 权限时，可以免 root 安装到用户目录（本开发机采用的方式）：

```bash
cd /tmp
apt-get download libapriltag-dev libapriltag3        # 下载不需要 sudo
rm -rf /tmp/apriltag-extract && mkdir -p /tmp/apriltag-extract \
    ~/.local/include ~/.local/lib ~/.local/lib/pkgconfig
for f in libapriltag*.deb; do dpkg-deb -x "$f" /tmp/apriltag-extract; done
cp -a /tmp/apriltag-extract/usr/include/apriltag ~/.local/include/
cp -a /tmp/apriltag-extract/usr/lib/x86_64-linux-gnu/libapriltag* ~/.local/lib/
cp -a /tmp/apriltag-extract/usr/lib/x86_64-linux-gnu/pkgconfig/apriltag.pc \
    ~/.local/lib/pkgconfig/
```

也可以从源码装到 `/usr/local`：

```bash
git clone https://github.com/AprilRobotics/apriltag.git /tmp/apriltag
cmake -S /tmp/apriltag -B /tmp/apriltag/build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build /tmp/apriltag/build --target install && sudo ldconfig
```

> **CMake 怎么找 AprilTag**：用 `find_path(apriltag/apriltag.h)` +
> `find_library(apriltag)`，搜索 `/usr`、`/usr/local`、`~/.local`，
> 不依赖 `pkg-config`（Ubuntu 的 `apriltag.pc` 不含 `Cflags`，用户级安装时拿不到头文件路径）。
> 库位于用户目录时会写入 `RUNPATH`，所以 `ros2 run` 无需额外设置 `LD_LIBRARY_PATH`。

### 1.2 导航栈（负责「激光扫描与地图匹配」）

```bash
sudo apt install \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-tf2-tools
```

### 1.3 检查清单

```bash
pkg-config --modversion opencv4       # 4.5.x
pkg-config --modversion realsense2    # 2.5x.x
# AprilTag 头与库（三选一即可，CMake 会自动搜索）
ls /usr/include/apriltag/apriltag.h /usr/lib/x86_64-linux-gnu/libapriltag.so 2>/dev/null
ls ~/.local/include/apriltag/apriltag.h ~/.local/lib/libapriltag.so 2>/dev/null
ls /usr/local/include/apriltag/apriltag.h /usr/local/lib/libapriltag.so 2>/dev/null
ros2 pkg list | grep -E 'nav2_amcl|nav2_map_server'
```

OpenCV / RealSense / AprilTag 三者都能找到时，`colcon build` 会正常编译出
`apriltag_localizer`；可用 `ros2 pkg executables apriltag_localization` 确认。

---

## 2. 坐标系与 TF 树

| 帧 | 记号 | 说明 |
| --- | --- | --- |
| `map` | M | 场地/地图全局系，Nav2 地图的 `frame_id` |
| `odom` | O | 里程计系，由轮速/IMU/腿部里程计给出 `odom -> base_link` |
| `base_link` | B | 机器人本体，`/initialpose` 描述的就是它在 `map` 中的位姿 |
| `laser` | L | 雷达系，`/scan` 的 `frame_id`，与 `base_link` 之间是**固定** TF |
| `camera_color_optical_frame` | C | D435 彩色光学系：**x 右、y 下、z 前** |
| `tag` | tag | 码自身：原点在码中心，+z 为码正面法向（见 4.3） |

```
map
 └─(AMCL 发布)─> odom
                  └─(里程计/URDF)─> base_link
                                     ├─(静态 TF)─> laser
                                     └─(静态 TF)─> camera_color_optical_frame
```

三个必须成立的 TF 查询（节点内部用 `lookupTransform`）：

1. `lookupTransform(lidar_frame, camera_frame)` → `T_L_C`（相机外参）
2. `lookupTransform(lidar_frame, base_frame)` → `T_L_B`（雷达外参）
3. 若 `lidar_frame == base_frame`，节点直接给单位变换，无需真实 TF

> **重点**：`/initialpose` 是给 **base_link** 的，不是给雷达的。但雷达与 base_link 是刚性连接，
> 所以 AMCL 修正 `map->base_link` 的同时，`map->laser` 也就被修正了 —— 这就是「继续修正雷达位姿」的含义。

检查 TF 树：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo laser camera_color_optical_frame
ros2 run tf2_ros tf2_echo laser base_link
```

---

## 3. 数学推导（核心）

### 3.1 记号

`T_A_B` 表示「把 B 坐标系下的坐标变换到 A 坐标系」：

\[
p_A = T_{A\_B}\; p_B
\]

两条规则贯穿全文：

\[
T_{A\_C} = T_{A\_B}\,T_{B\_C} \qquad
T_{B\_A} = T_{A\_B}^{-1}
\]

`tf2_ros::Buffer::lookupTransform(target, source)` 返回的正是 `T_target_source`，代码里可以直接用 `tf2::Transform` 相乘。

### 3.2 七步链路

| 步 | 公式 | 来源 |
| --- | --- | --- |
| ① | \(T_{C\_tag}\) | `cv::solvePnP` 的 `rvec/tvec`，tag 相对相机的位姿 |
| ② | \(T_{L\_C}\) | TF：`camera → lidar` 外参 |
| ③ | \(T_{L\_tag} = T_{L\_C}\,T_{C\_tag}\) | **雷达坐标系下的目标位姿** |
| ④ | \(T_{M\_tag}\) | 人工标定：tag 在场地 map 中的位姿 |
| ⑤ | \(T_{M\_L} = T_{M\_tag}\,T_{L\_tag}^{-1}\) | **雷达在 map 中的位姿（重定位结果）** |
| ⑥ | \(T_{L\_B}\) | TF：`base_link → lidar` 外参 |
| ⑦ | \(T_{M\_B} = T_{M\_L}\,T_{L\_B}\) | AMCL 需要的 base 在 map 中的位姿 |
| ⑧ | \((x, y, \theta) = (T_{M\_B}.t_x,\; T_{M\_B}.t_y,\; \mathrm{yaw}(T_{M\_B}))\) | 发布到 `/initialpose` |

**为什么 ⑤ 是右乘逆？** 因为 tag 的位姿可以从两条路径表达：

\[
T_{M\_tag} = T_{M\_L}\,T_{L\_tag}
\quad\Longrightarrow\quad
T_{M\_L} = T_{M\_tag}\,T_{L\_tag}^{-1}
\]

直观理解：相机告诉我「tag 在我（雷达）前方 1.5 m」，而地图告诉我「tag 在 x=2.5 处」，
那我一定在 `x = 2.5 − 1.5 = 1.0` 处。求逆就是在做这个减法（带旋转的减法）。

**为什么 ⑦ 还要乘 `T_L_B`？** 因为 `/initialpose` 的语义是 base_link 的位姿，而 ⑤ 得到的是雷达的。
若雷达装在机身前方 0.1 m，不补这一步就会让整个定位偏 0.1 m 且朝向相关。

**为什么 ⑧ 丢掉 z / roll / pitch？** AMCL 是 2D 粒子滤波，只维护 \((x, y, \theta)\)。
`PoseWithCovarianceStamped` 里 z/roll/pitch 的方差填 `1e6`（见 `pose_math::makeCovariance`），
表示「这几维不提供信息」。机器人本体本来就水平，所以 `T_M_B` 的 roll/pitch 理论上接近 0；
如果算出来明显不为 0，说明 tag 的 RPY 标定或外参填错了。

### 3.3 一个可以手算（也能跑代码验算）的例子

假设（全部单位 m / rad）：

- 机器人真实位姿：`base_link` 在 map 的 `(1.0, 1.2, 0)`，yaw = 0
- `laser` 与 `base_link` 重合：`T_L_B = I`
- 相机装在机身前 0.20 m、高 0.15 m，水平前视
- tag 平放在地面 `(2.50, 1.20, 0)`，图案 +x 指向 map +X（yaw = 0）

`base_link`（x 前 / y 左 / z 上）→ 相机光学系（x 右 / y 下 / z 前）的固定旋转是
`rpy = (−90°, 0, −90°)`，对应

\[
R_{L\_C} = \begin{bmatrix}0&0&1\\-1&0&0\\0&-1&0\end{bmatrix},\quad
t_{L\_C} = (0.2,\,0,\,0.15)
\]

于是可以正向算出相机应该看到的位姿：

\[
T_{C\_tag} = (T_{M\_B}\,T_{L\_C})^{-1}\,T_{M\_tag}
\;\Rightarrow\; t = (0,\;0.15,\;1.30)
\]

即 tag 在相机「正前方 1.30 m、下方 0.15 m」——和几何直觉完全一致。
把它代回节点里的链路：

\[
T_{L\_tag} = T_{L\_C}T_{C\_tag} \Rightarrow t=(1.5,0,0)
\]
\[
T_{M\_L} = T_{M\_tag}T_{L\_tag}^{-1} \Rightarrow t=(1.0,1.2,0)
\]
\[
T_{M\_B} = T_{M\_L}T_{L\_B} \Rightarrow (1.0,\;1.2,\;0)
\]

**精确还原真实位姿。**

可以拷到 Python 里自己验算（不依赖 ROS，纯 numpy）：

```python
import numpy as np

def make(R, t):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    M = np.eye(4); M[:3, :3] = R.T; M[:3, 3] = -R.T @ t; return M

R_L_C = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
T_L_C = make(R_L_C, [0.2, 0.0, 0.15])          # ②
T_M_tag = make(np.eye(3), [2.5, 1.2, 0.0])      # ④
T_M_B_true = make(np.eye(3), [1.0, 1.2, 0.0])   # 只用来"伪造"一次观测

T_C_tag = inv(T_M_B_true @ T_L_C) @ T_M_tag      # ①
T_L_tag = T_L_C @ T_C_tag                        # ③
T_M_L = T_M_tag @ inv(T_L_tag)                   # ⑤
T_M_B = T_M_L @ np.eye(4)                        # ⑦ (T_L_B = I)

print(T_C_tag[:3, 3])   # [0.   0.15 1.3 ]
print(T_L_tag[:3, 3])   # [1.5  0.   0.  ]
print(T_M_B[:3, 3])     # [1.   1.2  0.  ]
```

若把 `T_M_B_true` 改成任意位姿（含非零 yaw），最后一行都会精确等于它 —— 说明公式对朝向也成立。

### 3.4 代码对应

```cpp
// pose_math.hpp :: relocalize()
out.T_L_tag = T_L_C * T_C_tag;                 // ③
out.T_M_L   = T_M_tag * out.T_L_tag.inverse(); // ⑤
out.T_M_B   = out.T_M_L * T_L_B;               // ⑦
```

节点里 `lookupTransform(lidar_frame, camera_frame)` → `T_L_C`，
`lookupTransform(lidar_frame, base_frame)` → `T_L_B`，见 `apriltag_localizer.cpp::lookupTf()`。

---

## 4. 现场标定（三件必须做的事）

**定位精度 90% 取决于这三项，不取决于算法。** 顺序建议：先 4.1 → 4.2 → 用 4.3 的「反算法」一次性收尾。

### 4.1 tag 边长 `tag_size`

量的是 **AprilTag 黑色方块图案的外边长**（不含周围白边、不含打印纸），单位 **米**。
`tag36h11` 的图案是 10×10 单元，量整个黑白方块即可。当前配置是 `0.118`（原 demo 的值），**必须实测替换**。

> 填错的典型症状：`~/tag_pose_camera` 的 z 与真实距离成固定比例偏差（例如真实 1 m 报 2 m，说明边长填成了实际的两倍）。

### 4.2 相机外参 `T_B_C` / `T_L_C`

需要的是「相机光学系在 `base_link`（或 `laser`）中的位姿」，注意不是简单的平移：

- 平移：相机光心相对机身原点的 `[x, y, z]`
- 旋转：`base_link`(x前/y左/z上) → 光学系(x右/y下/z前) 的固定轴系变换
  = `roll=−90°, pitch=0°, yaw=−90°`；相机若朝下/朝侧面，还要叠加安装角

三种提供方式（任选其一）：

1. **推荐**：由 URDF / `robot_state_publisher` 发布 `base_link -> camera_color_optical_frame`，
   保持 `publish_camera_tf: false`。
2. 用 `static_transform_publisher`（`tf2_ros`）：
   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x 0.20 --y 0.0 --z 0.15 --roll -1.5708 --pitch 0 --yaw -1.5708 \
     --frame-id base_link --child-frame-id camera_color_optical_frame
   ```
3. 让本节点代发：设 `publish_camera_tf: true`，填写 `camera_tf_parent` 与 `camera_in_parent`，
   例如 `[0.20, 0.0, 0.15, -90.0, 0.0, -90.0]`。

**注意**：同一个 TF 只能有一个发布者，方式 1/2 与方式 3 不要同时用。

精度不够时做一次简单手眼标定：把机器人摆到已知位姿、tag 摆到已知位置，
微调外参的 6 个数，使节点解算的 `(x, y, yaw)` 与真值一致（和 4.3 的反算法同理）。

### 4.3 tag 在场地中的位姿 `T_M_tag`（`tag_poses`）

#### tag 坐标系约定

本包**沿用 `D435_apriltag/tag.cpp` 的角点顺序**（`setupCamera()` 里的 `object_points_`）：

```
p[0] -> (-s,-s,0)   p[1] -> ( s,-s,0)
p[3] -> (-s, s,0)   p[2] -> ( s, s,0)
```

因此：

- **+x**：从 `p[0]` 指向 `p[1]`
- **+y**：从 `p[0]` 指向 `p[3]`
- **+z** = +x × +y（右手系），指向**码正面外侧**，也就是「能看见码的那一侧」

`tag_poses` 的格式：

```
"id, x, y, z, roll, pitch, yaw"     # 单位 m / 度，RPY 为内旋 ZYX
```

常见贴法：

| 贴法 | `T_M_tag` 的 RPY | 说明 |
| --- | --- | --- |
| 平放地面，图案 +x 指向 map +X | `0, 0, 0` | +z 朝天，最常用 |
| 平放地面，图案旋转 θ | `0, 0, θ` | θ 用卷尺/量角器量图案边与 map X 轴夹角 |
| 竖直贴墙（法向水平） | 含 `±90` 的 roll 或 pitch | 需现场确认 +z 指向场地内部 |

> 贴墙的情况容易搞错方向，务必用 `~/tag_pose_camera` 验证：把相机正对 tag，
> 该位姿的平移 z 应约为真实距离，且旋转应显示 tag 的 +z 指向相机。

一个具体例子：tag 贴在 `y = y0` 的墙上，正面朝场地内部（map 的 `−Y`），
图案 `+x` 水平指向 map `+X`、图案 `+y` 朝上。此时 tag 的三轴在 map 中为
`+x = +X`、`+y = +Z`、`+z = −Y`，对应旋转是绕 X 轴 `+90°`，所以
`tag_poses` 填 `"id, x, y0, z, 90, 0, 0"`。
其它墙面同理先写出三轴再推 RPY，**不要死记角度**。

另外节点内置了防呆检查：机器人本体在 map 中应当是水平的，所以 `T_M_B` 的
roll/pitch 理论上接近 0。若解出很大的倾角（默认 > `max_tilt_deg: 30.0`），
节点会打印 `解出的 map->base 姿态异常 ...` 并**拒绝发布 `/initialpose`**。
看到这条日志，基本就是 `tag_poses` 的 RPY 或相机外参的旋转填反了。

#### 最省事的标定法：反算法（强烈推荐）

不必在理论上纠结「图案哪个角是 p[0]」，直接实测：

1. 把 tag 贴在场地固定位置，量出大致 `(x, y, z)`，`tag_poses` 里 RPY 先全填 0。
2. 把机器人**手动摆到一个已知位姿** `(x0, y0, yaw0)`（用卷尺量机身原点，量朝向），让它能看见 tag。
3. 启动节点，看日志的 `map 中 base=(...)`，或
   `ros2 topic echo /apriltag_localizer/lidar_pose_map`。
4. 设解算值为 `(x', y', yaw')`，误差 `(x0−x', y0−y', yaw0−yaw')` 就是 `tag_poses` 的偏差：
   - 平移误差 → 直接加到 `tag_poses` 的 x/y（若随朝向变化，说明外参 `camera_in_parent` 的平移有错）
   - 固定 yaw 偏差 → 加到 `tag_poses` 的 yaw
   - 换几个不同朝向重复一次，若误差随朝向系统性变化 → 检查相机外参的旋转
5. 改完重跑，直到静止时解算值稳定且与真值差在几厘米/几度内。

#### 多个 tag

`tag_poses` 可以配置多个（每个 ID 一行）。节点一帧只用「第一个有效 tag」产生一个样本，
避免不同 tag 的样本被平均到一起。`target_tag_id` 填某个 ID 可强制只用那一个。
未标定的 tag 检测到时会被忽略并打印 WARNING。

---

## 5. 参数说明（`config/localizer.yaml`）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `map_frame` / `base_frame` / `lidar_frame` / `camera_frame` | `map` / `base_link` / `laser` / `camera_color_optical_frame` | 帧名，必须与实际 TF 树一致 |
| `tag_size` | `0.118` | tag 图案外边长 [m]，**必须实测** |
| `tag_poses` | 见 yaml | tag 在 map 中的位姿表 |
| `target_tag_id` | `-1` | `-1` = 任意已标定 tag |
| `trigger_mode` | `auto_once` | `auto_once` 只发一次 / `manual` 只由服务触发 / `continuous` 周期发（**危险，见 8.3**） |
| `min_samples` | `10` | 累积多少帧取均值，抑制 PnP 抖动 |
| `max_spread` | `0.08` | 样本平移最大离散度 [m]，超阈值丢弃本轮 |
| `require_subscriber` | `true` | 等 AMCL 订阅 `/initialpose` 才发，防止消息丢失 |
| `sigma_x` / `sigma_y` / `sigma_yaw` | `0.15` / `0.15` / `0.10` | 初值标准差（代码平方成方差），决定 AMCL 粒子云扩散范围 |
| `tf_timeout` | `0.2` | 查静态 TF 的超时 [s] |
| `max_tilt_deg` | `30.0` | 解出的 `map->base` 允许的最大 roll/pitch；超阈值判定为标定错误并拒绝发布 |
| `publish_debug_poses` | `true` | 发布 3 个中间位姿话题 |
| `show_image` | `false` | OpenCV 预览窗口（检测框/中心点/坐标轴/XYZ）；无桌面必须 false |
| `publish_camera_tf` / `camera_tf_parent` / `camera_in_parent` | `false` / `base_link` / 见 yaml | 可选：由本节点发布相机外参静态 TF |
| `image_width/height/fps` | `640/480/30` | D435 彩色流 |
| `quad_decimate/quad_sigma/nthreads/refine_edges` | `1.0/0.0/4/true` | AprilTag 检测器参数 |

### 节点对外接口

| 类型 | 名称 | 消息 | 用途 |
| --- | --- | --- | --- |
| 发布 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **给 AMCL 的初值** |
| 发布 | `~/tag_pose_camera` | `PoseStamped`(frame=camera) | ① 校验 PnP |
| 发布 | `~/tag_pose_lidar` | `PoseStamped`(frame=laser) | ③ 校验 TF 变换 |
| 发布 | `~/lidar_pose_map` | `PoseStamped`(frame=map) | ⑤ 校验重定位结果 |
| 服务 | `~/relocalize` | `std_srvs/Trigger` | 重新触发一次初始化（机器人被搬动后用） |

---

## 6. 编译与运行

### 6.1 编译

```bash
cd ~/RL/AT_sim2real
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select apriltag_localization
# 也可用项目脚本（额外处理 ROS1/ROS2 的 package.xml 符号链接）：
# ./build.sh apriltag_localization

source install/setup.bash
ros2 pkg executables apriltag_localization   # 应看到 apriltag_localizer
```

正常输出是 `Finished <<< apriltag_localization` + `Summary: 1 package finished`，
**没有任何 warning / error**。若看到 `[apriltag_localization] 依赖不全，跳过 ... 节点编译`
（这是 `STATUS` 提示而不是 CMake Warning），说明第 1 节的三个库没找齐，装好后
`rm -rf build/apriltag_localization` 重新编译即可。

保持零警告的几个实现细节：

- 本包代码启用 `-Wall -Wextra -Wpedantic`；OpenCV / AprilTag 头文件用
  `target_include_directories(... SYSTEM PRIVATE ...)` 引入（`-isystem`），
  因此 AprilTag 的 C 柔性数组 `matd_t::data[]` 不会报
  `ISO C++ forbids flexible array member`，而我们自己的代码仍被完整检查。
- Ubuntu `libapriltag-dev 3.2.0` 的 `image_u8` 里 `width/height/stride` 是
  `const int32_t`，**不能**默认构造后逐个赋值；节点用聚合初始化一次性包装
  OpenCV 灰度数据（零拷贝）。
- 用户级安装的 `libapriltag.so.3` 通过 `INSTALL_RPATH` 写入可执行文件，
  运行时无需设置 `LD_LIBRARY_PATH`：`ldd` 应显示它指向 `~/.local/lib`。

### 6.2 运行前置条件

按顺序确认（缺一不可）：

```bash
# 1) 雷达在发 /scan，且 frame_id 与 lidar_frame 一致
ros2 topic echo /scan --once | head -20

# 2) TF 树完整
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo laser camera_color_optical_frame

# 3) Nav2 地图 .yaml 已准备好（map_server 需要）
ls /abs/path/to/field_map.yaml
```

同时把 `config/localizer.yaml` 里的 `tag_poses`、`tag_size`、帧名按第 4 节改好。

### 6.3 方式 A：完整链路（推荐）

```bash
ros2 launch apriltag_localization full_localization.launch.py \
    map_yaml:=/abs/path/to/field_map.yaml
```

它会依次拉起：

1. `nav2_map_server` → 发布 `/map`
2. `nav2_amcl` → 订阅 `/scan`、`/map`、`/initialpose`
3. `nav2_lifecycle_manager` → `autostart` 自动 configure/activate 上面两个节点
4. **延后 3 秒**启动 `apriltag_localizer` → 保证 AMCL 已订阅 `/initialpose`

`localizer_params` / `amcl_params` 也是 launch 参数，可指向你自己的 yaml。

### 6.4 方式 B：只启动视觉节点

已有导航栈、或 AMCL 由别的 launch 管理时：

```bash
# 终端 1（原有导航栈）
ros2 launch <your_nav_pkg> localization.launch.py ...

# 终端 2（视觉初值）
ros2 launch apriltag_localization apriltag_localization.launch.py \
    params_file:=/abs/path/to/localizer.yaml
```

**启动顺序很重要**：先 AMCL 后视觉节点。`/initialpose` 是 volatile QoS 的一次性消息，
晚订阅会直接丢。节点额外用 `require_subscriber: true` 做了保护：订阅数为 0 时只打 WARNING 不发布，
并保留 `need_publish_`，等 AMCL 起来后下一轮采样会补发。

### 6.5 重新触发一次重定位

机器人被人为搬动、或第一次初始化失败后：

```bash
ros2 service call /apriltag_localizer/relocalize std_srvs/srv/Trigger
```

服务会清空样本、把 `need_publish_` 置回 true，重新累积 `min_samples` 帧后发布一次新的 `/initialpose`。

---

## 7. 验证与调试

建议**按位姿链顺序逐级核对**，不要一上来就看 `/initialpose`。

### 7.1 ① 相机系位姿正确吗

```bash
ros2 topic echo /apriltag_localizer/tag_pose_camera
```

- `position.z` ≈ 卷尺量出的相机到 tag 的距离
- tag 在画面右侧 → `x > 0`；在画面下方 → `y > 0`（光学系 y 向下）
- 数量级不对 → 查 `tag_size`；左右上下反了 → 查相机镜像/外参旋转

### 7.2 ③ 雷达系位姿正确吗

```bash
ros2 topic echo /apriltag_localizer/tag_pose_lidar
ros2 run tf2_ros tf2_echo laser camera_color_optical_frame
```

- 机器人正对 tag 时，`position.x` 应约等于「雷达到 tag 的距离」，`y ≈ 0`
- 若 `x/y` 轴互换或 z 符号不对 → 相机外参 RPY 填错（回想 4.2 的 `−90/0/−90`）

### 7.3 ⑤ 重定位结果正确吗

```bash
ros2 topic echo /apriltag_localizer/lidar_pose_map
```

把机器人摆到地图已知点，看这里输出的 `(x, y, yaw)` 是否吻合。
不吻合 → 回到 4.3 用反算法修 `tag_poses`。

### 7.4 ⑧ `/initialpose` 发出去了吗

```bash
ros2 topic echo /initialpose --once
ros2 topic hz /initialpose          # 正常应只在初始化时看到 1 条
```

节点日志会打印：

```
已发布 /initialpose: x=1.003 y=1.198 yaw=0.8 deg (基于 10 帧, 离散度 0.012 m)
  -> 交给 AMCL 用激光 scan 与地图匹配继续修正
```

### 7.5 AMCL 收敛了吗

```bash
ros2 topic echo /amcl_pose           # 位姿均值及其 6x6 协方差
ros2 topic echo /particle_cloud --once | head -40   # 粒子云应逐渐收紧
ros2 run tf2_ros tf2_echo map base_link
```

在 RViz2 里同时显示 `Map`、`LaserScan`（`/scan`）、`PoseArray`（`/particle_cloud`）、`TF`：
**激光点云轮廓与地图墙线重合** 就是收敛的判据。若 scan 与地图整体错开几厘米，属于 AMCL 正常精修范围；
若错开几十厘米或粒子云始终散开，见第 9 节。

### 7.6 排查表

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 完全没有检测日志 | tag 尺寸/家族不对、光照、运动模糊 | 开 `show_image: true` 看画面；确认是 `tag36h11` |
| `TF ... 不可用` | 帧名写错 / 静态 TF 未发布 | `view_frames` 对照；补 `static_transform_publisher` |
| `尚无节点订阅 /initialpose` | AMCL 未启动或未 activate | 用 6.3 的完整 launch，或检查 lifecycle manager |
| `样本离散度 ... 本轮丢弃` | 初始化时机器人在动 / tag 抖动 | 静止后再触发 `~/relocalize`；适当放宽 `max_spread` |
| 解算 yaw 恒偏 180° | tag 图案朝向理解反了 | 在 `tag_poses` 的 yaw 上加 180，或用反算法 |
| `解出的 map->base 姿态异常 roll=.. pitch=..` | 贴墙 tag 的 ±90° RPY 或相机外参旋转填反 | 对照 4.3 重推三轴；临时调大 `max_tilt_deg` 可看数值，但不要带着错误旋转上线 |
| 解算值随距离线性放大 | `tag_size` 填错 | 重新量图案外边长 |
| AMCL 收敛到错误位置 | 初值协方差太小 + 场地对称 | 调大 `sigma_x/y/yaw`，或增大 `max_particles` |

---

## 8. AMCL 如何「继续修正雷达位姿」

### 8.1 原理（简化）

1. AMCL 收到 `/initialpose`，按其中的协方差在 \((x,y,\theta)\) 附近撒一圈粒子；
   协方差越小，粒子云越集中，表示「我相信视觉初值」。
2. 机器人运动时，用里程计 `odom -> base_link` 的增量按运动模型（`alpha1..alpha5` 噪声）推进粒子。
3. 每帧 `/scan` 到来，用 `laser_model_type: likelihood_field` 把激光端点投到占据栅格地图上打分：
   点云越贴合地图墙线，粒子权重越高。
4. 按权重重采样，粒子云逐渐收紧到真实位姿；取加权均值得到 `T_M_B`。
5. 由 \(T_{M\_O} = T_{M\_B}\,T_{O\_B}^{-1}\) 反算并发布 `map -> odom`，
   于是整条 `map -> odom -> base_link -> laser` 链路被修正。

因为 `laser` 与 `base_link` 是刚性固定 TF，**修正 base_link 就等于修正雷达在 map 中的位姿**。

### 8.2 `/initialpose` 的语义与协方差

- 话题：`/initialpose`，类型 `geometry_msgs/PoseWithCovarianceStamped`，`header.frame_id = "map"`
- AMCL 只取 `x`、`y` 和由四元数换算的 `yaw`，忽略 z/roll/pitch
- 协方差 6×6 行主序，顺序 `x, y, z, roll, pitch, yaw`；本节点只在对角线填
  `sigma_x²`、`sigma_y²`、`sigma_yaw²`，其余填 `1e6`
- `config/amcl.yaml` 中 **`set_initial_pose: false`** 必须保持，否则 AMCL 启动时会用 yaml 里的 `(0,0,0)` 覆盖视觉初值

经验值：视觉初值精度约 5~20 cm / 3~8° 时，`sigma_x = sigma_y = 0.15`、`sigma_yaw = 0.10` 比较稳妥。
初值很准可以调小；场地高度对称（长走廊）则应调大，给粒子留出覆盖正确解的空间。

### 8.3 为什么只发布一次

AMCL 每收到一条 `/initialpose` 都会**用新位姿重新撒粒子、丢弃之前积累的收敛结果**。
如果相机每帧都发，AMCL 会被反复重置，激光匹配永远无法收敛，定位还会一直被 PnP 噪声拉扯。

因此节点默认 `trigger_mode: auto_once`：发布一次后把 `need_publish_` 置 false，
之后只继续检测/打印/发布调试话题，把控制权完全交给 AMCL。
需要重新初始化时显式调用 `~/relocalize`。
`continuous` 模式仅用于调试位姿链，**不要在实际导航中使用**。

### 8.4 关键调参（`config/amcl.yaml`）

| 参数 | 作用 |
| --- | --- |
| `min_particles` / `max_particles` | 粒子数；初值不准或场地大时调大，代价是 CPU |
| `laser_model_type` | `likelihood_field` 对一般室内场地最鲁棒 |
| `laser_likelihood_max_dist` | 激光点到最近障碍的最大计分距离，墙厚/噪声大时调大 |
| `update_min_d` / `update_min_a` | 每移动/转动多少才强制更新，调小更跟手但更耗 CPU |
| `alpha1..alpha5` | 里程计噪声，里程计差就调大，否则粒子云会「过度自信」 |
| `max_beams` | 每帧使用的激光束数，降 CPU 用 |
| `tf_broadcast` | 必须为 true，AMCL 才会发布 `map -> odom` |

---

## 9. 常见问题 FAQ

**Q：没有 D435 相机，能先验证整条链路吗？**
A：可以验证 AMCL 半段。手动发一条初值：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{header: {frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 1.2, z: 0.0},
 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
 covariance: [0.0225, 0, 0, 0, 0, 0,  0, 0.0225, 0, 0, 0, 0,
              0, 0, 1e6, 0, 0, 0,  0, 0, 0, 1e6, 0, 0,
              0, 0, 0, 0, 1e6, 0,  0, 0, 0, 0, 0, 0.01]}}'
```

数学部分则用 3.3 的 Python 脚本验算。

**Q：tag 必须平放在地上吗？**
A：不必。贴墙、贴柱子都行，只要 `tag_poses` 的 RPY 如实描述 tag 坐标系，且 +z 法向朝向相机能看到的一侧。
但 2D AMCL 只用 x/y/yaw，所以 tag 安装姿态里的俯仰/滚转只通过 `T_M_tag` 影响解算，
**必须填对**；平放地面最不容易出错，优先选它。

**Q：精度能到多少？**
A：PnP 单帧在 1~2 m 距离、tag 边长 10 cm 量级时，平移约 1~3 cm、朝向约 1~3°；
外参和 tag 场地坐标的标定误差通常是主要误差源（可达 5~10 cm）。
因此 `min_samples: 10` 做多帧平均 + `max_spread` 剔除抖动，最后交给 AMCL 精修。

**Q：相机外参用了图像时间戳吗？**
A：节点用 `tf2::TimePointZero`（最新可用 TF）查询，因为相机与雷达的外参、以及 `base_link -> laser`
都是**静态** TF，不涉及时间插值。若相机装在云台上会动，需要改成用图像采集时刻查询，并给
`tf_buffer_` 配 `tf2_ros::MessageFilter` / 带时间戳的 `lookupTransform`。

**Q：为什么节点用了独立采集线程？**
A：`rs2::pipeline::wait_for_frames()` 是阻塞调用，放在 rclcpp executor 回调里会卡住服务/定时器。
节点在 `start()` 里起一个 `std::thread` 跑 `captureLoop()`，`rclcpp::spin()` 在主线程处理
`~/relocalize` 服务；样本用 `std::mutex` 保护，发布状态用 `std::atomic<bool>` 同步。

**Q：`lidar_frame` 和 `base_frame` 填成同一个可以吗？**
A：可以，`lookupTf()` 检测到两者相同直接返回单位变换。但只有当雷达原点确实与 base_link 原点重合时才这么做。

**Q：和 `obstacle_game` 的 `map -> base_link` 定位怎么配合？**
A：`obstacle_game::Robot::refresh_robot_pose()` 里查的就是 `lookupTransform("map", "base_link", ...)`。
AMCL 激活并发布 `map -> odom` 后，这条 TF 就是可用的，自动导航无需任何改动。

---

## 10. 代码导览（建议按这个顺序读）

| 顺序 | 位置 | 看点 |
| --- | --- | --- |
| 1 | `pose_math.hpp` 顶部注释 | 全文记号约定与位姿链总表 |
| 2 | `pose_math.hpp::fromSolvePnP()` | Rodrigues → `tf2::Matrix3x3` → `tf2::Quaternion`，得到 `T_C_tag` |
| 3 | `pose_math.hpp::relocalize()` | 三行核心公式 ③⑤⑦ |
| 4 | `pose_math.hpp::fromTransformMsg()` | `lookupTransform(target, source)` 为什么就是 `T_target_source` |
| 5 | `apriltag_localizer.cpp::setupCamera()` | D435 内参、角点约定（与原 demo 完全一致） |
| 6 | `apriltag_localizer.cpp::processDetection()` | ① solvePnP → ② 查 TF → ③⑤⑦ → 累积样本 |
| 7 | `apriltag_localizer.cpp::lookupTf()` | 静态外参查询与超时处理 |
| 8 | `apriltag_localizer.cpp::publishInitialPose()` | `/initialpose` 消息、协方差、只发一次的保护 |
| 9 | `pose_math.hpp::summarize()` | 多帧平均（yaw 用 sin/cos 在圆上平均）与离散度剔除 |
| 10 | `launch/full_localization.launch.py` | 完整启动顺序与 3 秒延迟的原因 |

---

## 附：从原 `tag.cpp` 改了什么

| 原 demo | 本功能包 |
| --- | --- |
| `main()` 里死循环打印 `X/Y/Z` | ROS2 节点，独立采集线程 + `rclcpp::spin` |
| 位姿只到 `tvec` | 补 `rvec` → 完整 `T_C_tag`（`Rodrigues` + `tf2::Transform`） |
| 无 TF 概念 | 查 `camera→lidar`、`base→lidar` 静态外参，得到 `T_L_tag` |
| 无地图概念 | 引入人工标定的 `T_M_tag`，反算 `T_M_L / T_M_B` |
| 无初始化接口 | 多帧均值 + 协方差 → `/initialpose`，并提供 `~/relocalize` 服务 |
| 只能用 `cv::imshow` 目视 | 3 个 `PoseStamped` 调试话题 + 可关闭的图像预览 |
| 无扫描匹配 | `config/amcl.yaml` + 完整 launch，AMCL 用 `/scan` 与 `/map` 持续修正 |
