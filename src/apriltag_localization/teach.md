> 当前 `apriltag_localizer` 已改为 Tag 对齐现实 odom 到理想地图，发布 `map -> odom` 和 `/odom_corrected`。下文的 AMCL、`/initialpose` 与旧参数描述属于历史方案，当前启动和配置请以 [README.md](README.md#43-重定位与校正里程计) 及 `config/localizer.yaml` 为准。

# AprilTag 视觉重定位教学文档

日常使用、位置修改和 TF 配置请先看 [README.md 使用指南](README.md)。本文保留依赖安装与详细推导；角点朝向、单位和接口以当前源码及 README 为准。

> 面向 `AT_sim2real` 工作区（ROS2 Humble）的功能包 `apriltag_localization`。
> 目标：把 `~/qq_down/D435_apriltag/tag.cpp` 里那个「打印 tag 在相机系下 XYZ」的 C++ demo，
> 改造成能给激光定位做**全局初值**的 ROS2 节点。
>
> **只想先做无雷达测试？** 直接看文末「第 11 节 无雷达纯视觉测试」：
> 不需要雷达 / AMCL / 地图 / TF 树，用 4x4 矩阵配置常数位姿，让机器人走到 tag 码前面的固定位姿。

---

## 0. 这个包解决什么问题

激光定位（AMCL）有个经典问题：机器人开机时不知道自己在地图的哪里，粒子滤波要靠运动 + 观测慢慢收敛；
场地大、地形对称时容易收敛到错误的局部极值，甚至完全不收敛。

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
| `include/apriltag_localization/pose_math.hpp` | 纯数学：solvePnP 结果转 `tf2::Transform`、整条位姿链、协方差、样本统计、tag 标定表解析、**4x4 矩阵解析/生成** |
| `src/apriltag_localizer.cpp` | ROS2 节点（完整链路）：D435 取图、AprilTag 检测、查 TF、发 `/initialpose`、重定位服务 |
| `src/tag_approach.cpp` | ROS2 节点（**无雷达测试**）：4x4 矩阵配置、**RealSense / UVC 自适应取图**、视觉伺服走到 tag 前固定位姿、急停与示教 |
| `config/localizer.yaml` | 完整链路节点参数（帧名、tag 尺寸、**tag 场地位姿**、外参、发布策略、协方差） |
| `config/approach.yaml` | **测试用 config.yaml**：两个 4x4 常数位姿矩阵 + 图像源/内参 + 全部控制/安全参数 |
| `config/amcl.yaml` | `nav2_amcl` 示例参数（激光 scan 与地图匹配） |
| `launch/apriltag_localization.launch.py` | 只启动完整链路视觉节点 |
| `launch/full_localization.launch.py` | 一键启动 `map_server + amcl + lifecycle_manager + 视觉节点` |
| `launch/tag_approach.launch.py` | **只启动无雷达测试节点**（不启动任何导航组件） |
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

本包的角点顺序（`setupCamera()` 里的 `object_points_`）与 apriltag 库的 `det->p[]` 严格对应：

```
p[0] -> ( s, s,0)   p[1] -> (-s, s,0)
p[3] -> ( s,-s,0)   p[2] -> (-s,-s,0)
```

角点编号由 tag 解码方向决定，随码旋转，不固定对应图像的右上/左上等位置。
当前物点对应关系决定：

- **+x**：从 `p[1]` 指向 `p[0]`
- **+y**：从 `p[3]` 指向 `p[0]`
- **+z** = +x × +y（右手系），指向码正面外侧

预览红/绿/蓝分别表示 tag 的 +X/+Y/+Z 投影。码旋转后红轴可能向左、绿轴可能向下，
这不代表相机光学系改变。改动代码的物点轴约定后，必须重新核对 `tag_poses` 和示教目标，
不能不加区分地给 map yaw 加减 180°。

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

把机器人摆到已知真实位姿，并测得 tag 在相机系中的完整位姿：

```text
T_M_tag = T_M_B_known * T_B_C * T_C_tag_measured
```

`T_M_B_known` 必须以机体原点为准，包含位置和旋转；`T_B_C` 来自标定外参或 TF，
`T_C_tag_measured` 可从 `/apriltag_localizer/tag_pose_camera` 获取。将结果转为
`x,y,z,roll,pitch,yaw`（米/度）填回 `tag_poses`，在不同机器人朝向下重复验证。
相机位姿话题的姿态是四元数，不能直接把其中的 x/y/z 当成 RPY。

只给 tag 的 XY 累加最终位置误差，不能普遍修复旋转标定错误。
`lidar_pose_map` 是雷达位姿而非机体位姿；两者原点不重合时不能拿雷达坐标直接比较机体真值。

#### 多个 tag

`tag_poses` 可以配置多个（每个 ID 一行）。节点一帧只用「第一个有效 tag」产生一个样本，
限制每帧的样本数；跨帧仍可能切换 ID，各 tag 的 map 标定必须一致。`target_tag_id` 填某个 ID 可强制只用那一个。
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
| 11 | `pose_math.hpp::fromMatrix4x4()` | 4x4 矩阵解析 + 正交性/det 校验（测试节点的配置入口） |
| 12 | `tag_approach.cpp::captureLoop()` | 无 TF 树的三段位姿链、目标示教、倾角防呆 |
| 13 | `tag_approach.cpp::controlTick()` | P 控制、限速平滑、到位判定、丢 tag 急停 |

---

## 11. 无雷达纯视觉测试：走到 tag 码前面的固定位姿

> 本节**完全独立**于前面 1~10 节：不需要雷达、不需要 `/scan`、不需要 AMCL / `map_server`，
> 不需要 `map` 帧，也不需要 TF 树。所有常数位姿由人手以 **4x4 矩阵**写在
> `config/approach.yaml`（就是测试用的 `config.yaml`）里，节点 `tag_approach` 只做视觉闭环。

### 11.1 链路

```
相机彩色图 -> AprilTag + solvePnP -> T_C_tag                      (tag 在相机系)
                     │
                     ├─ × T_B_C        (4x4 常数，相机安装外参)
                     ▼
                  T_B_tag                                       (tag 在当前机体系)
                     │
                     ├─ × T_tag_B_goal (4x4 常数，目标站位)
                     ▼
                  T_Bnow_Bgoal                                  (目标机体在当前机体系)
                     │
                     └─ (dx, dy, yaw) -> P 控制 -> robot_move_cmd / cmd_vel -> 到位停车
```

速度命令以固定频率（`control_rate`，默认 **50 Hz**，见 11.5「控制频率对齐」）由**独立定时器**发布，
相机采集在**独立线程**里跑；两者通过带 mutex 的最新误差通信。
这样即使相机掉帧或 tag 被遮挡，定时器也能可靠地超时停车。

**图像源（`image_source`）**：取图与检测/解算是解耦的，节点里只有 `grabFrame()` 接触相机 SDK，
上层拿到的永远是一张 BGR `cv::Mat`。

| 取值 | 行为 | 内参来源 |
| --- | --- | --- |
| `auto`（默认） | 先探 D435，有就用它，没有就退回普通 USB 摄像头 | 按实际选中的源 |
| `realsense` | 强制 librealsense2 读取 D435 彩色流 | SDK 的 `get_intrinsics()`，**无需标定** |
| `uvc` | 强制 OpenCV `VideoCapture`（V4L2）读取 `uvc_device` | `camera_fx/fy/cx/cy` + `distortion`，需自己标定 |

`uvc_device` 可以写设备路径（`/dev/video2`）也可以写纯数字索引（`0`），用
`v4l2-ctl --list-devices` 查。普通 UVC 摄像头**没有内参**，此时若 `camera_fx/fy` 为 0，
节点会按 `hfov_deg`（默认 60°）估算 \(f_x = (w/2)/\tan(\text{hfov}/2)\)、\(f_y=f_x\)、主点取图像中心，
启动日志会给出显式告警：

```
UVC 未标定内参：按 hfov_deg=60.0 估算 fx=fy=554.3，PnP 距离会有系统性偏差，
请棋盘格标定后填 camera_fx/fy/cx/cy
```

> 估算内参能跑通流程，但 **PnP 解出的 `dx` 会有系统性缩放误差**（`dx` 直接来自 `tvec`），
> 表现为「离 tag 越远、误差越大」。要求精度时先做棋盘格标定：
> `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=<你的图像话题>`，
> 把得到的 \(f_x, f_y, c_x, c_y\) 与 \(k_1,k_2,p_1,p_2,k_3\) 填进 `config/approach.yaml`。
> 驱动若没接受请求的分辨率，节点会以**实际分辨率**为准并打印 WARN（内参必须和实际图像尺寸匹配）。

**画面太暗怎么办（曝光 / 增益）**

UVC 摄像头没有「自动亮度」，曝光和增益是驱动里两个独立控制项，任何一次「手动 8 ms + 最低增益」
都能让画面接近全黑——本机就踩过这个坑：`auto_exposure=1 (Manual)`、`exposure_time_absolute=80`、`gain=1`。
节点启动时会按 `config/approach.yaml` 把这些值写回驱动，并逐项打印生效值：

```
UVC auto_exposure(manual) = 1
UVC exposure[100us] = 330     # 330 * 100us = 33 ms
UVC gain = 64
```

本机实测亮度（中位值 /255，括号为过曝像素占比）：

| 设置 | 亮度 | 说明 |
| --- | --- | --- |
| 自动曝光 `auto_exposure: true` | 56 | 偏暗，暗光下仍不够 |
| 33 ms + gain 48 | 85 | 可用 |
| **33 ms + gain 64** | **110** | **★ 默认值**，仍在 30 fps 上限内 |
| 60 ms + gain 48 | 128 | 更亮，但帧率降到约 16 fps（曝光不能超过 1/fps） |
| 100 ms + gain 64 | 194 | 开始过曝，tag 黑白边界会糊 |

```yaml
auto_exposure: false    # 手动曝光
exposure: 330           # [100us] 330 = 33ms；范围 50~10000
gain: 64                # 1~128
```

调参顺序建议：

1. **先固定 `gain`，只动 `exposure`**：曝光决定亮度且不引入噪点，但 `exposure > 1/fps`（33 ms @30 fps）
   会掉帧，掉帧严重就先把 `fps` 降到 15。
2. 曝光到 33 ms 还不亮，再加 `gain`（1→48→64→96），代价是噪点变大、tag 边缘变毛。
3. 强光下过曝就把 `exposure` 往下调（200、150），或直接 `auto_exposure: true` 交回固件。
4. 现场快速试值不用改代码，直接在终端调，节点启动时会覆盖回去，所以**要固化的值写进 YAML**：

```bash
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=330   # 33 ms
v4l2-ctl -d /dev/video2 --set-ctrl=gain=64
v4l2-ctl -d /dev/video2 --get-ctrl=auto_exposure,exposure_time_absolute,gain
```

> 日光灯下画面有横向条纹时设工频（OpenCV 没有映射这个控制项，只能手工设一次）：
> `v4l2-ctl -d /dev/video2 --set-ctrl=power_line_frequency=1`（国内 50 Hz）。
> 调好的有效范围可用 `v4l2-ctl -d /dev/video2 --list-ctrls` 查，不同摄像头差别很大。

RealSense 走 `rs2_option`，同一套参数名的量纲不同：`exposure` 仍按 100 µs 书写（内部 ×100 转 µs），
`gain` / `brightness` 是 RealSense 自己的刻度（D435 彩色 gain 16~248），**不要照抄 UVC 的数值**。

### 11.2 两个 4x4 矩阵的语义（最容易写反的地方）

沿用全文记号：`T_A_B` 表示把 B 系坐标变换到 A 系，即 `p_A = T_A_B * p_B`。

| 参数 | 定义 | 含义 | 谁提供 |
| --- | --- | --- | --- |
| `T_B_C` | \(p_B = T_{B\_C}\,p_C\) | **相机光学系在机体系中的位姿**（安装外参）：相机装在哪、朝哪 | 按安装实测填 |
| `T_tag_B_goal` | \(p_{tag} = T_{tag\_B\_goal}\,p_{B\_goal}\) | **目标机体系在 tag 坐标系中的位姿**：机器人最后要站在 tag 的哪里、朝哪 | 手填或 `~/record_goal` 示教 |

矩阵按**行主序 16 个数**写，YAML 里可以直接排成 4 行，看起来就是矩阵：

```yaml
T_B_C: [ 0.0,  0.0,  1.0,  0.20,     # [ r00 r01 r02 tx ]
        -1.0,  0.0,  0.0,  0.00,     # [ r10 r11 r12 ty ]
         0.0, -1.0,  0.0,  0.15,     # [ r20 r21 r22 tz ]
         0.0,  0.0,  0.0,  1.00]     # [  0   0   0   1 ]
```

上面这个 `T_B_C` 就是「相机水平前视、装在机身前 0.20 m / 高 0.15 m」：
机体系（x 前 / y 左 / z 上）→ 光学系（x 右 / y 下 / z 前）的轴系旋转，
与第 4.2 节的 `rpy = (-90°, 0, -90°)` 完全一致。

节点启动时会**校验矩阵**（`pose_math::fromMatrix4x4`），填错会立刻报出来而不是让机器人乱跑：

- 必须恰好 16 个数、最后一行必须是 `(0, 0, 0, 1)`
- 左上 3×3 必须正交（\(R^TR = I\)），否则报「不是正交旋转矩阵」
- 行列式必须为 `+1`，否则报「坐标系含镜像，请检查轴方向」

> 注意：矩阵**转置写反**仍是合法旋转，校验抓不出来，只会让机器人朝反方向走。
> 所以推荐用 11.4 的示教法生成 `T_tag_B_goal`，不要手推。

### 11.3 位姿链推导

\[
\begin{aligned}
T_{C\_tag} &= \text{solvePnP}(rvec, tvec) && \text{① tag 在相机系}\\
T_{B\_tag} &= T_{B\_C}\,T_{C\_tag} && \text{② tag 在当前机体系}\\
T_{B_{now}\_B_{goal}} &= T_{B\_tag}\,T_{tag\_B\_goal} && \text{③ 目标机体在当前机体系}
\end{aligned}
\]

③ 之所以成立，是因为 `tag` 在中间自然抵消：
\(T_{B_{now}\_tag}\,T_{tag\_B_{goal}}\)。

**到位自检**：若机器人已经在目标位姿，则当前机体就是目标机体 \(B = B_{goal}\)，
于是 \(T_{B\_tag} = T_{B_{goal}\_tag} = T_{tag\_B\_goal}^{-1}\)，代入 ③ 得
\(T_{B_{now}\_B_{goal}} = I\)，误差全为 0 —— 这正是控制律的收敛点。

从 ③ 取平移 `dx, dy` 和 `yaw`（`getRPY`），就是「目标点在我（当前机体）的前/左多远、
我要转多少度才能和目标朝向一致」，可直接做 P 控制。

用真实 C++ 代码跑过的数值验证（`toMatrix4x4` 往返误差 0）：

| 情形 | 解出 dx | dy | yaw 误差 | 控制响应 |
| --- | --- | --- | --- | --- |
| 已在目标位姿 | 0 | 0 | 0 | 零速站立 |
| 从目标位姿沿自身 −x 后退 0.5 m | **+0.5** | 0 | 0 | `vx > 0` 前进（方向正确） |
| 相对目标朝向偏 +20° | 0 | 0 | **−20°** | `wz < 0` 反向转回（符号正确） |

### 11.4 最省事的标定：一键示教 `~/record_goal`

手推 `T_tag_B_goal` 要搞清楚 AprilTag 码坐标系的轴方向，很容易错。
**直接示教**：

```bash
# 1) 启动节点（参数里 T_tag_B_goal 先随便放一个合法占位矩阵）
ros2 launch apriltag_localization tag_approach.launch.py

# 2) 把机器人手动摆到「tag 前面那个固定位姿」，让相机稳定看见 tag
# 3) 示教
ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger
```

节点会在日志里打印：

```
已示教 T_tag_B_goal，请把下面的 4x4 矩阵抄回 config/approach.yaml:
  [   0.000000,   1.000000,   0.000000,   0.000000]
  ...
```

把这 16 个数抄进 `config/approach.yaml` 的 `T_tag_B_goal`，重启即可。
原理：示教那一刻相机看到的是 \(T_{B_{goal}\_tag}\)，而节点要的正是它的逆
\(T_{tag\_B\_goal} = T_{B\_tag}^{-1}\)。

`T_B_C` 仍需按安装外参填（或从 `rpy + 平移` 生成），但它不随目标点变化，标一次就够。

### 11.5 控制律与安全

| 机制 | 说明 |
| --- | --- |
| `holonomic`（默认） | `vx = kp_x·dx`，`vy = kp_y·dy`，`wz = kp_yaw·yaw`；四足全向 |
| `differential` | 先转向对准 `atan2(dy, dx)` 再前进，不发横移 |
| `yaw_align_band` | 朝向偏差超过该角度时只转向、不前进，避免「斜着冲过去」 |
| 限速 / 死区 / 一阶平滑 | `max_vx/vy/wz`、`min_velocity`、`smooth_tau`（时间常数），防止起步猛、末端抖 |
| 到位判定 | `dist ≤ pos_tolerance` 且 `|yaw| ≤ yaw_tolerance` 连续保持 `hold_time` 秒 |
| 丢 tag 急停 | 超过 `lost_timeout` 没有新检测 → 立即发零速；`resume_on_reacquire` 控制是否自动恢复 |
| 未激活不抢占 | `auto_start: false` 时节点**完全不发速度**，控制权留给遥控器；`~/start` 才开始 |
| 到位保持零速 | 底层驱动会保持最后收到的速度命令，因此到位/停车时**持续**发零速站立帧 |
| 倾角防呆 | 解出的目标相对当前机体 roll/pitch > `max_tilt_deg` → 判定矩阵填反，本帧忽略 |

#### 控制频率对齐（`rl_real_atdog2` 实例）

测试时两个进程：先 `ros2 run rl_sar rl_real_atdog2`，再启动本节点发 `robot_move_cmd`。
`rl_real_atdog2.cpp` 里有三条独立循环，速度命令要走过它们才真正生效：

| 循环 | 周期 | 频率 | 作用 |
| --- | --- | --- | --- |
| `loop_command` | 0.05 s（写死） | 20 Hz | `KeyboardInterface()` 读键盘，**不碰 `remote_cmd`** |
| `loop_control` | `dt` = 0.005 s | 200 Hz | `RobotControl()`：读 `remote_cmd` → `control.setMode/setVel` → 下发电机命令 |
| `loop_rl` | `dt × decimation` = 0.02 s | **50 Hz** | `RunModel()`：`obs.commands = {control.x, control.y, control.yaw}`，速度真正进策略 |

`dt` / `decimation` 来自 `policy/atdog2/base.yaml`（`dt: 0.005`、`decimation: 4`）。
`robot_move_cmd` 的订阅回调只做 `remote_cmd = msg`（QoS depth 10），
**没有时间戳、没有看门狗**，命令是锁存的：发得再快也只是被 200 Hz 循环重复读，
但策略每 20 ms 才采用一次。

所以 `config/approach.yaml` 取 `control_rate: 50.0`（周期 20 ms），与 `loop_rl` 一一对应。
原来的 20 Hz 与 50 Hz 不成整数倍，会出现「2 个 / 3 个策略周期交替更新」的相位抖动，故不再使用。

> 视觉误差受 D435 彩色流限制最多 30 Hz 更新；控制环 50 Hz 时对误差做零阶保持，
> 低通按时间常数换算，因此提高频率不会让控制变激进。

**凡是按「周期数」定义的参数都已改成时间制**，改频率不会悄悄改变控制行为：

| 参数 | 含义 | 代码里的换算 |
| --- | --- | --- |
| `smooth_tau: 0.10` | 一阶低通时间常数 [s] | \( \alpha = 1 - e^{-T/\tau}\)，\(T = 1/control\_rate\) |
| `hold_time: 0.30` | 到位判定需连续满足的时间 [s] | `hold_ticks = ceil(hold_time × control_rate)` |
| `lost_timeout: 0.5` | 丢 tag 超时 [s] | 本来就是秒，无需换算 |

> **安全提醒**：`rl_real_atdog2` 对 `robot_move_cmd` 没有超时保护。
> 若本节点被 `kill -9` 或异常崩溃，机器人会保持最后一条速度命令一直走。
> 正常 Ctrl-C 退出时本节点析构会补发一条零速站立帧；真机测试请随时准备
> `~/stop` 或遥控器急停，建议后续在 `rl_sar` 侧给 `remote_cmd` 加时间戳超时保护。

三个服务：

```bash
ros2 service call /tag_approach/start       std_srvs/srv/Trigger  # 开始趋近
ros2 service call /tag_approach/stop        std_srvs/srv/Trigger  # 急停并交还控制权
ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger  # 示教目标位姿
```

调试话题（frame 固定写 `base_link`，纯显示用，不参与计算）：

```bash
ros2 topic echo /tag_approach/tag_pose_body    # ② tag 在当前机体系的位姿
ros2 topic echo /tag_approach/goal_pose_body   # ③ 目标机体在当前机体系的位姿（误差来源）
```

### 11.6 速度输出接口（很容易踩坑）

`cmd_output` 三选一：

| 值 | 消息 / 话题 | 适用 |
| --- | --- | --- |
| `robot_move_cmd`（默认） | `robot_msgs/Cmd` → `robot_move_cmd` | 本工作区实机（`rl_sar` 的 `rl_real_atdog2/3`） |
| `cmd_vel` | `geometry_msgs/Twist` → `/cmd_vel` | 通用底盘 / Gazebo 仿真 |
| `none` | 不发布，只打日志 | **纯算法干跑**，第一次测试推荐 |

**关键约定**：本工作区 `robot_msgs/Cmd` 的 `vz` 是**偏航角速度**，不是 z 向线速度
（`obstacle_game::Pilot` 里 `cmd.vz = omega`，`rl_real_atdog2.cpp` 里 `setVel(vx, vy, vz)`）。
所以节点把 `wz` 填进 `cmd.vz`，Twist 模式则填 `angular.z`。

另一个坑是 `mode`：`rl_real_atdog2.cpp::RobotControl()` 只在
`remote_cmd.mode != 0` 时才由上层接管，`fsm_atdog2.hpp::ResolveRemoteModeState()`
再把 mode 映射成 FSM 状态。节点默认：

- `cmd_mode: 2` → `RLFSMStateRLLocomotion`（RL 行走，运动中）
- `stand_mode: 1` → `RLFSMStateGetUp`（位控站立，到位/停车时）
- `mode = 0` → `setMode(0)` 且**不调用 `setVel`**，`control.x/y/yaw` 保持上次值，本节点不会发 0

> 到位时切 `mode=1` 会触发一次 FSM 状态切换（与 `obstacle_game::Pilot::stop()` 的约定一致）。
> 若不希望有姿态过渡，把 `stand_mode` 也设成 `2`，靠零速度让 RL 策略原地站稳。

### 11.7 运行步骤（建议按这个顺序）

```bash
cd ~/RL/AT_sim2real
source /opt/ros/humble/setup.bash
source install/setup.bash

# ① 编译（正常应零 warning / 零 error）
colcon build --symlink-install --packages-select apriltag_localization
source install/setup.bash
ros2 pkg executables apriltag_localization     # 应同时看到 apriltag_localizer 和 tag_approach
```

**第一次务必干跑**：把 `config/approach.yaml` 的 `cmd_output` 改成 `none`，
确认日志里的位姿误差方向和数量级合理，机器人绝不会动。

启动时会打印实际选中的图像源与内参，先核对这两行再往下走：

```
image_source=auto -> uvc
图像源=UVC /dev/video2 | 640x480@30 | fx=554.3 fy=554.3 cx=320.0 cy=240.0 | tag_size=0.1180 m
相机预览窗口：开（叠加 tag 检测框与坐标轴，ESC 关闭）
```

**相机预览窗口**由 `tag_approach` 自己用 OpenCV 创建（窗口标题「Tag Approach 相机预览」），
画面里叠加 tag 检测框、ID 与 PnP 坐标轴——找 tag、判断 tag 装没装正、看 dx 方向对不对都靠它。
窗口由 launch 参数控制，默认开：

```bash
ros2 launch apriltag_localization tag_approach.launch.py                 # 开窗口
ros2 launch apriltag_localization tag_approach.launch.py show_image:=false  # 不开窗口
```

- 窗口可拖动缩放，画面按窗口大小自适应；按 **ESC** 关窗口，节点继续跑（不影响控制）。
- 需要图形界面：本机桌面直接可用，远程 `ssh` 用 `ssh -X` 转发。
- 无 `DISPLAY` 时节点打印 WARN 并**自动禁用窗口**（OpenCV 的 `imshow` 在无图形环境下会直接 abort）。
- 这个窗口不依赖 `realsense2_camera` / `usb_cam`，所以也不需要 `rqt_image_view`；
  如果确实想在 ROS 里看图，先把图像发布成话题再说（当前节点不发图像话题）。

设备不确定时先查：

```bash
v4l2-ctl --list-devices        # 普通 USB 摄像头 -> 填到 uvc_device
rs-enumerate-devices | head    # D435（装了 librealsense2-utils 才有）
```

**两个进程、两个终端**（每个终端都要 `source install/setup.bash`）：

```bash
# 终端 A：底层控制器（atdog2 实机），先启动
ros2 run rl_sar rl_real_atdog2

# 终端 B：视觉趋近节点，后启动
ros2 launch apriltag_localization tag_approach.launch.py

# 终端 C：核对移动话题
ros2 interface show robot_msgs/msg/Cmd     # int32 mode / float32 vx,vy,vz,wheel_vel
ros2 topic info /robot_move_cmd            # 应能看到 rl_sar 的订阅 + tag_approach 的发布
```

> **不要同时运行 `obstacle_game` / `remote_node`**：`obstacle_game::Robot` 也会往
> `robot_move_cmd` 发 `Cmd`，会覆盖本节点的速度和 `mode`，导致机器人乱动。
> 测试期间只保留 `rl_real_atdog2` 与 `tag_approach` 两个发布/订阅方。

首次标定与启动：

```bash
# ① 摆好机器人 -> 示教 -> 把打印的 4x4 矩阵抄回 yaml -> 重启节点
ros2 service call /tag_approach/record_goal std_srvs/srv/Trigger

# ② 把机器人挪开一点，开始趋近
ros2 service call /tag_approach/start std_srvs/srv/Trigger

# ③ 观察命令与频率（注意 auto_start=false 时未激活不发消息，需先 ~/start）
ros2 topic hz /robot_move_cmd               # 应约 50 Hz
ros2 topic echo /robot_move_cmd
ros2 topic echo /tag_approach/goal_pose_body

# ④ 任何不对劲立即急停
ros2 service call /tag_approach/stop std_srvs/srv/Trigger
```

`ros2 topic hz` 显示约 50 Hz 就说明本节点的发布频率已与
`rl_real_atdog2` 的 `loop_rl`（`dt × decimation = 20 ms`）对齐。

日志会持续打印：

```
T_C_tag | x=-0.0213 y=+0.0084 z=+1.0021 m | roll=+178.94 pitch=-0.42 yaw=+1.13 deg
tag 0 | 机体系误差 dx=+0.512 dy=-0.031 dz=+0.002 yaw=-3.2 deg
cmd vx=+0.320 vy=-0.015 wz=-0.067 | err d=0.513 yaw=-3.2 deg
已到达目标位姿 (dist=0.038 m, yaw=1.1 deg)，保持零速站立
```

`T_C_tag` 是 **tag 中心在相机光学系下**的位姿（6 个量，1 Hz 节流）：

- `x/y/z` 单位 m，光学系 x 右 / y 下 / z 前。`z` 是轴向深度，
  中心到相机原点的直线距离是 `sqrt(x*x+y*y+z*z)`；倾斜码平面的垂直距离也不能直接用 z 代替。
- `roll/pitch/yaw` 单位 deg，满足 `R = Rz(yaw)·Ry(pitch)·Rx(roll)`。
  tag +X 朝图像右、+Y 朝图像上且正对相机时，约为 `(±180°,0°,0°)`；
  其它码朝向不能直接套这个基准。一般情况下 Euler 角不是绕 tag 自身三轴的独立转角。
- 这一行是 PnP 原始结果，没有经过 `T_B_C`；后面「机体系误差」才是控制用量。
- UVC 未标定时，包括 z 在内的位姿都可能有系统误差。请标定内参、核对实际分辨率，
  并量取图案黑框外边长作为 `tag_size`。
- 当前趋近节点没有 `tag_pose_camera` 话题，六维相机位姿看日志；全局重定位节点才提供
  `/apriltag_localizer/tag_pose_camera`（PoseStamped，姿态为四元数）。

### 11.8 排错

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `image_source 指定 realsense 但未检测到 D435` | 没接 D435 / 被别的进程占用 / udev 规则缺失 | `rs-enumerate-devices` 检查，关掉其它相机进程；或把 `image_source` 设为 `auto`/`uvc` |
| `探测 RealSense 失败 (...)，按 UVC 处理` | librealsense 拿不到设备访问权限 | `sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules`，重新插拔；只用 UVC 可忽略 |
| `无法打开 UVC 摄像头 '/dev/video2'` | 设备号变了 / 被 `usb_cam` 等节点占用 / 无读权限 | `v4l2-ctl --list-devices` 查实际路径，改 `uvc_device`；`sudo usermod -aG video $USER` 后重新登录 |
| `UVC 读帧失败` 反复打印 | 分辨率/格式不被支持、USB 带宽不足、线松 | 降到 `640x480@30`，换 USB 口，或 `v4l2-ctl -d /dev/video2 --list-formats-ext` 核对支持的模式 |
| **启动后不打印 `T_C_tag` / 任何位姿** | 最常见是 `tag_id` 与视野里的码不符：节点只处理目标 ID，其它码静默跳过（本机实测视野里是 **ID 5**，当前 YAML 为 5，C++ 参数缺省值为 0） | 看预览窗口里画在每个码旁的 `ID: x`，或日志 `本帧检测到 N 个 tag (id=...)`，把 `tag_id` 改成那个 ID；临时换码可设 `tag_id: -1` 接受任意 ID |
| 日志有 `本帧未检测到任何 AprilTag` | 视野里没有完整码 / 画面太暗或过曝 / tag 家族不匹配（本包只检测 tag36h11） | 调整相机朝向、核对 11.1 的曝光参数、实测 `tag_size`（黑框外边长） |
| 检测到 tag 但没有 `T_C_tag` 行 | `solvePnP` 失败或 `z` 不在 (0,10] m；倾角异常会阻止后续机体系误差，但不会阻止前面的 `T_C_tag` 日志 | 看紧随其后的 WARN：前两个查内参/`tag_size`；倾角异常多半是改过角点顺序后 **`T_tag_B_goal` 没重新示教**，重新 `~/record_goal` |
| 预览画面很暗 / 几乎全黑 | 驱动被设成「手动 8 ms + 最低增益」（`auto_exposure=1`、`exposure_time_absolute=80`、`gain=1`） | 用 `config/approach.yaml` 的 `auto_exposure/exposure/gain` 覆盖，见 11.1「画面太暗怎么办」 |
| 画面亮但 tag 检测不到 | 过曝导致黑白边界糊掉，或工频条纹 | 降低 `exposure`（330→200）；`v4l2-ctl --set-ctrl=power_line_frequency=1` |
| `UVC 设置 xxx 失败` | 该摄像头没有这个控制项 | 无害，`v4l2-ctl --list-ctrls` 看支持哪些，把对应参数设回 `-1` |
| `show_image=true 但没有图形界面 ...` | 在无桌面的 ssh / systemd 里启动 | 用本机桌面启动、`ssh -X` 转发，或 `show_image:=false` 关窗口 |
| 预览窗口一打开就崩 / 一闪而过 | OpenCV 的 Qt/GTK 后端缺库，或 `DISPLAY` 指向失效的 X server | `sudo apt install libgtk2.0-dev pkg-config` 重装 OpenCV 后端；`echo $DISPLAY` 确认有效（如 `:1`） |
| 没有预览窗口但日志有 `tag 0 \|` | 窗口被 ESC 关掉了（只关窗口不停节点）或 `show_image:=false` | 重启节点；核对 launch 参数 |
| UVC 下 `dx` 与实际距离差一个比例 | 用了按 `hfov_deg` 估算的内参 | 棋盘格标定后填 `camera_fx/fy/cx/cy`，见 11.1 |
| `参数 T_B_C 无效: ... 不是正交旋转矩阵` | 矩阵填成平移/缩放，或抄漏数字 | 对照 11.2 检查 16 个数与最后一行 |
| `旋转部分行列式不为 +1` | 轴方向含镜像（左右手系混了） | 检查 `y` 轴方向；用示教代替手推 |
| `目标相对当前机体倾角异常` | `T_tag_B_goal` 的旋转写反/写转置 | 用 `~/record_goal` 重新示教 |
| 越走越远 | `T_B_C` 或 `T_tag_B_goal` 转置写反 | 先看 `~/goal_pose_body` 的 dx 符号：目标在前方应为正 |
| 发了命令机器人不动 | `mode` 为 0 / 底层 `rl_sar` 没启动 / 遥控器仍占控制 | 检查 `cmd_mode=2`，`ros2 topic echo /robot_move_cmd` |
| `ros2 topic hz /robot_move_cmd` 无数据或不是 50 Hz | 未激活（`auto_start: false`）、`cmd_output: none`、或 `control_rate` 被改 | 先 `~/start`；核对 `cmd_output` 与 `control_rate: 50.0` |
| 机器人被别的节点抢着动 | 同时跑了 `obstacle_game` / `remote_node`，它们也发 `robot_move_cmd` | 关掉，测试期间只留 `rl_real_atdog2` + `tag_approach` |
| 一直在原地转 | `yaw_align_band` 太小或 `yaw` 误差符号反 | 适当放大 band；核对 `T_tag_B_goal` 旋转 |
| 到位后还微微动 | `pos_tolerance` 太严 / 电机零漂 | 放宽阈值，或到位后调 `~/stop` |
| 一遮住 tag 就冲出去 | 不会：超时立即零速 | 确认 `lost_timeout` 不要设太大（默认 0.5 s） |

### 11.9 测试通过后怎么接回完整链路

测试阶段把常数位姿塞在矩阵里；转正式链路时把它们「拆开」即可：

| 测试矩阵 | 完整链路中的对应物 |
| --- | --- |
| `T_B_C`（相机外参） | URDF / `static_transform_publisher` 的 `base_link -> camera_color_optical_frame`，见 4.2 |
| `T_tag_B_goal`（目标站位） | `config/localizer.yaml` 的 `tag_poses`（tag 在 map 中的位姿），见 4.3 |
| `T_B_tag` 的换算 | `lookupTransform(lidar_frame, camera_frame)` 由 TF 树给出，见第 3 节 |

之后回到第 6 节启动 `full_localization.launch.py`，让 `/initialpose` 交给 AMCL，
由激光扫描与地图匹配继续精修位姿。

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
