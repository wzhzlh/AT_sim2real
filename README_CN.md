# ATDog 仿真与实机部署

本工作区保留 ATDog、ATDog2、ATDog3 的 MuJoCo 仿真、ROS2 实机控制、状态机、动作及切换。支持 TorchScript 与 ONNX 推理；保留遥控、AprilTag 接口、日志、绘图和辅助工具。

详细构建命令、动作范围与验证记录见 [ATDog 清理与构建说明](ATDOG_CLEANUP.md)。

[English](README.md)

## 准备

拉取仓库

```bash
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
```

如需更新

```bash
git pull
git submodule update --init --recursive --recommend-shallow --progress
```

## 依赖

安装必要的依赖：

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev

# macOS
brew install boost yaml-cpp tbb libomp pkg-config glfw
```


如果需要使用ROS，安装下列依赖包：

```bash

# ros2-foxy (Ubuntu20.04) / ros2-humble (Ubuntu22.04)
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joy
```

## 编译

在项目根目录中执行下面的脚本编译整个项目

```bash
./build.sh
```

若想单独编译某几个包，可以在后面加上包名

```bash
./build.sh package1 package2
```

若想删除构建，可以使用下列命令，此命令会删除所有编译产物和创建的软链接

```bash
./build.sh -c  # or ./build.sh --clean
```

如果不需要仿真，只在机器人上运行，可以使用CMake进行编译，同时禁用ROS（编译生成的可执行文件在`cmake_build/bin`中，库在`cmake_build/lib`中）

```bash
./build.sh -m  # or ./build.sh --cmake
```

若想使用Mujoco仿真器

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build common libraries using CMake
  -mj,--mujoco   Build with MuJoCo simulator support (CMake only)"
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build common libraries using CMake
  ./build.sh -mj                # Build with CMake and MuJoCo simulator support
```

> [!TIP]

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `atdog2/robot_lab` 、 `atdog3/robot_lab`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

#### Mujoco

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco atdog2 scene
```

#### Docker

Docker支持仿真和部署。详见 [docker/README.md](docker/README.md)。

```bash
# 启动容器
xhost +local:docker  # 在主机上启用X11
cd docker && docker compose up -d
docker compose exec rl_sar bash

# 容器内 - MuJoCo仿真
./cmake_build/bin/rl_sim_mujoco atdog2 scene


# 容器内 - 真实机器人
ros2 run rl_sar rl_real_atdog2
```

### 使用手机网页控制 (实验性)

安装依赖

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server

# 如果您使用的ROS2版本不是Humble、Jazz或Rolling，需要从源码编译 `web_video_server`
cd <your_ros2_workspace>/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd <your_ros2_workspace>
colcon build --packages-select web_video_server
```

在机器人上运行 rosbridge 和 web_video_server

```bash
# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

访问 [http://robot.robotsfan.com/](http://robot.robotsfan.com/)， 填写IP地址和端口，检查右上角的设置界面，然后连接机器人。进入控制页面后，将屏幕水平放置，点击左上角的全屏按钮，即可使用手机浏览器控制机器人！

### 使用手柄或键盘控制

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽速度命令，接收`cmd_vel`话题)|
|Y|N/A|N/A|
|**仿真**|||
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

```bash
source install/setup.bash
ros2 run rl_sar rl_real_atdog2
# 其他入口：rl_real_atdog、rl_real_atdog3
```

### 训练执行器网络

下面拿ATDog2举例

1. 取消注释`rl_real_atdog2.hpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会记录所有数据到`src/rl_sar/policy/<ROBOT>/motor.csv`。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/policy/`。
    ```bash
    ros2 run rl_sar actuator_net.py --mode train --data atdog2/motor.csv --output atdog2/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    ros2 run rl_sar actuator_net.py --mode play --data atdog2/motor.csv --output atdog2/motor.pt
    ```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境。你只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考ATDog2对应的文件）

```yaml
# 你的机器人description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/urdf/<ROBOT>.urdf

# 你训练的policy
policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # libtorch使用，注意导出jit
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # onnxruntime使用

# 机器人的fsm
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# 你实物机器人的代码
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [google-deepmind/mujoco-3.2.7](https://github.com/google-deepmind/mujoco)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- [chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
