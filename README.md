# ATDog simulation and hardware deployment

This workspace supports ATDog, ATDog2 and ATDog3 with MuJoCo simulation, ROS2 hardware entry points, state machines and action switching. TorchScript/ONNX inference, remote control, AprilTag integration, logging, plotting and helper tools are retained.

See [build and cleanup notes](ATDOG_CLEANUP.md) for current commands, action coverage and validation.

[中文](README_CN.md)

## Preparation

Clone the repository

```bash
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
```

To update

```bash
git pull
git submodule update --init --recursive --recommend-shallow --progress
```

## Dependency

Install the required packages:

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev

# macOS
brew install boost yaml-cpp tbb libomp pkg-config glfw
```


If you need to use ROS, install the following dependency packages:

```bash

# ros2-foxy (Ubuntu20.04) / ros2-humble (Ubuntu22.04)
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joy
```

## Compilation

Execute the following script in the project root directory to compile the entire project:

```bash
./build.sh
```

To compile specific packages individually, you can append the package names:

```bash
./build.sh package1 package2
```

To clean the build, use the following command. This will remove all compiled outputs and created symbolic links:

```bash
./build.sh -c  # or ./build.sh --clean
```

If simulation is not needed and you only want to run on the robot, you can compile using CMake while disabling ROS (the compiled executables will be in `cmake_build/bin` and libraries in `cmake_build/lib`):

```bash
./build.sh -m  # or ./build.sh --cmake
```

To use the Mujoco simulator

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

For detailed usage instructions, you can check them via `./build.sh -h`:

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

## Running

In the following text, **\<ROBOT\>/\<CONFIG\>** is used to represent different environments, such as `atdog2/robot_lab` and `atdog3/robot_lab`.

Before running, copy the trained pt model file to `rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`, and configure the parameters in `<ROBOT>/<CONFIG>/config.yaml` and `<ROBOT>/base.yaml`.

### Simulation

#### Mujoco

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco atdog2 scene
```

#### Docker

Docker support is available for simulation and deployment. See [docker/README.md](docker/README.md) for details.

```bash
# Start container
xhost +local:docker  # Enable X11 on host
cd docker && docker compose up -d
docker compose exec rl_sar bash

# Inside container - MuJoCo
./cmake_build/bin/rl_sim_mujoco atdog2 scene


# Inside container - Real robot
ros2 run rl_sar rl_real_atdog2
```

### Control with Mobile Web (Experimental)

Install dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server

# If you are using a ROS2 version other than Humble, Jazz, and Rolling, you need to build `web_video_server` from source
cd <your_ros2_workspace>/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd <your_ros2_workspace>
colcon build --packages-select web_video_server
```

Run rosbridge and web_video_server in robot

```bash
# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

Visit [http://robot.robotsfan.com/](http://robot.robotsfan.com/), fill in the IP address and port, check the settings page in the upper right corner, then connect to the robot. After entering the control page, turn the screen horizontally and click the full screen button in the upper left corner, Then you can control the robot using your phone's browser!

### Control with Gamepad or Keyboard

|Gamepad Control|Keyboard Control|Description|
|---|---|---|
|**Basic**|||
|A|Num0|Move the robot from its initial program pose to the `default_dof_pos` defined in `base.yaml` using position control interpolation|
|B|Num9|Move the robot from its current position to the initial program pose using position control interpolation|
|X|N|Toggle navigation mode (disables velocity commands, receives `cmd_vel` topic)|
|Y|N/A|N/A|
|**Simulation**|||
|**Motor**|||
|LB+A|M|N/A (Recommended for motor enable)|
|LB+B|K|N/A (Recommended for motor disable)|
|LB+X|P|N/A Motor passive mode (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (Recommended for emergency stop)|
|**Skill**|||
|RB+DPadUp|Num1|Basic Locomotion|
|RB+DPadDown|Num2|Skill 2|
|RB+DPadLeft|Num3|Skill 3|
|RB+DPadRight|Num4|Skill 4|
|LB+DPadUp|Num5|Skill 5|
|LB+DPadDown|Num6|Skill 6|
|LB+DPadLeft|Num7|Skill 7|
|LB+DPadRight|Num8|Skill 8|
|**Movement**|||
|LY Axis|W/S|Forward/Backward movement (X-axis)|
|LX Axis|A/D|Left/Right movement (Y-axis)|
|RX Axis|Q/E|Yaw rotation|
|N/A (Release joystick)|Space|Reset all control commands to zero|

### Real Robot

```bash
source install/setup.bash
ros2 run rl_sar rl_real_atdog2
# Alternatives: rl_real_atdog, rl_real_atdog3
```

### Train the actuator network

Take ATDog2 as an example below

1. Uncomment `#define CSV_LOGGER` in the top of `rl_real_atdog2.hpp`. You can also modify the corresponding part in the simulation program to collect simulation data for testing the training process.
2. Run the control program, and the program will log all data in `src/rl_sar/policy/<ROBOT>/motor.csv`.
3. Stop the control program and start training the actuator network. Note that `rl_sar/src/rl_sar/policy/` is omitted before the following paths.
    ```bash
    ros2 run rl_sar actuator_net.py --mode train --data atdog2/motor.csv --output atdog2/motor.pt
    ```
4. Verify the trained actuator network.
    ```bash
    ros2 run rl_sar actuator_net.py --mode play --data atdog2/motor.csv --output atdog2/motor.pt
    ```

## Add Your Robot

The following uses **\<ROBOT\>/\<CONFIG\>** to represent your robot environment. You only need to create or modify the following files, and the names must exactly match those shown below. (You can refer to the corresponding files in atdog2 as examples.)

```yaml
# your robot description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/urdf/<ROBOT>.urdf

# your policy
policy/<ROBOT>/base.yaml  # This file must follow the physical robot's joint order
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # for libtorch, note that exporting JIT is required
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # for onnxruntime

# fsm for robot
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# your real robot code
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # You can customize the forward() function as needed to adapt to your policy
```

## Contributing

Wholeheartedly welcome contributions from the community to make this framework mature and useful for everyone. These may happen as bug reports, feature requests, or code contributions.

[List of contributors](CONTRIBUTORS.md)

## Citation

Please cite the following if you use this code or parts of it:

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## Acknowledgements

The project uses some code from the following open-source code repositories:

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
