# Docker Support for rl_sar

Docker environment for simulation and deployment with ROS2 Humble.

## Prerequisites

Before building, initialize git submodules:
```bash
git submodule update --init --recursive
```

## Quick Start

```bash
# Enable X11 forwarding (on host)
xhost +local:docker

# Build and run (from repo root or docker/ directory)
cd docker
docker compose up -d

# To use software rendering (Mesa llvmpipe) instead of GPU:
# docker compose --profile software up -d rl_sar_software

# Check which renderer is used:
# docker exec -it rl_sar glxinfo | grep "OpenGL renderer"
#   GPU mode:      NVIDIA GeForce RTX xxxx
#   Software mode: llvmpipe (LLVM xx.x, 256 bits)

# Rebuild after code changes, if only change policies not codes, no need to rebuild
docker compose up -d --build

# in the docker folder
docker compose exec rl_sar bash
# or anywhere
docker exec -it rl_sar bash
```

## Usage

### MuJoCo Simulation

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example:
./cmake_build/bin/rl_sim_mujoco atdog2 scene
./cmake_build/bin/rl_sim_mujoco atdog3 scene_terrain
```

### Real Robot

```bash
source install/setup.bash
ros2 run rl_sar rl_real_atdog2
# Alternatives: rl_real_atdog, rl_real_atdog3
```

## Controls

| Gamepad | Keyboard | Description |
|---------|----------|-------------|
| A | Num0 | Stand up (move to default pose) |
| B | Num9 | Return to initial pose |
| RB+DPadUp | Num1 | Basic locomotion |
| LY/LX | W/S, A/D | Move forward/backward, left/right |
| RX | Q/E | Yaw rotation |

## Files

| File | Description |
|------|-------------|
| `Dockerfile` | ROS2 Humble + MuJoCo + ATDog hardware |
| `docker-compose.yml` | X11 + gamepad + network support |
| `entrypoint.sh` | Sources ROS2 setup |

## Build Manually

```bash
# From repo root
docker build -f docker/Dockerfile -t rl_sar:humble .
```

## Notes

- Policy files are mounted read-only from `../policy`
- Container runs in privileged mode with host network for robot communication
- Gamepad devices are passed through via `/dev/input`
