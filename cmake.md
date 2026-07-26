# CMake 知识总结（基于 AT_sim2real 项目）

本文档从 `AT_sim2real` 机器人强化学习项目中提炼 CMake 实际用法，按知识点分章节讲解，每节附项目中的真实代码示例和文件位置。

---

## 目录

1. [基本结构](#1-基本结构)
2. [C++ 标准设置](#2-c-标准设置)
3. [编译选项与警告](#3-编译选项与警告)
4. [构建类型与选项缓存](#4-构建类型与选项缓存)
5. [查找依赖 find_package](#5-查找依赖-find_package)
6. [pkg-config 集成](#6-pkg-config-集成)
7. [ROS1 catkin 与 ROS2 ament 双支持](#7-ros1-catkin-与-ros2-ament-双支持)
8. [添加目标 add_executable / add_library](#8-添加目标-add_executable--add_library)
9. [包含目录 target_include_directories](#9-包含目录-target_include_directories)
10. [链接库 target_link_libraries](#10-链接库-target_link_libraries)
11. [ament_target_dependencies](#11-ament_target_dependencies)
12. [安装规则 install](#12-安装规则-install)
13. [导入库 IMPORTED](#13-导入库-imported)
14. [add_subdirectory 子项目](#14-add_subdirectory-子项目)
15. [库别名 ALIAS](#15-库别名-alias)
16. [平台与架构检测](#16-平台与架构检测)
17. [环境变量检测 ROS_DISTRO](#17-环境变量检测-ros_distro)
18. [条件编译定义](#18-条件编译定义)
19. [RPATH 配置](#19-rpath-配置)
20. [消息接口生成](#20-消息接口生成)
21. [测试与代码检查](#21-测试与代码检查)
22. [导出目标与包配置文件](#22-导出目标与包配置文件)
23. [插件导出](#23-插件导出)
24. [文件操作](#24-文件操作)
25. [生成器表达式](#25-生成器表达式)
26. [编译命令导出](#26-编译命令导出)
27. [常用变量速查表](#27-常用变量速查表)

---

## 1. 基本结构

每个 `CMakeLists.txt` 文件最开头必须有这两行：

```cmake
cmake_minimum_required(VERSION 3.8)
project(task_game)
```

- `cmake_minimum_required`：指定最低 CMake 版本，保证语法兼容性。
- `project()`：声明项目名，可选 `VERSION`、`LANGUAGES` 等参数。

**带版本和语言的写法**（`src/rl_sar/CMakeLists.txt:1-2`）：
```cmake
cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)
```

**带版本号的写法**（`src/robot_msgs/CMakeLists.txt:1-2`）：
```cmake
cmake_minimum_required(VERSION 3.5)
project(robot_msgs VERSION 3.0.0)
```

| 参数 | 说明 |
|------|------|
| `VERSION x.y.z` | 项目版本号，赋值给 `PROJECT_VERSION` |
| `LANGUAGES CXX` | 启用 C++ 语言支持（还可 `C`, `Fortran` 等） |

---

## 2. C++ 标准设置

项目中有两种设置 C++ 标准的方式：

### 方式一：全局变量设置（`src/task_game/CMakeLists.txt:8-10`）

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

| 变量 | 作用 |
|------|------|
| `CMAKE_CXX_STANDARD` | C++ 标准版本（11/14/17/20/23） |
| `CMAKE_CXX_STANDARD_REQUIRED ON` | 强制要求该标准，不允许降级 |
| `CMAKE_CXX_EXTENSIONS OFF` | 关闭编译器扩展（如 GNU 扩展），使用纯标准 C++ |

### 方式二：目标级特性设置（`src/remote_node/CMakeLists.txt:83`）

```cmake
target_compile_features(remote_node PUBLIC cxx_std_17)
```

这种方式只对特定目标生效，是更现代的写法。

---

## 3. 编译选项与警告

### 添加编译警告（几乎所有包通用模式）

```cmake
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

- `CMAKE_COMPILER_IS_GNUCXX`：检测是否为 GCC 编译器。
- `CMAKE_CXX_COMPILER_ID MATCHES "Clang"`：检测是否为 Clang。
- `add_compile_options()`：对当前目录及子目录的所有目标添加编译选项。

### Debug 模式下添加调试信息（`src/rl_sar/CMakeLists.txt:35-41`）

```cmake
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()
```

- `string(TOUPPER ...)`：将字符串转大写，方便比较。

---

## 4. 构建类型与选项缓存

### CACHE 变量（`src/rl_sar/CMakeLists.txt:13`）

```cmake
set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
set(USE_MUJOCO OFF CACHE BOOL "Build with MuJoCo simulator support")
```

`CACHE` 关键字将变量存入 CMake 缓存，用户可通过 `-DUSE_MUJOCO=ON` 在命令行覆盖。

格式：`set(VAR value CACHE TYPE "description" [FORCE])`

| TYPE | 说明 |
|------|------|
| `BOOL` | 布尔值 ON/OFF |
| `PATH` | 目录路径 |
| `FILEPATH` | 文件路径 |
| `STRING` | 字符串 |

### option 命令（`src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/CMakeLists.txt:16-17`）

```cmake
option(ATDOG_BUILD_IMU_DRIVER "Build the IMU driver shared library" ON)
option(ATDOG_BUILD_LEG_DRIVER "Build the leg driver shared library" ON)
```

`option()` 等价于 `set(... CACHE BOOL ...)`，语义更清晰，适合布尔开关。

---

## 5. 查找依赖 find_package

`find_package` 是 CMake 中查找外部依赖的核心命令。

### 基本用法（`src/task_game/CMakeLists.txt:15-26`）

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
```

- `REQUIRED`：找不到则报错终止。
- 不加 `REQUIRED` 则静默跳过，可通过检查 `XXX_FOUND` 判断。

### QUIET 安静模式与回退逻辑（`src/remote_node/CMakeLists.txt:19-21`）

```cmake
find_package(serial QUIET)

if(NOT serial_FOUND)
  message(WARNING "ROS package 'serial' was not found. Falling back to bundled sources...")
  # ... 使用内置源码编译
endif()
```

### 指定搜索路径（`src/arm_calc/CMakeLists.txt:30`）

```cmake
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)
```

### 查找系统库（`src/rl_sar/CMakeLists.txt:236-238`）

```cmake
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
```

`COMPONENTS` 用于指定需要的组件，如 Python3 的 `Interpreter`（解释器）和 `Development`（开发头文件和库）。

### 可选组件查找（`src/rl_sar/CMakeLists.txt:514-519`）

```cmake
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
```

---

## 6. pkg-config 集成

对于不提供 CMake 配置文件的库，可用 `pkg-config` 查找。

**示例**（`src/robot_driver/CMakeLists.txt:17-18`）：

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
```

使用时：
```cmake
target_include_directories(robot_driver PUBLIC ${LIBUSB_INCLUDE_DIRS})
target_link_directories(robot_driver PUBLIC ${LIBUSB_LIBRARY_DIRS})
target_link_libraries(robot_driver ${LIBUSB_LIBRARIES})
```

**更现代的 IMPORTED_TARGET 写法**（`src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/CMakeLists.txt:21-22`）：

```cmake
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
# ...
target_link_libraries(atdog_leg_driver PUBLIC PkgConfig::LIBUSB)
```

`IMPORTED_TARGET` 会自动创建 `PkgConfig::LIBUSB` 导入目标，自动包含头文件路径和库路径。

---

## 7. ROS1 catkin 与 ROS2 ament 双支持

本项目大量使用 `$ENV{ROS_DISTRO}` 环境变量来同时支持 ROS1 和 ROS2。

### 消息包的典型双支持结构（`src/robot_msgs/CMakeLists.txt`）

```cmake
if($ENV{ROS_DISTRO} MATCHES "noetic")
  # === ROS1: catkin 方式 ===
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )
  add_message_files(FILES MotorCommand.msg MotorState.msg ...)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
  # === ROS2: ament 方式 ===
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
    "msg/Remote.msg"
    DEPENDENCIES builtin_interfaces std_msgs
  )
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
```

### 控制器插件的双支持（`src/robot_joint_controller/CMakeLists.txt`）

ROS1 用 `catkin_package()` + `include_directories` + `target_link_libraries`；
ROS2 用 `find_package(ament_cmake)` + `ament_target_dependencies` + `ament_package()`。

### ROS2 内版本区分（`src/robot_joint_controller/CMakeLists.txt:32-38`）

```cmake
if($ENV{ROS_DISTRO} MATCHES "foxy")
    add_definitions(-DROS_DISTRO_FOXY)
elseif($ENV{ROS_DISTRO} MATCHES "humble")
    add_definitions(-DROS_DISTRO_HUMBLE)
else()
    add_definitions(-DROS_DISTRO_UNKNOWN)
endif()
```

这样在 C++ 代码中可以通过 `#ifdef ROS_DISTRO_HUMBLE` 区分版本。

---

## 8. 添加目标 add_executable / add_library

### 可执行文件（`src/task_game/CMakeLists.txt:41-52`）

```cmake
add_executable(robot_control
  src/main.cpp
  src/core/robot.cpp
  src/core/pilot.cpp
  src/core/behavior_tree.cpp
  src/nodes/generate_plan.cpp
  src/nodes/catch_box.cpp
  src/nodes/place_box.cpp
)
```

### 静态库

```cmake
add_library(remote_node_serial STATIC
  "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/serial.cc"
)
```

### 共享库（`src/robot_joint_controller/CMakeLists.txt:54-57`）

```cmake
add_library(${PROJECT_NAME} SHARED
    ros2/src/robot_joint_controller.cpp
    ros2/src/robot_joint_controller_group.cpp
)
```

### 接口库（纯头文件库，`src/rl_sar/CMakeLists.txt:378`）

```cmake
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
    library/thirdparty/robot_sdk/zhinao/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)
```

`INTERFACE` 库不产生编译产物，只传递编译/链接信息。

| 类型 | 关键字 | 说明 |
|------|--------|------|
| 静态库 | `STATIC` | 编译为 `.a`/`.lib` |
| 共享库 | `SHARED` | 编译为 `.so`/`.dylib`/`.dll` |
| 接口库 | `INTERFACE` | 无编译产物，仅传递属性 |
| 模块库 | `MODULE` | 运行时 `dlopen` 加载的插件 |

---

## 9. 包含目录 target_include_directories

### 现代写法（推荐，`src/task_game/CMakeLists.txt:63-66`）

```cmake
target_include_directories(robot_control
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
```

| 可见性 | 含义 |
|--------|------|
| `PRIVATE` | 仅自身使用 |
| `PUBLIC` | 自身和依赖者都使用 |
| `INTERFACE` | 仅依赖者使用（自身不使用） |

### 带 INSTALL_INTERFACE 的完整写法（`src/remote_node/CMakeLists.txt:89-93`）

```cmake
target_include_directories(remote_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

- `$<BUILD_INTERFACE:...>`：构建树中使用的路径。
- `$<INSTALL_INTERFACE:...>`：安装后使用的相对路径。

### 旧式全局写法（`src/rl_sar/CMakeLists.txt:433-445`）

```cmake
include_directories(
    include
    library/core/rl_sdk
    library/core/observation_buffer
)
```

`include_directories()` 是全局的，会影响所有目标，不推荐在新代码中使用。

---

## 10. 链接库 target_link_libraries

### 基本用法（`src/task_game/CMakeLists.txt:80`）

```cmake
target_link_libraries(robot_control Eigen3::Eigen yaml-cpp)
```

### 链接多个库（`src/rl_sar/CMakeLists.txt:477-484`）

```cmake
target_link_libraries(rl_sdk PUBLIC
    motion_loader
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
    Eigen3::Eigen
)
```

### 系统线程库（`src/remote_node/CMakeLists.txt:112-114`）

```cmake
target_link_libraries(remote_node
  Threads::Threads
)
```

`Threads::Threads` 是 `find_package(Threads)` 提供的跨平台线程库目标。

### macOS 框架链接（`src/rl_sar/CMakeLists.txt:911-915`）

```cmake
target_link_libraries(rl_sim_mujoco
    "-framework CoreVideo"
    "-framework OpenGL"
    "-framework AppKit"
)
```

---

## 11. ament_target_dependencies

ROS2 特有命令，用于将 ROS 依赖的头文件和库链接到目标。

```cmake
ament_target_dependencies(
  robot_control
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  robot_msgs
)
```

与 `target_link_libraries` 的区别：`ament_target_dependencies` 会自动处理 ROS 包的 include 路径、库和依赖传递，是 ROS2 推荐的方式。

---

## 12. 安装规则 install

### 安装可执行文件（`src/task_game/CMakeLists.txt:103-108`）

```cmake
install(TARGETS
  robot_control
  test_node
  measure_node
  DESTINATION lib/${PROJECT_NAME}
)
```

ROS2 中可执行文件安装到 `lib/<package_name>/` 目录。

### 安装目录（launch、config 等，`src/task_game/CMakeLists.txt:110-114`）

```cmake
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
```

### 安装 Python 脚本（`src/rl_sar/CMakeLists.txt:791-795`）

```cmake
install(PROGRAMS
  scripts/actuator_net.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`PROGRAMS` 会自动设置可执行权限。

### 安装库文件并导出（`src/robot_joint_controller/CMakeLists.txt:93-100`）

```cmake
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
```

| 目的地 | 说明 |
|--------|------|
| `ARCHIVE DESTINATION` | 静态库 `.a` |
| `LIBRARY DESTINATION` | 共享库 `.so` |
| `RUNTIME DESTINATION` | 可执行文件 |
| `INCLUDES DESTINATION` | 头文件（仅记录，不实际安装） |

### 按通配符安装文件（`src/rl_sar/CMakeLists.txt:349-353`）

```cmake
file(GLOB GLOB_UNITREE_LEGGED_SDK ".../libunitree_legged_sdk_amd64.so")
install(FILES ${GLOB_UNITREE_LEGGED_SDK} DESTINATION lib/)
```

### 使用 GNUInstallDirs 标准路径（`src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/CMakeLists.txt:130-135`）

```cmake
include(GNUInstallDirs)
install(TARGETS ${ATDOG_EXPORT_TARGETS}
  EXPORT atdog_robot_driver_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
```

| 变量 | 典型值 |
|------|--------|
| `CMAKE_INSTALL_LIBDIR` | `lib` 或 `lib64` |
| `CMAKE_INSTALL_BINDIR` | `bin` |
| `CMAKE_INSTALL_INCLUDEDIR` | `include` |

---

## 13. 导入库 IMPORTED

对于预编译的第三方 `.so` 文件，使用 `IMPORTED` 导入。

**ONNX Runtime 导入**（`src/rl_sar/CMakeLists.txt:155-163`）：

```cmake
add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime_prebuilt PROPERTIES
    IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
)
```

| 属性 | 说明 |
|------|------|
| `IMPORTED_LOCATION` | 库文件的实际路径 |
| `INTERFACE_INCLUDE_DIRECTORIES` | 头文件目录（传递给使用者） |
| `IMPORTED_NO_SONAME TRUE` | 不使用 SONAME |
| `GLOBAL` | 全局可见（默认仅当前目录可见） |

**find_library 查找库路径**（`src/rl_sar/CMakeLists.txt:147-151`）：

```cmake
find_library(ONNX_RUNTIME_LIB_PATH
    NAMES libonnxruntime.so onnxruntime.so
    PATHS "${ONNX_RUNTIME_DIR}/lib"
    NO_DEFAULT_PATH
)
```

---

## 14. add_subdirectory 子项目

将第三方库源码作为子项目编译进当前项目。

**编译 vendored LCM**（`src/rl_sar/CMakeLists.txt:324`）：

```cmake
add_subdirectory(library/thirdparty/lcm EXCLUDE_FROM_ALL)
```

`EXCLUDE_FROM_ALL` 表示子项目中的目标不会被默认构建，除非被主项目依赖。

**编译 unitree_sdk2**（`src/rl_sar/CMakeLists.txt:360`）：

```cmake
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build" FORCE)
add_subdirectory(library/thirdparty/robot_sdk/unitree/unitree_sdk2)
```

在 `add_subdirectory` 前用 `CACHE ... FORCE` 关闭子项目的示例构建。

---

## 15. 库别名 ALIAS

为库创建命名空间别名，方便引用。

```cmake
add_library(atdog_serial STATIC ${ATDOG_SERIAL_SOURCES})
add_library(atdog::serial ALIAS atdog_serial)
```

之后可以用 `atdog::serial` 来链接：

```cmake
target_link_libraries(atdog_imu_driver PUBLIC atdog::serial)
```

---

## 16. 平台与架构检测

### 操作系统检测（`src/rl_sar/CMakeLists.txt:54-60`）

```cmake
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND RL_SAR_BUILD_RPATHS "\$ORIGIN" "\$ORIGIN/../lib")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    list(APPEND RL_SAR_BUILD_RPATHS "@loader_path" "@loader_path/../lib")
endif()
```

| `CMAKE_SYSTEM_NAME` | 平台 |
|----------------------|------|
| `Linux` | Linux |
| `Darwin` | macOS |
| `Windows` | Windows |

### CPU 架构检测（`src/rl_sar/CMakeLists.txt:297-307`）

```cmake
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(ARCH_DIR "aarch64")
    set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(ARCH_DIR "x86_64")
    set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
```

### macOS 特定库查找（`src/remote_node/CMakeLists.txt:47-57`）

```cmake
if(APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
  target_link_libraries(remote_node_serial PUBLIC
    ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
  target_link_libraries(remote_node_serial PUBLIC rt pthread)
elseif(WIN32)
  target_link_libraries(remote_node_serial PUBLIC setupapi)
endif()
```

`APPLE`、`UNIX`、`WIN32` 是 CMake 内置布尔变量。

---

## 17. 环境变量检测 ROS_DISTRO

通过 `$ENV{变量名}` 读取系统环境变量：

```cmake
if($ENV{ROS_DISTRO} MATCHES "noetic")
    # ROS1 代码
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    # ROS2 代码
endif()
```

`MATCHES` 支持正则，`"foxy|humble"` 表示匹配 foxy 或 humble。

---

## 18. 条件编译定义

### add_compile_definitions（现代写法，推荐）

```cmake
add_compile_definitions(USE_ROS2)
add_compile_definitions(USE_MUJOCO)
add_compile_definitions(USE_ONNX)
```

### add_definitions（旧式写法）

```cmake
add_definitions(-DROS_DISTRO_FOXY)
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
```

### target_compile_definitions（目标级，`src/rl_sar/CMakeLists.txt:451`）

```cmake
target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
```

`PRIVATE` 表示仅该目标内部可见，不传递给依赖者。

> 注意：定义字符串值时需要用引号包裹，如 `-DPOLICY_DIR="${路径}"`。

---

## 19. RPATH 配置

RPATH（Run-time Path）是 Linux/macOS 下可执行文件查找共享库的机制。

**配置构建和安装时的 RPATH**（`src/rl_sar/CMakeLists.txt:52-64`）：

```cmake
# Linux 使用 $ORIGIN 相对路径
list(APPEND RL_SAR_BUILD_RPATHS "\$ORIGIN" "\$ORIGIN/../lib" "${CMAKE_BINARY_DIR}/lib")
list(APPEND RL_SAR_INSTALL_RPATHS "\$ORIGIN" "\$ORIGIN/..")

# macOS 使用 @loader_path 相对路径
list(APPEND RL_SAR_BUILD_RPATHS "@loader_path" "@loader_path/../lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
list(APPEND CMAKE_BUILD_RPATH ${RL_SAR_BUILD_RPATHS})
list(APPEND CMAKE_INSTALL_RPATH ${RL_SAR_INSTALL_RPATHS})
```

| 变量 | 说明 |
|------|------|
| `CMAKE_BUILD_RPATH` | 构建时使用的 RPATH |
| `CMAKE_INSTALL_RPATH` | 安装时使用的 RPATH |
| `CMAKE_SKIP_BUILD_RPATH` | 是否跳过构建 RPATH |
| `CMAKE_BUILD_WITH_INSTALL_RPATH` | 构建时是否使用安装 RPATH |

**为特定目标设置 RPATH**（`src/rl_sar/CMakeLists.txt:621-623`）：

```cmake
set_target_properties(rl_real_go2 PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/.../lib/${ARCH_DIR}"
)
```

**Linux 下禁用新 dtags**（让 RPATH 生效而非 RUNPATH，`src/rl_sar/CMakeLists.txt:226-228`）：

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
```

---

## 20. 消息接口生成

### ROS1 catkin 消息生成（`src/robot_msgs/CMakeLists.txt:13-33`）

```cmake
add_message_files(
  FILES
  MotorCommand.msg
  MotorState.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
```

### ROS2 rosidl 消息生成（`src/robot_msgs/CMakeLists.txt:53-72`）

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/Remote.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```

---

## 21. 测试与代码检查

### BUILD_TESTING 守卫（`src/task_game/CMakeLists.txt:29-39`）

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过 copyright 检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
```

- `BUILD_TESTING` 是 CMake 内置变量，默认 ON，`-DBUILD_TESTING=OFF` 可关闭。
- `set(ament_cmake_xxx_FOUND TRUE)` 用于跳过特定 linter。
- `ament_lint_auto_find_test_dependencies()` 自动查找 `package.xml` 中声明的测试依赖。

### pluginlib_export_plugin_description_file（`src/robot_joint_controller/CMakeLists.txt:74`）

```cmake
pluginlib_export_plugin_description_file(controller_interface ros2/robot_joint_controller_plugins.xml)
```

这会将插件描述文件安装到正确位置，让 `pluginlib` 能在运行时发现插件。

---

## 22. 导出目标与包配置文件

当你的库需要被其他 CMake 项目通过 `find_package` 找到时，需要导出目标和生成配置文件。

**完整示例**（`src/rl_sar/library/thirdparty/robot_sdk/atdog/robot_driver/CMakeLists.txt`）：

```cmake
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# 1. 安装目标并导出
install(TARGETS ${ATDOG_EXPORT_TARGETS}
  EXPORT atdog_robot_driver_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 2. 生成配置文件（从模板）
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/atdog_robot_driverConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)

# 3. 生成版本文件
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# 4. 安装导出的 targets（带命名空间）
install(EXPORT atdog_robot_driver_targets
  FILE atdog_robot_driverTargets.cmake
  NAMESPACE atdog::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)

# 5. 安装配置文件
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)
```

之后其他项目即可：
```cmake
find_package(atdog_robot_driver REQUIRED)
target_link_libraries(my_app PRIVATE atdog::imu_driver atdog::leg_driver)
```

### ROS2 中的 ament 导出

```cmake
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp robot_msgs geometry_msgs)
ament_package()
```

---

## 23. 插件导出

ROS2 控制器插件需要用 `pluginlib` 导出：

```cmake
pluginlib_export_plugin_description_file(controller_interface ros2/robot_joint_controller_plugins.xml)
```

第一个参数是插件基类所在的包，第二个参数是插件描述 XML 文件。

---

## 24. 文件操作

### file(GLOB ...) 收集源文件（`src/rl_sar/CMakeLists.txt:872`）

```cmake
file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
```

### file(READ ...) 读取文件内容（`src/rl_sar/CMakeLists.txt:125-127`）

```cmake
file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
```

### file(MAKE_DIRECTORY ...) 创建目录（`src/rl_sar/CMakeLists.txt:824`）

```cmake
file(MAKE_DIRECTORY "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
```

### file(COPY ...) 复制文件（`src/rl_sar/CMakeLists.txt:826`）

```cmake
file(GLOB MUJOCO_HEADERS "${MUJOCO_DIR}/Headers/*.h")
file(COPY ${MUJOCO_HEADERS} DESTINATION "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
```

### file(EXISTS ...) 检测文件/目录是否存在

```cmake
if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
endif()
```

---

## 25. 生成器表达式

生成器表达式在生成构建系统时求值，语法为 `$<...>`。

### BUILD_INTERFACE / INSTALL_INTERFACE

```cmake
target_include_directories(robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
```

- `$<BUILD_INTERFACE:...>`：仅在构建时生效。
- `$<INSTALL_INTERFACE:...>`：仅在安装后被其他项目使用时生效。

### 其他常用生成器表达式

| 表达式 | 说明 |
|--------|------|
| `$<TARGET_OBJECTS:target>` | 获取目标的对象文件 |
| `$<CONFIG:Debug>` | 当前配置是否为 Debug |
| `$<PLATFORM_ID:Linux>` | 当前平台是否为 Linux |
| `$<BOOL:value>` | 将值转为布尔 |

---

## 26. 编译命令导出

项目几乎所有 CMakeLists.txt 中都有：

```cmake
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

这会在构建目录下生成 `compile_commands.json`，供 IDE/语言服务器（如 clangd、VSCode）使用，实现代码补全、跳转、错误提示等功能。

---

## 27. 常用变量速查表

### 项目相关

| 变量 | 说明 |
|------|------|
| `PROJECT_NAME` | `project()` 中定义的项目名 |
| `PROJECT_VERSION` | `project(VERSION x.y.z)` 中的版本 |
| `CMAKE_CURRENT_SOURCE_DIR` | 当前 CMakeLists.txt 所在目录 |
| `CMAKE_CURRENT_BINARY_DIR` | 当前构建目录 |
| `CMAKE_BINARY_DIR` | 顶层构建目录 |
| `CMAKE_SOURCE_DIR` | 顶层源码目录 |

### 编译相关

| 变量 | 说明 |
|------|------|
| `CMAKE_CXX_STANDARD` | C++ 标准版本 |
| `CMAKE_BUILD_TYPE` | 构建类型（Debug/Release/RelWithDebInfo） |
| `CMAKE_CXX_COMPILER_ID` | 编译器标识（GNU/Clang/MSVC） |
| `CMAKE_CXX_COMPILER_VERSION` | 编译器版本 |
| `CMAKE_COMPILER_IS_GNUCXX` | 是否为 GCC |
| `CMAKE_POSITION_INDEPENDENT_CODE` | 生成位置无关代码（-fPIC） |

### 系统相关

| 变量 | 说明 |
|------|------|
| `CMAKE_SYSTEM_NAME` | 操作系统名（Linux/Darwin/Windows） |
| `CMAKE_SYSTEM_PROCESSOR` | CPU 架构（x86_64/aarch64） |
| `APPLE` | 是否为 macOS |
| `UNIX` | 是否为 Unix 系统（含 macOS） |
| `WIN32` | 是否为 Windows |

### 安装相关

| 变量 | 说明 |
|------|------|
| `CMAKE_INSTALL_PREFIX` | 安装前缀（默认 `/usr/local`） |
| `CMAKE_INSTALL_LIBDIR` | 库安装目录 |
| `CMAKE_INSTALL_BINDIR` | 可执行文件安装目录 |
| `CMAKE_INSTALL_INCLUDEDIR` | 头文件安装目录 |

### ROS2 相关

| 变量/路径 | 说明 |
|-----------|------|
| `lib/${PROJECT_NAME}` | ROS2 可执行文件安装位置 |
| `share/${PROJECT_NAME}` | ROS2 数据文件安装位置 |

---

## 附录：项目 CMakeLists.txt 文件索引

| 文件 | 类型 | 特点 |
|------|------|------|
| `src/rl_sar/CMakeLists.txt` | 核心框架 | 最复杂，含 ONNX/Torch/MuJoCo/RPATH/多平台 |
| `src/robot_msgs/CMakeLists.txt` | 消息定义 | ROS1/ROS2 双支持消息生成 |
| `src/robot_joint_controller/CMakeLists.txt` | 控制器插件 | pluginlib 导出 + ROS1/ROS2 双支持 |
| `src/task_game/CMakeLists.txt` | 应用节点 | 标准 ament 可执行文件 |
| `src/obstacle_game/CMakeLists.txt` | 应用节点 | 标准 ament 可执行文件 |
| `src/remote_node/CMakeLists.txt` | 串口节点 | 依赖回退 + 多平台串口 |
| `src/robot_driver/CMakeLists.txt` | USB 驱动 | pkg-config + libusb |
| `src/arm_calc/CMakeLists.txt` | 机械臂计算 | KDL + Eigen |
| `src/keyboard/CMakeLists.txt` | 键盘节点 | 最简单的 ament 可执行文件 |
| `src/launch_pack/CMakeLists.txt` | 纯数据包 | 仅安装 launch/config/rviz |
| `src/rl_sar_zoo/*/CMakeLists.txt` | 描述资源包 | 仅安装 meshes/urdf/launch |
| `src/rl_sar/.../atdog/robot_driver/CMakeLists.txt` | SDK 库 | 完整导出目标 + 配置文件 |
