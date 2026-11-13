cmake_minimum_required(VERSION 3.8)
project(apriltag_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------- ROS2 依赖 ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)

# ---------------- 第三方依赖 ----------------
# 这三个库来自 ~/qq_down/D435_apriltag/tag.cpp 的原生 C++ 方案：
#   OpenCV        -> cv::solvePnP 求 tag 相对相机位姿
#   librealsense2 -> 读取 D435 彩色图与内参
#   apriltag      -> tag36h11 检测
# 未安装时不让整个工作区的 colcon build 失败：只跳过本节点编译，
# 仍然安装 launch/config/teach.md（缺依赖时仅打印 STATUS，不产生 CMake Warning）。
find_package(OpenCV QUIET)
find_package(realsense2 QUIET)

# AprilTag 用 find_path/find_library 查找，比 pkg-config 稳健：
# Ubuntu libapriltag-dev 的 apriltag.pc 不含 Cflags，用户级安装时拿不到头文件路径；
# 源码安装的某些版本则根本没有 .pc。除系统路径外还支持 /usr/local 与 ~/.local。
set(APRILTAG_PREFIXES /usr/local /usr)
if(DEFINED ENV{HOME})
  list(APPEND APRILTAG_PREFIXES "$ENV{HOME}/.local")
endif()

find_path(APRILTAG_INCLUDE_DIR
  NAMES apriltag/apriltag.h
  PATHS ${APRILTAG_PREFIXES}
  PATH_SUFFIXES include
)
find_library(APRILTAG_LIBRARY
  NAMES apriltag
  PATHS ${APRILTAG_PREFIXES}
  PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu
)

if(APRILTAG_INCLUDE_DIR AND APRILTAG_LIBRARY)
  set(APRILTAG_FOUND TRUE)
  set(APRILTAG_INCLUDE_DIRS "${APRILTAG_INCLUDE_DIR}")
  set(APRILTAG_LIBRARIES "${APRILTAG_LIBRARY}")
  get_filename_component(APRILTAG_LIBRARY_DIR "${APRILTAG_LIBRARY}" DIRECTORY)
else()
  set(APRILTAG_FOUND FALSE)
endif()

set(TAG_DEPS_READY TRUE)
if(NOT OpenCV_FOUND)
  message(STATUS "[apriltag_localization] 缺少 OpenCV: sudo apt install libopencv-dev")
  set(TAG_DEPS_READY FALSE)
endif()
if(NOT realsense2_FOUND)
  message(STATUS "[apriltag_localization] 缺少 librealsense2: 见 teach.md「1. 依赖安装」")
  set(TAG_DEPS_READY FALSE)
endif()
if(NOT APRILTAG_FOUND)
  message(STATUS "[apriltag_localization] 缺少 apriltag: 见 teach.md「1. 依赖安装」")
  set(TAG_DEPS_READY FALSE)
endif()
if(NOT TAG_DEPS_READY)
  message(STATUS "[apriltag_localization] 依赖不全，跳过 apriltag_localizer 节点编译；"
                 "补齐依赖后重新 colcon build --packages-select apriltag_localization")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(TAG_DEPS_READY)
  # 两个节点共用同一套 D435 + AprilTag + OpenCV 依赖：
  #   apriltag_localizer : Tag 绝对地标 -> map/odom TF + 校正 Odometry
  #   tag_approach       : 无雷达纯视觉测试 -> 走到 tag 前固定位姿
  add_executable(apriltag_localizer src/apriltag_localizer.cpp)
  add_executable(tag_approach src/tag_approach.cpp)

  if(TARGET realsense2::realsense2)
    set(REALSENSE_TARGET realsense2::realsense2)
  elseif(realsense2_LIBRARIES)
    set(REALSENSE_TARGET ${realsense2_LIBRARIES})
  else()
    set(REALSENSE_TARGET realsense2)
  endif()

  foreach(tag_target IN ITEMS apriltag_localizer tag_approach)
    target_include_directories(${tag_target}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    # 第三方头文件按 SYSTEM 引入：AprilTag 的 matd.h 用了 C 柔性数组，
    # 在 -Wpedantic 下会报 "ISO C++ forbids flexible array member"。
    # 本项目自身代码仍保持 -Wall -Wextra -Wpedantic 全量检查。
    target_include_directories(${tag_target} SYSTEM PRIVATE
      ${OpenCV_INCLUDE_DIRS}
      ${APRILTAG_INCLUDE_DIRS}
    )

    ament_target_dependencies(${tag_target}
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
      nav_msgs
      std_srvs
      tf2
      tf2_ros
      robot_msgs
    )

    target_link_libraries(${tag_target}
      ${OpenCV_LIBS}
      ${REALSENSE_TARGET}
      ${APRILTAG_LIBRARIES}
    )

    # 用户级安装（~/.local、/usr/local）的 .so 可能不在 ld 默认搜索路径，
    # 写入 RPATH，保证 ros2 run 时能直接加载 libapriltag.so。
    if(APRILTAG_LIBRARY_DIR)
      set_target_properties(${tag_target} PROPERTIES
        BUILD_RPATH "${APRILTAG_LIBRARY_DIR}"
        INSTALL_RPATH "${APRILTAG_LIBRARY_DIR}"
      )
    endif()
  endforeach()

  install(TARGETS
    apriltag_localizer
    tag_approach
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  teach.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
