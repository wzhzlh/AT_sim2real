# Robot descriptions

本工作区维护 atdog、atdog2、atdog3 和 arm1 的模型与场景。模型随项目维护，构建不会从上游全量下载其它机器人。

`arm1_description` 提供六轴 URDF 和五轴 MuJoCo 控制场景。当前 `2026_R2_NEW_ARM_LOWER` 固件只驱动 joint1 至 joint5；仿真与 RL 部署按五轴策略接口对齐。
