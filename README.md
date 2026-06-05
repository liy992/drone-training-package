# Isaac Sim + Pegasus + PX4 Drone Training Package

这是一个无人机仿真控制说明文档与示例代码包，包含：

- 可导入 Isaac Sim / Pegasus 的 Sunray150 比赛无人机 USD 资产。
- Isaac Sim `--exec` 导入脚本。
- ROS 2 / PX4 Offboard 命令行控制菜单。
- 拆分后的中文培训文档和配套图片。

推荐运行环境：

```text
Ubuntu 20.04
ROS 2 Foxy
PX4 v1.14.x
```

代码按 Ubuntu 20.04、24.04 和 26.04 做了兼容处理，但目前仅在 Ubuntu 20.04 上完成测试。建议使用 Ubuntu 20.04 运行本文档中的示例。24.04 和 26.04 需要根据实际 ROS 2 版本调整依赖。

ROS 2 Foxy 已经停止官方维护。本文档继续使用 Foxy，是为了匹配 Ubuntu 20.04 环境。

文档已拆分为 6 个文件。文件名沿用早期版本命名，正文已按模块化说明文档组织：

```text
docs/00_文档导读.md
docs/01_实训系列一_基础认知与仿真控制链路.md
docs/02_实训系列二_Isaac_Sim_Pegasus_PX4_官方_Iris_试飞.md
docs/03_实训系列三_PX4_Offboard_ROS2_代码控制.md
docs/04_实训系列四_自定义无人机接入与比赛场景实战.md
docs/05_疑难杂症_QA.md
```

## 目录

```text
.
├── assets/robots/sunray150_with_mid360_cargo/
│   ├── sunray150_with_mid360_cargo.usda   # Final drone asset entry
│   └── _deps/                            # Internal USD dependencies, do not delete
├── control/px4_offboard_menu.py    # ROS 2 / PX4 Offboard control menu
├── docs/                           # Training Markdown and images
├── isaac/import_sunray150_with_px4.py
├── run_control_menu.sh
└── run_import_asset.sh
```

## 环境要求

系统：

- Ubuntu 20.04 是测试环境。
- ROS 2 默认使用 Foxy。

Isaac Sim / Pegasus：

- Isaac Sim 可以正常启动。
- Pegasus Simulator 已安装并启用。
- Pegasus 里已经配置 PX4-Autopilot 路径。

ROS 2 / PX4：

- ROS 2 Foxy 已安装并 source。
- `px4_msgs` 已编译并 source。
- Micro XRCE-DDS Agent 或 PX4 ROS 2 bridge 正常运行。
- 能看到 `/fmu/*` 话题。

常用 source 命令：

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ros_ws/install/setup.bash
```

检查话题：

```bash
ros2 topic list --no-daemon | grep /fmu
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

这两个 local-position 话题只要有一个有数据，就可以继续往下测。

## 在 Isaac Sim 中加载无人机

```bash
cd <repo-root>
./run_import_asset.sh
```

无人机资产入口：

```text
assets/robots/sunray150_with_mid360_cargo/sunray150_with_mid360_cargo.usda
```

`_deps/` 是 USD 相对引用用到的依赖目录。一般不需要打开，也不应删除。

常用参数：

```bash
./run_import_asset.sh --x 0 --y 0 --z 1.0
./run_import_asset.sh --yaw-deg 90
./run_import_asset.sh --no-px4-autolaunch
```

只想检查 USD 能不能加载时，再使用 `--no-px4-autolaunch`。

如果 Isaac Sim 不在默认路径，先设置：

```bash
export ISAACSIM_BIN=/path/to/isaacsim
export PEGASUS_EXTENSION=/path/to/PegasusSimulator/extensions/pegasus.simulator
```

## 运行 PX4 Offboard 控制菜单

打开另一个终端：

```bash
cd <repo-root>
./run_control_menu.sh
```

建议按以下顺序测试：

```text
status
takeoff 1.0
status
move 1 0 0
hover
land
quit
```

快速 demo：

```text
demo
```

`move` 不是起飞命令。它只会在 PX4 进入 `OFFBOARD` 后修改目标点。如果无人机已经通过 QGC 起飞，先运行：

```text
offboard
move 1 0 0
```

## 坐标提醒

控制菜单使用 PX4 local / NED：

```text
x = North
y = East
z = Down
```

所以，上升意味着 `z` 变小。

例子：

```text
takeoff 1.0       # 目标 z = current_z - 1.0
move 0 0 -0.2    # 在当前目标点基础上上升约 0.2m
move 0 0 0.2     # 在当前目标点基础上下降约 0.2m
```

## 说明

- 控制脚本默认使用 `/usr/bin/python3`，避免 conda Python 和 ROS Python ABI 不匹配。
- 已兼容带版本号的 PX4 输出话题：`/fmu/out/vehicle_local_position_v1` 和 `/fmu/out/vehicle_status_v3`。
- 这个仓库只用于无人机仿真、资产导入和 ROS 2/PX4 控制训练。
- 遇到问题先查看 `docs/05_疑难杂症_QA.md`。该文件为问题排查文档。
