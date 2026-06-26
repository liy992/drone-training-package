# Isaac Sim + Pegasus + PX4 Drone Technical Package

这是一个无人机仿真控制技术说明文档与示例代码包，包含：

- 可导入 Isaac Sim / Pegasus 的 Sunray150 自定义无人机 USD 资产。
- Isaac Sim `--exec` 导入脚本。
- ROS 2 / PX4 Offboard 命令行控制菜单。
- 独立货舱 USD 和控制脚本。
- RTX LiDAR 点云、TF、Odom 和 EGO-Planner 避障教学链路。
- 拆分后的中文技术说明文档和配套图片。

支持的系统组合：

| Ubuntu | ROS 2 | 说明 |
| --- | --- | --- |
| 20.04 | Foxy | 历史兼容环境，保留兼容指令 |
| 22.04 | Humble | 中间版本环境，适合较新的 ROS 2 Bridge / PX4 工作流 |
| 24.04 | Jazzy | 验证环境示例，用于说明本项目包的默认配置 |

适配环境示例：

```text
Ubuntu 24.04.4
ROS 2 Jazzy
PX4-Autopilot: /home/robot-a/PX4-Autopilot
px4_msgs: /home/robot-a/ros2_ws/src/px4_msgs
Micro XRCE-DDS Agent: /home/robot-a/px4_agent_ws
Isaac Sim: /home/robot-a/miniconda3/envs/env_isaacsim/bin/isaacsim
Pegasus Simulator: /home/robot-a/Documents/PegasusSimulator
```

本版本在保留 Ubuntu 20.04 + ROS 2 Foxy 历史指令的基础上，补充 Ubuntu 22.04 + ROS 2 Humble、Ubuntu 24.04 + ROS 2 Jazzy 的指令。宿主机 ROS 2 控制端应按实际系统选择对应发行版；验证环境使用 Jazzy。Isaac Sim 启动脚本内部可能会设置 `ROS_DISTRO=humble` 以匹配 Isaac Sim ROS 2 Bridge，需避免在同一个终端中反复 source 多套 ROS 2 环境。

文档已按模块拆分。文件名沿用早期版本命名，正文已按模块化说明文档组织：

```text
docs/00_文档导读.md
docs/01_实训系列一_基础认知与仿真控制链路.md
docs/02_实训系列二_Isaac_Sim_Pegasus_PX4_官方_Iris_试飞.md
docs/03_实训系列三_PX4_Offboard_ROS2_代码控制.md
docs/04_实训系列四_自定义无人机接入与比赛场景实战.md
docs/05_实训系列五_货舱资产加载与控制.md
docs/06_实训系列六_雷达点云与EGO_Planner避障.md
docs/07_疑难杂症_QA.md
```

## 目录

```text
.
├── assets/cargo_bay/
│   ├── air_fpv_box.usd
│   └── cargo_bay_resized_0p2_0p14_0p06.usd
├── assets/robots/sunray150_with_mid360_cargo/
│   ├── sunray150_with_mid360_cargo.usda   # Final drone asset entry
│   └── _deps/                            # Internal USD dependencies, do not delete
├── control/cargo_bay_control.py     # Cargo bay String command publisher
├── control/px4_offboard_menu.py     # ROS 2 / PX4 Offboard control menu
├── docs/                           # Technical Markdown and images
├── isaac/ego_demo/                 # Isaac LiDAR + EGO scene entry
├── isaac/import_sunray150_with_px4.py
├── isaac/load_cargo_bay.py
├── ros2_ws/src/ego_training_demo/  # EGO point cloud and PX4 bridge nodes
├── run_cargo_bay_control.sh
├── run_load_cargo_bay.sh
├── run_control_menu.sh
├── run_ego_stage_04.sh
├── run_ego_stage_05.sh
├── run_ego_stage_06.sh
└── run_import_asset.sh
```

## 环境要求

系统：

- Ubuntu 20.04 + ROS 2 Foxy、Ubuntu 22.04 + ROS 2 Humble、Ubuntu 24.04 + ROS 2 Jazzy 三套指令并列说明。
- 验证环境为 Ubuntu 24.04.4，ROS 2 控制终端默认使用 Jazzy。

Isaac Sim / Pegasus：

- Isaac Sim 可以正常启动。
- Pegasus Simulator 已安装并启用。
- Pegasus 里已经配置 PX4-Autopilot 路径。
- Isaac Sim 启动程序默认使用 `/home/robot-a/miniconda3/envs/env_isaacsim/bin/isaacsim`。
- Pegasus extension 默认路径为 `/home/robot-a/Documents/PegasusSimulator/extensions/pegasus.simulator`。

ROS 2 / PX4：

- 对应系统的 ROS 2 发行版已安装并 source；验证环境使用 Jazzy。
- `/home/robot-a/ros2_ws` 里的 `px4_msgs` 已编译并 source。
- `/home/robot-a/px4_agent_ws` 里的 Micro XRCE-DDS Agent 可以启动。
- 能看到 `/fmu/*` 话题。

source 命令示例：

Ubuntu 20.04 + Foxy：

```bash
source /opt/ros/foxy/setup.bash
source /home/robot-a/ros2_ws/install/setup.bash
```

Ubuntu 22.04 + Humble：

```bash
source /opt/ros/humble/setup.bash
source /home/robot-a/ros2_ws/install/setup.bash
```

Ubuntu 24.04 + Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-a/ros2_ws/install/setup.bash
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

参数示例：

```bash
./run_import_asset.sh --x 0 --y 0 --z 1.0
./run_import_asset.sh --yaw-deg 90
./run_import_asset.sh --no-px4-autolaunch
```

仅检查 USD 是否可加载时，可使用 `--no-px4-autolaunch`。

如果 Isaac Sim 不在默认路径，先设置：

```bash
export ISAACSIM_BIN=/home/robot-a/miniconda3/envs/env_isaacsim/bin/isaacsim
export PEGASUS_EXTENSION=/home/robot-a/Documents/PegasusSimulator/extensions/pegasus.simulator
```

## 运行 PX4 Offboard 控制菜单

打开另一个终端：

```bash
cd <repo-root>
./run_control_menu.sh
```

可按以下顺序进行功能验证：

```text
status
takeoff 1.0
status
move 1 0 0
hover
land
quit
```

简化验证命令：

```text
demo
```

`move` 不是起飞命令。它只会在 PX4 进入 `OFFBOARD` 后修改目标点。如果无人机已经通过 QGC 起飞，先运行：

```text
offboard
move 1 0 0
```

## 坐标提醒

`move` 命令使用世界坐标 / ENU 相对位移：

```text
move x = 世界坐标 x 正方向
move y = 世界坐标 y 正方向
move z = 向上
```

脚本会在内部把世界坐标 / ENU 位移转换为 PX4 local / NED：

```text
px4_dx = world_dy
px4_dy = world_dx
px4_dz = -world_dz
```

所以，`move 1 0 0` 表示沿世界坐标 x 正方向移动 1m；发送给 PX4 时会转换为 PX4 local y 正方向增加 1m。

例子：

```text
takeoff 1.0       # 目标 z = current_z - 1.0
move 1 0 0       # 沿世界坐标 x 正方向移动 1m
move 0 1 0       # 沿世界坐标 y 正方向移动 1m
move 0 0 0.2     # 在当前目标点基础上上升约 0.2m
move 0 0 -0.2    # 在当前目标点基础上下降约 0.2m
```

`goto` 仍然使用 PX4 local / NED 绝对目标点。只有 `move` 做世界坐标 / ENU 到 PX4 local / NED 的相对位移转换。

## EGO-Planner 雷达避障演示

精简后的 EGO 教学链路保留三个入口：

- Isaac 脚本：`isaac/ego_demo/`
- ROS 2 教学包：`ros2_ws/src/ego_training_demo/`

第一步，启动 Isaac 侧完整场景：

```bash
./run_ego_stage_04.sh
```

`run_ego_stage_04.sh` 是当前 Isaac 侧主入口。这个脚本会打开 Isaac Sim，加载无人机、三个障碍物、隐藏 RTX LiDAR，发布：

```text
/clock
/training/lidar/pointcloud
/tf
/tf_static
/drone_0_ego_odom
```

第二步，验证 EGO-Planner 是否能输出规划命令：

```bash
./run_ego_stage_05.sh
```

常用演示命令：

```bash
./run_ego_stage_05.sh send_default_goal:=true
```

第三步，接入 PX4 Offboard 闭环：

```bash
./run_ego_stage_06.sh
```

常用演示命令：

```bash
./run_ego_stage_06.sh send_default_goal:=true
```

说明文档：

```text
docs/06_实训系列六_雷达点云与EGO_Planner避障.md
```

- 控制脚本默认使用 `/usr/bin/python3`，避免 conda Python 和 ROS Python ABI 不匹配。
- 已兼容带版本号的 PX4 输出话题：`/fmu/out/vehicle_local_position_v1` 和 `/fmu/out/vehicle_status_v3`。
- 这个仓库只用于无人机仿真、资产导入和 ROS 2/PX4 控制验证。
- 出现异常时可查看 `docs/07_疑难杂症_QA.md`。该文件为问题排查文档。

## 独立货舱

本项目包还包含一个可以单独加载的货舱 USD，不依赖主项目场景。

货舱资产：

```text
assets/cargo_bay/cargo_bay_resized_0p2_0p14_0p06.usd
assets/cargo_bay/air_fpv_box.usd
```

加载货舱：

```bash
cd <repo-root>
./run_load_cargo_bay.sh
```

控制货舱：

```bash
cd <repo-root>
./run_cargo_bay_control.sh status
./run_cargo_bay_control.sh left_open
./run_cargo_bay_control.sh left_close
./run_cargo_bay_control.sh bottom_open
./run_cargo_bay_control.sh bottom_close
```

货舱控制只使用字符串话题：

```text
/cargo_bay/command
/cargo_bay/status
```

货舱加载和控制的完整说明见：

```text
docs/05_实训系列五_货舱资产加载与控制.md
```
