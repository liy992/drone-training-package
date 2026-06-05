# 模块三：PX4 Offboard 与 ROS 2 代码控制

学习目标：

- 理解 PX4 local / NED 坐标系。
- 理解 ENU / NED、FLU / FRD 的区别。
- 能通过 ROS 2 话题控制 PX4 Offboard。
- 能使用控制菜单完成起飞、移动、悬停、降落。

前置条件：

- 官方 Iris 已经能通过 QGC 手动飞行。
- Ubuntu 20.04 + ROS 2 Foxy 环境、`px4_msgs`、Micro XRCE-DDS Agent 安装完成。
- 能检索到 `/fmu/out/*` 状态话题。


### 01 PX4 local / NED 坐标系

坐标系是无人机控制中常见的错误来源。飞机方向相反、高度不对，通常与坐标映射错误或 frame 不匹配有关。

常见坐标系：

| 坐标系 | 是否固定 | 含义 |
| --- | --- | --- |
| world / map | 固定 | 场景或规划使用的世界坐标 |
| PX4 local | 固定 | PX4 本地位置控制使用的局部坐标，通常以 home/local origin 为原点 |
| body | 随无人机移动 | 无人机机身坐标，随姿态变化 |
| sensor frame | 随传感器移动 | 例如相机或其它传感器自己的坐标系 |

PX4 local 通常按 NED 理解：

```text
PX4 local x: North
PX4 local y: East
PX4 local z: Down
```

控制时记住三点：

- 无人机上升时，PX4 local z 通常变得更负。
- `TrajectorySetpoint.position = [0, 0, -1]` 常表示飞到上方约 1 米。
- 如果直接把 Isaac / map 的 `z=+1.0` 发给 PX4，高度方向通常会错。

### 02 ENU / NED 与 FLU / FRD 转换

ENU:

```text
x = East  东
y = North 北
z = Up    上
```

NED:

```text
x = North 北
y = East  东
z = Down  下
```

ENU 和 NED 都是右手系，只是轴定义不同。不能把 NED 简单理解成左手系。

PX4 常用机体系是 FRD：

```text
x = Forward 前
y = Right   右
z = Down    下
```

ROS 常见机体系是 FLU：

```text
x = Forward 前
y = Left    左
z = Up      上
```

PX4 body 和 ROS body 的 y、z 方向不同。写桥接代码时别混用。

### 03 PX4 输入输出话题说明

ROS 2 控制 PX4 最常用的输入话题：

| 话题 | 作用 | 注意 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | 告诉 PX4 外部控制器控制位置/速度/姿态等 | 必须持续发布 |
| `/fmu/in/trajectory_setpoint` | 发送 PX4 local 下的位置/速度/加速度/yaw setpoint | 坐标必须是 PX4 local/NED |
| `/fmu/in/vehicle_command` | arm、disarm、切模式、land 等命令 | 通常需要重复或按时序发送 |

常用输出话题：

| 话题 | 作用 |
| --- | --- |
| `/fmu/out/vehicle_local_position` | PX4 local 位置估计 |
| `/fmu/out/vehicle_local_position_v1` | 部分 PX4 / px4_msgs 版本里的 local position 话题 |
| `/fmu/out/vehicle_status` | 飞行模式、解锁状态 |
| `/fmu/out/vehicle_status_v3` | 部分 PX4 / px4_msgs 版本里的 vehicle status 话题 |

### 04 Offboard 控制最小流程

ROS 2 控制 PX4 的基本流程如下：

```text
1. 等待 PX4 local position 有效
2. 持续发布 OffboardControlMode
3. 持续发布 TrajectorySetpoint
4. 发送 VehicleCommand 切换 Offboard
5. 发送 VehicleCommand 解锁 arm
6. 持续刷新 setpoint
7. 到达目标后继续保持、降落或切换其他状态
```

![PX4 Offboard 控制链路](assets/isaac_px4_training/px4_offboard_chain.png)

### 05 控制菜单使用

配套代码包默认放在桌面：

```text
/home/robot-a/Desktop/drone_training_github_release
```

里面主要有这些文件：

| 文件 | 作用 |
| --- | --- |
| `run_import_asset.sh` | 用 Isaac Sim 执行导入脚本，加载比赛无人机资产，并启动 Pegasus PX4 backend |
| `isaac/import_sunray150_with_px4.py` | Isaac Sim `--exec` 脚本，调用 Pegasus `Multirotor` 加载无人机 |
| `run_control_menu.sh` | 启动 ROS 2/PX4 命令行控制菜单 |
| `control/px4_offboard_menu.py` | 发布 `/fmu/in/*` Offboard 控制话题，订阅 `/fmu/out/*` 状态话题 |

使用前先确认 PX4 话题可见：

```bash
ros2 topic list --no-daemon | grep /fmu
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

`/fmu/out/vehicle_local_position` 和 `/fmu/out/vehicle_local_position_v1` 只要有一个有数据，就说明 ROS 2 已经能读到 PX4 local position。

第一步，启动 Isaac Sim 并导入无人机：

```bash
cd /home/robot-a/Desktop/drone_training_github_release
./run_import_asset.sh
```

如果只是检查 USD 资产能不能加载，不想启动 PX4，可以用：

```bash
./run_import_asset.sh --no-px4-autolaunch
```

常用参数：

```bash
./run_import_asset.sh --x 0 --y 0 --z 1.0
./run_import_asset.sh --yaw-deg 90
```

第二步，打开新终端启动控制菜单：

```bash
cd /home/robot-a/Desktop/drone_training_github_release
./run_control_menu.sh
```

进入菜单后，按下面顺序测试：

```text
status
takeoff 1.0
status
move 1 0 0
hover
land
quit
```

想快速验证控制链路，可以输入：

```text
demo
```

`demo` 会自动执行：起飞 1m，等待进入 Offboard，然后把 PX4 local x 目标增加 1m。

菜单命令说明：

| 命令 | 作用 |
| --- | --- |
| `status` | 显示 PX4 local position、当前目标点、解锁状态和飞行模式 |
| `takeoff 1.0` | 以当前位置为基准，上升约 1m，并切入 Offboard、发送 arm |
| `offboard` | 如果无人机已经通过 QGC 起飞，先保持当前位置并切入 Offboard |
| `move 1 0 0` | 在当前目标点基础上，PX4 local x 增加 1m |
| `goto 0 0 -1.2` | 直接设置 PX4 local/NED 目标点 |
| `hover` | 把当前位置设为目标点并保持 |
| `land` | 发送 PX4 land 命令 |

注意：`move` 不是起飞命令。它只在 PX4 已经处于 `OFFBOARD` 模式后改变目标点。如果 PX4 仍在 `AUTO_LOITER`、`POSCTL`、`MANUAL` 等模式，脚本会提示先运行 `takeoff` 或 `offboard`。

控制菜单使用 PX4 local/NED 坐标：

```text
x = North
y = East
z = Down
```

所以上升是 `z` 变小。比如：

```text
takeoff 1.0       # 目标 z = current_z - 1.0
move 0 0 -0.2    # 在当前目标基础上再上升约 0.2m
move 0 0 0.2     # 在当前目标基础上下降约 0.2m
```

如果脚本启动时报 Python 版本问题，通常是因为在 conda `(base)` 环境中运行了 ROS 2。`run_control_menu.sh` 默认使用 `/usr/bin/python3`，不应改成 conda 的 `python3`。

### 06 Python 控制代码讲解

下面只保留核心结构。实际代码还要处理 QoS、时间戳、状态检查和错误分支。

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class SimpleOffboard(Node):
    def __init__(self):
        super().__init__("simple_offboard")
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.timer = self.create_timer(0.05, self.tick)
        self.count = 0

    def tick(self):
        now = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.timestamp = now
        mode.position = True
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [0.0, 0.0, -1.0]  # PX4 local / NED: z=-1 表示向上约 1m
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)

        if self.count == 20:
            self.set_offboard()
            self.arm()
        self.count += 1

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def set_offboard(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)


def main():
    rclpy.init()
    node = SimpleOffboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 07 自动起飞、移动、悬停、降落

如果目标已经在 PX4 local/NED 坐标里：

- `x` 增大：向 North。
- `x` 减小：向 South。
- `y` 增大：向 East。
- `y` 减小：向 West。
- `z` 减小：向上。
- `z` 增大：向下。

不能把“前进”直接等同于 PX4 local x 增大。前进通常指 body forward，它和世界系/PX4 local 的关系取决于当前 yaw。

控制菜单示例：

```text
status
takeoff 1.0
move 1 0 0
move 0 1 0
move 0 0 -0.2
hover
land
```

### 08 Offboard 常见问题排查

Offboard 不起飞一般先查这些：

- 没有持续发布 `OffboardControlMode`。
- 没有持续发布 `TrajectorySetpoint`。
- 还没发送 setpoint 就切 Offboard。
- local position 无效。
- 没有 arm 成功。
- setpoint 的 z 方向错了。

飞行方向反了，优先检查：

- 目标点是在 map/ENU 还是 PX4 local/NED。
- 是否把 z 上直接发给了 PX4 z 下。
- 是否需要 x/y 交换。
- 是否把 body 前进误认为 local x 方向。

ROS 2 控制检查清单：

- [ ] 能看到 `/fmu/out/vehicle_local_position`。
- [ ] 能看到 `/fmu/out/vehicle_status`。
- [ ] 能持续发布 `/fmu/in/offboard_control_mode`。
- [ ] 能持续发布 `/fmu/in/trajectory_setpoint`。
- [ ] 能发送 `/fmu/in/vehicle_command`。
- [ ] 能切入 Offboard。
- [ ] 能 arm。
- [ ] 能按目标点移动。
