# 模块二：Isaac Sim + Pegasus + PX4 官方 Iris 试飞

学习目标：

- 安装并启用 Pegasus 插件。
- 理解 PX4-Autopilot、px4_msgs、Micro XRCE-DDS Agent、QGC 的作用。
- 使用 Pegasus 官方 Iris 验证最小飞行闭环。

前置条件：

- 已理解模块一中的基础链路。
- 操作系统使用 Ubuntu 20.04。
- ROS 2 使用 Foxy。
- Isaac Sim 能正常启动。
- 能打开终端执行基础命令。

完成要求：

- Isaac Sim 中能加载 Iris。
- Pegasus 能启动 PX4 backend。
- QGC 能连接 PX4 并显示 Ready To Fly。
- 能用 QGC 手动 arm、takeoff、移动、land。


### 01 Pegasus 插件安装与启用

流程分两步：先把 Pegasus 安装到 Isaac Sim 自己的 Python 环境，再到 Isaac Sim GUI 里添加 extension 路径并启用插件。

安装 Pegasus Simulator 源码：

```bash
# 如果在容器里操作，可以参考下面路径
cd /root
git clone --branch v4.5.1 --depth 1 https://github.com/PegasusSimulator/PegasusSimulator.git
cd /root/PegasusSimulator/extensions

# Isaac Sim 插件需要安装到 Isaac Sim 自己的 Python 环境
export ISAACSIM_PATH=/isaac-sim
export ISAACSIM_PYTHON=/isaac-sim/python.sh

cd ~/PegasusSimulator/extensions/pegasus.simulator
/isaac-sim/python.sh -m pip install --editable pegasus.simulator
```

如果上面命令不生效，换成下面写法：

```bash
$ISAACSIM_PYTHON -m pip install -e ~/PegasusSimulator/extensions/pegasus.simulator
```

本机路径可能不同。比如 Pegasus 源码可能在：

```bash
/home/robot-a/Documents/PegasusSimulator
```

Python 环境是常见错误来源。安装 Pegasus 插件时应使用 Isaac Sim 自带的 Python，不应使用系统 Python 或 conda Python。否则 GUI 里可能找不到插件，或者插件能显示但 import 失败。

安装完成后，启动 Isaac Sim GUI。在菜单栏进入：

```text
Window -> Extensions
```

![Extensions 入口](ph/微信图片_20260607153721.png)

在 Extensions 界面右上角打开 `Settings`，把 Pegasus extension 路径加入 Isaac Sim 的搜索路径。

![Extensions 设置界面](ph/微信图片_20260607153722.png)

点击路径列表旁边的加号，添加 Pegasus 源码里的 `extensions` 目录。注意添加的是 `extensions` 目录，不是里面的 `pegasus.simulator` 子目录。

常见路径：

```bash
/root/PegasusSimulator/extensions
/home/robot-a/Documents/PegasusSimulator/extensions
```

![点击加号添加 Pegasus extensions 路径](ph/微信图片_202606071537221.png)

路径添加完成后，在左上角搜索 `pegasus`。路径正确时，列表里会出现 Pegasus 插件。点击 `Install` 安装。

![搜索并安装 Pegasus 插件](ph/微信图片_202606071537201.png)

选择和当前 Isaac Sim 匹配的插件版本。Isaac Sim 4.5 对应 4.5.0；其他版本按实际情况选。安装后勾选 `Enabled`。



### 02 PX4-Autopilot、px4_msgs、Micro XRCE-DDS Agent、QGC 的作用

这一节安装无人机控制需要的依赖。下面命令以 Ubuntu 20.04 + ROS 2 Foxy 为准。Ubuntu 24.04 和 26.04 可以参考思路，但 ROS 2 发行版、Python 版本和 apt 源都可能不同，不能直接照抄。

Foxy 已经停止官方维护。本文档仍然使用 Foxy，是为了和 Ubuntu 20.04、PX4 v1.14.x 以及现有控制代码保持一致。

先装基础工具：

```bash
sudo apt update
sudo apt install -y \
  git wget curl gnupg2 lsb-release software-properties-common \
  build-essential cmake ninja-build python3-pip python3-venv
```

设置 locale：

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

安装 ROS 2 Foxy：

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep
```

初始化 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

每个新终端都需要 source ROS 2：

```bash
source /opt/ros/foxy/setup.bash
```

PX4-Autopilot 是飞控代码仓库。仿真时运行的是 PX4 SITL。Pegasus 通过 PX4 MAVLink backend 启动或连接它。

PX4-Autopilot 负责这些飞控逻辑：

- 飞控状态机。
- arm、disarm、mode switch。
- 接收 QGC 或 ROS 2 的目标。
- 输出执行器控制。

常见位置：

```bash
/root/PX4-Autopilot
```

Ubuntu 20.04 下可以这样下载和编译 PX4 v1.14：

```bash
cd ~
git clone --branch v1.14.3 --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

执行完依赖脚本后，重新打开终端，检查 SITL 是否能编译：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default none
```

如果 Pegasus 要自动启动 PX4，Pegasus UI 里的 PX4 路径应指向这个目录：

```text
/home/<用户名>/PX4-Autopilot
```

版本要对齐：

- Pegasus、PX4、`px4_msgs` 要匹配。
- 本培训包按 PX4 v1.14.x 编写。
- `px4_msgs` 和 PX4 版本不一致时，ROS 2 topic 可能出现，但消息字段会对不上。

`px4_msgs` 是 PX4 和 ROS 2 通信用的消息定义包。ROS 2 节点要发布 `OffboardControlMode`、`TrajectorySetpoint`、`VehicleCommand`，就需要它。

它的作用包括：

- 提供 PX4 输入输出消息类型。
- 让 ROS 2 节点能编译和运行 PX4 控制代码。
- 保持 ROS 2 消息和 PX4 uORB 消息对应。

常见位置：

```bash
/root/px4_ros_ws/src/px4_msgs
/home/robot-a/ros2_ws/src/px4_msgs
```

`px4_msgs` 应与 PX4 版本匹配。PX4 v1.14.x 通常使用 `release/1.14` 分支：

```bash
source /opt/ros/foxy/setup.bash

mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone --branch release/1.14 https://github.com/PX4/px4_msgs.git

cd ~/px4_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros_ws/install/setup.bash
```

检查消息是否可用：

```bash
ros2 interface show px4_msgs/msg/TrajectorySetpoint
ros2 interface show px4_msgs/msg/OffboardControlMode
ros2 interface show px4_msgs/msg/VehicleCommand
```

Micro XRCE-DDS Agent 是 PX4 和 ROS 2 DDS 网络之间的桥。

没有 Agent 时，常见现象是：

- PX4 侧 uXRCE-DDS Client 发出的数据到不了 ROS 2。
- ROS 2 发给 `/fmu/in/*` 的数据到不了 PX4。
- 看不到 `/fmu/out/*`，或者话题有了但没有数据。

常见启动形式：

```bash
MicroXRCEAgent udp4 -p 8888
```

端口和启动方式以当前 PX4 / Pegasus 配置为准。

如果系统里没有 `MicroXRCEAgent`，可以从源码编译：

```bash
cd ~
git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

检查命令：

```bash
MicroXRCEAgent --help
MicroXRCEAgent udp4 -p 8888
```

QGC 是地面站。它不是 ROS 2 控制的必需条件，但调试时建议打开。

它可以帮你确认这些状态：

- PX4 有没有连接。
- 是否 Ready To Fly。
- 当前模式、告警、参数是否正常。
- 虚拟摇杆或手柄能不能控制飞行。
- PX4 为什么不让飞机 arm 或起飞。

部分飞控问题在终端中不够直观，QGC 通常可以直接显示原因。

Ubuntu 20.04 下可以使用 QGroundControl AppImage：

```bash
cd ~/Downloads
wget -O QGroundControl.AppImage https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

如果 AppImage 报 `FUSE` 相关错误，先安装：

```bash
sudo apt update
sudo apt install -y libfuse2
```

如果最新 AppImage 在 Ubuntu 20.04 上启动失败，可以去 QGroundControl 的历史版本页面下载 4.3.x 或 4.4.x AppImage。培训只要求 QGC 能连接 PX4、显示状态并进行手动控制，不强制使用最新版。

Isaac Sim ROS 2 Bridge 需要在安装 Isaac Sim 时就安装好。它负责把仿真数据发布到 ROS 2，例如 `/clock`、TF、odometry、camera image。

### 03 Pegasus UI 配置与 PX4 路径检查

启用成功后，Isaac Sim 里会出现 Pegasus UI。使用前先检查：

- PX4 路径是否和前面安装的 `PX4-Autopilot` 路径一致。
- `Load Scene` 是否能加载插件自带环境。
- `Load Vehicle` 是否能加载插件自带的 Iris。
- Backend 是否选择 PX4。

如果 PX4 路径不对，Pegasus UI 可能能打开，但后面加载无人机或启动 PX4 backend 会失败。

![Pegasus 插件界面和基础试飞入口](ph/微信图片_202606071537212.png)

启动 QGC 的常见命令：

```bash
~/Downloads/QGroundControl.AppImage
```

QGC 连接后看到 `Ready To Fly`，说明 Pegasus、PX4、QGC 的基础链路已经通了。

### 04 加载官方 Iris 无人机

Iris 是 PX4/Pegasus 常用的官方四旋翼示例机型。

Iris 的资产和动力学参数已经配好，PX4 airframe 通常也匹配。Iris 能飞，说明 Pegasus、PX4、QGC、MAVLink 这条基础链路基本正常。

![Pegasus 插件界面和基础试飞入口](assets/isaac_px4_training/pegasus_ui_basic_flight.png)

通用流程：

1. 启动 Isaac Sim。
2. 启用 Pegasus 插件。
3. 在 Pegasus UI 中选择官方 Iris。
4. 选择 PX4 backend。
5. 加载场景。
6. 等待 PX4 SITL 启动。
7. 打开 QGroundControl。
8. 确认 QGC 连接 PX4。
9. 确认 Ready To Fly。
10. 使用 QGC 虚拟摇杆或手柄起飞、移动、降落。

这条链路可以理解为：

![整体仿真链路](assets/isaac_px4_training/overall_sim_chain.png)

Pegasus 负责在 Isaac Sim 里加载无人机和 PX4 backend；PX4 负责飞控状态机和控制器；QGC 负责地面站连接、状态检查和手动控制。

### 05 QGC 连接检查与 Ready To Fly 判断

进行代码控制时也建议打开 QGC。PX4 无法 arm、拒绝起飞、模式切换失败时，QGC 通常会直接显示原因。

QGC 可以用虚拟摇杆，也可以接手柄。测试时按这个顺序看：

- 先确认 QGC 能连接 PX4。
- 再确认摇杆输入映射正确。
- 起飞前看有没有红色告警。
- PX4 未 Ready To Fly 时，不应强行 arm。

![QGroundControl 界面](assets/isaac_px4_training/qgc_overview.png)

### 06 手动 arm、takeoff、移动、land

Iris 手动试飞用于确认 Isaac Sim、Pegasus、PX4、QGC 之间的基础链路是否正常。

按以下顺序检查：

```text
1. QGC 能看到 PX4 连接
2. QGC 显示 Ready To Fly
3. 可以 arm
4. 可以 takeoff
5. 可以用虚拟摇杆或手柄控制方向
6. 可以 land
```


### 07 基础链路故障排查

QGC 连接不上时，先查：

```text
1. Pegasus 是否启动 PX4 backend
2. PX4 SITL 是否正在运行
3. MAVLink 端口是否被占用
4. QGC 是否和 PX4 在同一网络可达范围
5. 容器网络是否隔离
```

Iris 不能起飞时，按下面顺序检查：

- PX4 是否 Ready To Fly。
- QGC 是否有 preflight check 报错。
- 仿真 timeline 是否播放。
- Pegasus 是否正确加载 Iris。
- PX4 backend 是否正常。

PX4 topic 不出现时，检查：

```bash
ros2 topic list --no-daemon | grep fmu
```

如果没有 `/fmu/out/*`：

- 检查 Micro XRCE-DDS Agent。
- 检查 PX4 是否启动 uXRCE-DDS Client。
- 检查 ROS 2 RMW 是否和环境匹配。
- 检查是否 source 了 `px4_msgs` 工作空间。

### 08 Iris 试飞检查清单

- [ ] Pegasus 能加载 Iris。
- [ ] PX4 SITL 能启动。
- [ ] QGC 能连接。
- [ ] QGC 显示 Ready To Fly。
- [ ] 能 arm。
- [ ] 能 takeoff。
- [ ] 能通过摇杆或虚拟摇杆控制方向。
- [ ] 能 land。
