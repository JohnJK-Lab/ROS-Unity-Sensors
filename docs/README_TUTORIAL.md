<p align="center">
  <a href="../README.md">简体中文主页</a> |
  <a href="README_EN.md">English README</a>
</p>

# Unity 与 ROS 2 联合仿真教程

# Unity and ROS 2 Integration Tutorial

本文档说明如何在 Unity 与 Ubuntu ROS 2 之间建立连接，运行本项目的相机、RGB-D、2D LiDAR 和 3D LiDAR，并在 RViz2 中验证数据。示例环境为 Unity `2022.3.55f1c1`、Ubuntu 22.04 和 ROS 2 Humble。

This guide explains how to connect Unity to ROS 2 on Ubuntu, run the RGB camera, RGB-D camera, 2D LiDAR, and 3D LiDAR in this project, and verify their output in RViz2. The example environment uses Unity `2022.3.55f1c1`, Ubuntu 22.04, and ROS 2 Humble.

## 中文教程（The English tutorial is provided below the Chinese tutorial.）

### 1. 通信结构

本项目的数据链路如下：

```text
Unity 传感器脚本
        │
        ▼
ROS-TCP-Connector
        │  TCP，默认端口 10000
        ▼
ROS-TCP-Endpoint（Ubuntu / ROS 2）
        │
        ▼
ROS 2 Topic、TF、RViz2
```

Unity 端负责生成并序列化 ROS 消息；Ubuntu 端的 ROS-TCP-Endpoint 接收数据并发布到 ROS 2。Unity 和 Ubuntu 可以运行在两台物理机上，也可以使用 Windows 主机加 Ubuntu 虚拟机，但双方必须能够通过网络互相访问。

### 2. 准备环境

建议准备：

- Unity Hub 与 Unity `2022.3.55f1c1`
- Ubuntu 22.04
- ROS 2 Humble
- Git、colcon 与 rosdep
- RViz2

如果 Ubuntu 运行在虚拟机中，桥接网络通常最直观。也可以使用 NAT，但必须正确配置端口转发。首先确认 Unity 主机能够访问 Ubuntu 的 IP，并确认 TCP 端口 `10000` 没有被防火墙阻止。

### 3. 安装 Unity ROS 包

如果直接打开仓库中的 `Unity_sensors`，所需依赖已经写在 `Packages/manifest.json` 中，Unity 通常会自动下载。若要在其他 Unity 工程中安装：

1. 打开 `Window > Package Manager`。
2. 点击左上角 `+`。
3. 选择 `Add package from git URL...`。
4. 分别添加下面两个地址。

ROS-TCP-Connector：

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

Visualizations：

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations
```

Visualizations 包不是传感器发布所必需，但可以帮助在 Unity 内查看 ROS 消息。

### 4. 在 Ubuntu 安装 ROS-TCP-Endpoint

打开 Ubuntu 终端：

```bash
mkdir -p ~/ROS-TCP-Endpoint/src
cd ~/ROS-TCP-Endpoint/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

安装依赖并编译：

```bash
cd ~/ROS-TCP-Endpoint
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

每次打开新终端后，先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS-TCP-Endpoint/install/setup.bash
```

启动 Endpoint：

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0
```

默认监听端口为 `10000`。`0.0.0.0` 表示监听本机所有网络接口；Unity 中仍应填写 Ubuntu 实际可访问的 IP，而不是 `0.0.0.0`。

查询 Ubuntu IP：

```bash
hostname -I
```

### 5. 配置 Unity 网络连接

1. 在 Unity 顶部菜单打开 `Robotics > ROS Settings`。
2. 将 `ROS IP Address` 设置为 Ubuntu/ROS 主机的 IP。
3. 将 `Host Port` 保持为 `10000`，除非你在 Endpoint 端改用了其他端口。
4. ROS 2 发行版选择或保持为 ROS 2 模式。
5. 保存设置并等待 Unity 完成脚本编译。

如果 Unity 和 ROS 2 运行在同一台 Linux 主机或正确映射端口的容器中，可以按实际网络结构使用 `127.0.0.1`；Windows 主机连接 Ubuntu 虚拟机时通常应填写虚拟机的局域网 IP。

### 6. 打开本项目

完整工程方式：

1. 在 Unity Hub 中选择 `Add project from disk`。
2. 选择仓库内的 `Unity_sensors` 文件夹。
3. 打开 `Assets/Scenes/SampleScene.unity`。
4. 等待 Package Manager 与脚本编译完成。

脚本复用方式：

1. 将根目录 `Scripts` 中需要的传感器脚本复制到目标 Unity 工程的 `Assets/Scripts`。
2. 如需时间或 TF，再复制 `auxiliary script` 中的相关脚本。
3. 将脚本挂载到对应 GameObject。
4. 在 Inspector 中设置 Topic、Frame ID、频率、量程、分辨率和 Layer Mask。

完整示例工程还提供：

- `Assets/Prefab/2D.prefab`
- `Assets/Prefab/3D.prefab`
- `Assets/Prefab/Camera.prefab`
- `Assets/Prefab/RGBD.prefab`

可将需要的 Prefab 拖入场景，再根据机器人模型调整位置和旋转。

### 7. 配置传感器与 Collider

主要脚本位于 `Scripts`：

- `CameraImagePublisher.cs`
- `RGBDCameraPublisher.cs`
- `LaserScan2DPublisher.cs`
- `LaserScan3DPublisher.cs`

2D LiDAR、3D LiDAR 和 RGB-D 深度都使用 Physics.Raycast。所有需要被检测的场景物体必须带有 Collider，例如 Box Collider、Mesh Collider 或 Sphere Collider。

LiDAR 使用 `Keep Scan Horizontal` 时，建议同时将 `LidarTFPublisher.keepFrameHorizontal` 设置为相同值，并让 TF 的 `childFrameId` 与传感器消息的 Frame ID 一致。

如果使用 `ROSClockPublisher.cs` 发布 `/clock`，场景中只保留一个该组件。需要采用仿真时间的 ROS 2 节点应设置：

```bash
ros2 param set /节点名称 use_sim_time true
```

### 8. 启动顺序

推荐按以下顺序运行：

1. 在 Ubuntu 终端启动 ROS-TCP-Endpoint。
2. 在另一个终端启动 RViz2。
3. 在 Unity 中进入 Play 模式。
4. 使用 ROS 2 命令检查话题与频率。

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /points
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

查看一次 2D LiDAR 消息：

```bash
ros2 topic echo /scan --once
```

### 9. 在 RViz2 中显示

启动：

```bash
rviz2
```

如果已经发布 TF，可将 `Fixed Frame` 设置为 `map`；如果没有完整 TF 树，可临时设为传感器消息使用的 Frame ID，例如 `base_scan` 或 `camera_link`。

添加显示项：

- `LaserScan` → Topic `/scan`
- `PointCloud2` → Topic `/points`
- `Image` → Topic `/camera/image_raw`
- `Image` → Topic `/camera/color/image_raw`
- `Image` → Topic `/camera/depth/image_raw`
- `TF` → 检查 `map` 与传感器 Frame 的关系

相机脚本已经针对 Unity 与 ROS 图像行顺序的差异执行一次上下翻转，不要在传输链路中重复翻转图像。

### 10. 常见问题

**Unity 无法连接 Endpoint**

- 检查 Ubuntu 终端是否显示 Endpoint 正在监听端口 `10000`。
- 检查 Unity 中填写的是 Ubuntu 实际 IP。
- 检查虚拟机网络模式、防火墙和端口。
- 确认两端没有使用不同端口。

**ROS 2 中看不到话题**

- 先确认 Unity 已进入 Play 模式。
- 查看 Unity Console 是否有连接或脚本异常。
- 重新加载 ROS 2 与 Endpoint 工作空间环境。

**RViz2 提示 No transform**

- 检查 `Fixed Frame`。
- 检查消息 Frame ID 与 TF `child_frame_id` 是否完全一致。
- 检查 `LidarTFPublisher` 或 `ROSAbsoluteTransformPublisher` 是否正在发布 `/tf`。

**LiDAR 扫描到地面**

- 开启 `Keep Scan Horizontal`。
- 让 LiDAR 与 TF 发布器的水平设置保持一致。
- 检查传感器安装高度、Layer Mask 与 Collider。

**RGB-D 运行较慢**

- 降低图像分辨率。
- 降低发布频率。
- 缩小最大深度或减少参与 Raycast 的层。

---

## English Tutorial

### 1. Communication Architecture

The data path used by this project is:

```text
Unity sensor scripts
        │
        ▼
ROS-TCP-Connector
        │  TCP, port 10000 by default
        ▼
ROS-TCP-Endpoint on Ubuntu / ROS 2
        │
        ▼
ROS 2 topics, TF, and RViz2
```

Unity generates and serializes ROS messages. ROS-TCP-Endpoint receives them on Ubuntu and publishes them into ROS 2. Unity and Ubuntu may run on separate physical machines or on a Windows host with an Ubuntu virtual machine, but the two systems must be able to reach each other over the network.

### 2. Prerequisites

Recommended environment:

- Unity Hub and Unity `2022.3.55f1c1`
- Ubuntu 22.04
- ROS 2 Humble
- Git, colcon, and rosdep
- RViz2

If Ubuntu runs in a virtual machine, bridged networking is usually the simplest option. NAT also works when port forwarding is configured correctly. Confirm that the Unity host can reach the Ubuntu IP and that TCP port `10000` is not blocked by a firewall.

### 3. Install the Unity ROS Packages

When you open the repository's `Unity_sensors` project, the required dependencies are already declared in `Packages/manifest.json`, so Unity should download them automatically. To install the packages in another Unity project:

1. Open `Window > Package Manager`.
2. Click the `+` button.
3. Select `Add package from git URL...`.
4. Add the following two URLs separately.

ROS-TCP-Connector:

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

Visualizations:

```text
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations
```

The Visualizations package is optional for publishing sensor data, but it is useful for viewing ROS messages inside Unity.

### 4. Install ROS-TCP-Endpoint on Ubuntu

Open an Ubuntu terminal:

```bash
mkdir -p ~/ROS-TCP-Endpoint/src
cd ~/ROS-TCP-Endpoint/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Install dependencies and build the workspace:

```bash
cd ~/ROS-TCP-Endpoint
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

In every new terminal, source the environments first:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS-TCP-Endpoint/install/setup.bash
```

Start the endpoint:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0
```

The default listening port is `10000`. `0.0.0.0` tells the endpoint to listen on every local network interface. In Unity, enter the Ubuntu machine's actual reachable IP address, not `0.0.0.0`.

Find the Ubuntu IP with:

```bash
hostname -I
```

### 5. Configure the Unity Connection

1. In Unity, open `Robotics > ROS Settings`.
2. Set `ROS IP Address` to the Ubuntu/ROS host IP.
3. Keep `Host Port` at `10000`, unless the endpoint uses another port.
4. Select or keep ROS 2 mode.
5. Save the settings and wait for Unity to finish compiling.

If Unity and ROS 2 run on the same Linux host, or inside a container with correctly mapped ports, `127.0.0.1` may be appropriate. A Windows host connecting to an Ubuntu virtual machine normally needs the VM's LAN IP.

### 6. Open This Project

Complete project workflow:

1. In Unity Hub, select `Add project from disk`.
2. Select the repository's `Unity_sensors` folder.
3. Open `Assets/Scenes/SampleScene.unity`.
4. Wait for Package Manager and script compilation to finish.

Reusable script workflow:

1. Copy the required files from the root `Scripts` directory to `Assets/Scripts` in your Unity project.
2. Copy the required files from `auxiliary script` when simulation time or TF publishing is needed.
3. Attach each script to the appropriate GameObject.
4. Configure topics, Frame IDs, rates, ranges, resolution, and Layer Masks in the Inspector.

The complete example also includes:

- `Assets/Prefab/2D.prefab`
- `Assets/Prefab/3D.prefab`
- `Assets/Prefab/Camera.prefab`
- `Assets/Prefab/RGBD.prefab`

Drag the required prefabs into the scene and adjust their positions and rotations for your robot model.

### 7. Configure Sensors and Colliders

The main scripts are stored in `Scripts`:

- `CameraImagePublisher.cs`
- `RGBDCameraPublisher.cs`
- `LaserScan2DPublisher.cs`
- `LaserScan3DPublisher.cs`

The 2D LiDAR, 3D LiDAR, and RGB-D depth implementation use Physics.Raycast. Every object that should be detected must have a Collider, such as a Box Collider, Mesh Collider, or Sphere Collider.

When a LiDAR uses `Keep Scan Horizontal`, set `LidarTFPublisher.keepFrameHorizontal` to the same value and make the TF `childFrameId` match the sensor message's Frame ID.

If `ROSClockPublisher.cs` publishes `/clock`, keep only one instance in the scene. ROS 2 nodes that should follow simulation time must enable `use_sim_time`:

```bash
ros2 param set /node_name use_sim_time true
```

### 8. Recommended Startup Order

1. Start ROS-TCP-Endpoint in an Ubuntu terminal.
2. Start RViz2 in another terminal.
3. Enter Play mode in Unity.
4. Verify the topics and rates with ROS 2 commands.

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /points
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

Inspect one 2D LiDAR message:

```bash
ros2 topic echo /scan --once
```

### 9. Visualize Data in RViz2

Start RViz2:

```bash
rviz2
```

If TF is available, set `Fixed Frame` to `map`. If the complete TF tree is not available, temporarily use the message Frame ID, such as `base_scan` or `camera_link`.

Add these displays:

- `LaserScan` → `/scan`
- `PointCloud2` → `/points`
- `Image` → `/camera/image_raw`
- `Image` → `/camera/color/image_raw`
- `Image` → `/camera/depth/image_raw`
- `TF` → verify the relationship between `map` and the sensor frames

The camera scripts already perform one vertical flip to account for the different Unity and ROS image row conventions. Do not add another flip elsewhere in the transport path.

### 10. Troubleshooting

**Unity cannot connect to the endpoint**

- Confirm that the Ubuntu terminal reports that the endpoint is listening on port `10000`.
- Confirm that Unity uses the Ubuntu machine's actual IP address.
- Check the virtual machine network mode, firewall, and port settings.
- Make sure both sides use the same port.

**No topics appear in ROS 2**

- Confirm that Unity is in Play mode.
- Check the Unity Console for connection or script errors.
- Source both the ROS 2 and endpoint workspaces again.

**RViz2 reports “No transform”**

- Check the RViz2 `Fixed Frame`.
- Make sure the message Frame ID exactly matches the TF `child_frame_id`.
- Confirm that `LidarTFPublisher` or `ROSAbsoluteTransformPublisher` is publishing `/tf`.

**The LiDAR detects the ground**

- Enable `Keep Scan Horizontal`.
- Use the same horizontal setting in the LiDAR and TF publisher.
- Check sensor height, Layer Mask, and Colliders.

**RGB-D performance is low**

- Reduce image resolution.
- Reduce the publishing rate.
- Reduce the maximum depth or limit the layers used by Raycast.

## Official References

- [ROS-TCP-Connector installation](https://github.com/Unity-Technologies/ROS-TCP-Connector#installation)
- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- [Unity ROS 2 integration setup](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md)
