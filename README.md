<p align="center">
  <a href="README.md">简体中文</a> |
  <a href="docs/README_EN.md">English</a> |
  <a href="docs/README_TUTORIAL.md">中英双语教程</a>
</p>

# ROS Unity Sensors

**正式版 V1.0**

基于 **Unity、ROS-TCP-Connector 与 ROS 2** 的机器人传感器仿真项目。项目提供可独立复用的 C# 传感器脚本、辅助时间与 TF 脚本，以及可直接打开的完整 Unity 示例工程。传感器数据可发布到 ROS 2，并在 RViz2 中显示。

<p align="center">
  <img src="fig/unity01.png" alt="Unity 中的传感器仿真场景" width="49%">
  <img src="fig/rviz01.png" alt="RViz2 中的传感器数据" width="49%">
</p>

## V1.0 内容

- 2D LiDAR：发布 `sensor_msgs/LaserScan`
- 3D LiDAR：发布 `sensor_msgs/PointCloud2`
- RGB Camera：发布 `sensor_msgs/Image`
- RGB-D Camera：发布 RGB、Depth 与 `sensor_msgs/CameraInfo`
- LiDAR 和通用物体的动态 TF 发布
- Unity 仿真时钟与 `/clock` 发布
- 可直接打开的 `Unity_sensors` 示例工程
- 传感器模型、预制体、示例场景与 RViz2 展示图片

## 运行环境

本版本工程与教程使用以下环境：

- Ubuntu 22.04
- ROS 2 Humble
- Unity `2022.3.55f1c1`
- Unity Robotics ROS-TCP-Connector
- Unity Robotics Visualizations
- RViz2

Unity 与 ROS 2 之间通过 ROS-TCP-Connector 和 ROS-TCP-Endpoint 建立 TCP 通信。完整安装与连接步骤见[中英双语教程](docs/README_TUTORIAL.md)。

## 快速开始

### 使用完整 Unity 工程

1. 使用 Unity Hub 打开 `Unity_sensors` 文件夹。
2. 在 Ubuntu 中编译并启动 ROS-TCP-Endpoint。
3. 在 Unity 中打开 `Robotics > ROS Settings`，填写 Ubuntu/ROS 主机 IP，端口保持 `10000`。
4. 打开 `Assets/Scenes/SampleScene.unity`。
5. 启动 RViz2，然后运行 Unity 场景。

### 将脚本用于其他 Unity 工程

1. 安装 ROS-TCP-Connector。
2. 将 `Scripts` 中需要的传感器脚本复制到 Unity 工程的 `Assets` 下。
3. 如需仿真时间或 TF，再从 `auxiliary script` 复制相应脚本。
4. 将脚本挂载到对应的 Camera 或 LiDAR GameObject，并在 Inspector 中配置 Topic、Frame ID、频率与量程。
5. 确保需要被 LiDAR 或深度相机检测的场景物体带有 Collider。

## 主要传感器

| 传感器 | 脚本 | 默认话题 | ROS 2 消息 | 默认频率 |
| --- | --- | --- | --- | --- |
| RGB Camera | `CameraImagePublisher.cs` | `/camera/image_raw` | `sensor_msgs/Image` | 10 Hz |
| RGB-D Camera | `RGBDCameraPublisher.cs` | `/camera/color/image_raw`、`/camera/depth/image_raw` | `sensor_msgs/Image`、`sensor_msgs/CameraInfo` | 5 Hz |
| 2D LiDAR | `LaserScan2DPublisher.cs` | `/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| 3D LiDAR | `LaserScan3DPublisher.cs` | `/points` | `sensor_msgs/PointCloud2` | 10 Hz |

## RGB Camera

脚本：`Scripts/CameraImagePublisher.cs`

默认参数：

- Topic：`/camera/image_raw`
- Frame ID：`camera_link`
- 分辨率：`320 × 240`
- 发布频率：`10 Hz`
- Encoding：`rgb8`

Unity Texture2D 通常从左下角理解像素行，而 ROS Image 查看器通常把第一行当作图像顶部。脚本在发布前执行一次上下翻转，使图像在 ROS 2 与 RViz2 中方向正确。

在 RViz2 中添加 `Image`，并选择 `/camera/image_raw`。

![RGB Camera](fig/img.png)

## RGB-D Camera

脚本：`Scripts/RGBDCameraPublisher.cs`

默认发布：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/camera_info`

默认参数：

- Frame ID：`camera_link`
- 分辨率：`320 × 240`
- 发布频率：`5 Hz`
- RGB Encoding：`rgb8`
- Depth Encoding：`32FC1`
- 深度单位：米
- 最小/最大深度：`0.1 m / 20 m`

深度图通过逐像素 Physics.Raycast 计算。场景物体必须带有 Collider；提高分辨率或发布频率会明显增加 CPU 开销。

![RGB-D Camera](fig/rgbd.png)

## 2D LiDAR

脚本：`Scripts/LaserScan2DPublisher.cs`

默认配置：

- Topic：`/scan`
- Frame ID：`base_scan`
- 发布周期：`0.1 s`
- 量程：`0.12–100 m`
- 扫描角：`0°` 到 `-359°`
- 每圈测量数：`180`
- `Keep Scan Horizontal`：开启

开启 `Keep Scan Horizontal` 后，扫描平面只跟随机器人的 Yaw，忽略 Roll 和 Pitch，可减少车辆加速、制动或越障时的地面误检。关闭后，扫描平面会完整跟随车体姿态。

![2D LiDAR - Keep Horizontal](fig/2d1.png)

![2D LiDAR - Follow Pitch](fig/2d2.png)

Debug 射线中，红色表示命中物体，绿色表示最大量程内未命中。RViz2 中添加 `LaserScan` 并选择 `/scan`。

## 3D LiDAR

脚本：`Scripts/LaserScan3DPublisher.cs`

默认配置：

- Topic：`/points`
- Frame ID：`base_scan`
- 发布周期：`0.1 s`
- 量程：`0.1–50 m`
- 水平扫描：`0°` 到 `-359°`，`360` 个采样
- 垂直扫描：`-15°` 到 `15°`，`16` 线
- `Keep Scan Horizontal`：开启
- 发布字段：`x`、`y`、`z`、`intensity`

坐标转换关系：

- ROS X = Unity Z
- ROS Y = -Unity X
- ROS Z = Unity Y

![3D LiDAR](fig/3d.png)

RViz2 中添加 `PointCloud2` 并选择 `/points`。

## 辅助脚本

`auxiliary script` 中包含：

| 文件 | 用途 |
| --- | --- |
| `Clock.cs` | 提供 Unity 仿真时间 |
| `TimeStamp.cs` | 在 Unity 时间与 ROS 2 Time 消息之间转换 |
| `ROSClockPublisher.cs` | 发布仿真时钟到 `/clock`；场景中只应保留一个实例 |
| `Lidar TF Publisher.cs` | 发布 LiDAR 相对世界原点的动态 TF；Frame ID 与水平设置应和 LiDAR 脚本一致 |
| `ROSAbsoluteTransformPublisher.cs` | 发布指定 Transform 相对 Unity 世界原点的动态 TF |

## ROS 2 检查命令

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /points
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /scan --once
```

## 项目结构

```text
ROS-Unity-Sensors/
├── auxiliary script/       # 时间、/clock 与 TF 辅助脚本
├── fig/                    # README 与效果展示图片
├── Scripts/                # 主要传感器脚本
├── Unity_sensors/          # 完整 Unity 示例工程
│   ├── Assets/
│   ├── Packages/
│   └── ProjectSettings/
├── docs/
│   ├── README_EN.md        # English README
│   └── README_TUTORIAL.md  # 中英双语安装与联调教程
└── README.md
```

> 提交 Unity 工程到 GitHub 时，不要提交 `Library`、`Temp`、`Logs`、`obj`、`UserSettings`、`.vs`、`*.csproj` 和 `*.sln` 等自动生成内容。请在上传前配置 Unity `.gitignore`。

## 注意事项

- Topic 名称和 Frame ID 必须与 RViz2、TF 树及其他 ROS 2 节点保持一致。
- 2D LiDAR、3D LiDAR 与 RGB-D 深度均依赖 Physics.Raycast 和 Collider。
- LiDAR 射线数、RGB-D 分辨率和发布频率越高，Unity 计算开销越大。
- 3D LiDAR 发布 `PointCloud2`，不是 `LaserScan`。
- 使用 `/clock` 时，需要让相关 ROS 2 节点启用 `use_sim_time`。

## 参考项目

- [Unity ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- [Unity Robotics Nav2 SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

## 用途说明

本项目主要用于学习、科研与机器人仿真开发。如果项目对你有帮助，欢迎 Star。
