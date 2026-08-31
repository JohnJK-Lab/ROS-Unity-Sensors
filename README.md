<p align="center">
  <a href="README.md">简体中文</a> |
  <a href="README_EN.md">English</a>
</p>

# Unity ROS2 Sensors

基于 **Unity + ROS-TCP-Connector + ROS2** 实现的自定义机器人传感器仿真组件。
Custom robot sensor simulation components based on **Unity + ROS-TCP-Connector + ROS2**.
# Unity ROS2 Sensors

基于 **Unity + ROS-TCP-Connector + ROS2** 实现的机器人传感器仿真组件。

目前实现：

- 2D LiDAR
- 3D LiDAR
- RGB Camera
- RGB-D Camera
- 后续还会更新...

传感器数据通过 Unity Robotics ROS-TCP-Connector 发布到 ROS2，并可在 RViz2 中进行可视化。

---

## 1. 文件说明

- `CameraImagePublisher.cs`：RGB 相机，发布 `sensor_msgs/Image`
- `RGBDCameraPublisher.cs`：RGB-D 相机，发布 RGB、Depth 和 CameraInfo
- `LaserScan2DPublisher.cs`：2D 激光雷达，发布 `sensor_msgs/LaserScan`
- `LaserScan3DPublisher.cs`：3D 激光雷达，发布 `sensor_msgs/PointCloud2`

---

## 2. 环境

本项目测试环境：

- Ubuntu 22.04
- ROS2 Humble
- Unity 2020.3.48
- Unity Robotics ROS-TCP-Connector
- RViz2

本项目在 Unity 官方 ROS2 + Nav2 + SLAM 示例项目基础环境中进行了复现和测试：

https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example

Unity 与 ROS2 通过 ROS-TCP-Connector 建立通信。

---

## 3. RGB Camera

脚本：

`CameraImagePublisher.cs`

默认发布话题：

`/camera/image_raw`

消息类型：

`sensor_msgs/Image`

默认参数：

- Frame ID：`camera_link`
- Publish Hz：`10`
- Width：`640`
- Height：`480`
- Encoding：`rgb8`

程序对 Unity 图像进行了上下翻转，使图像在 ROS2 / RViz2 中保持正常方向。

RViz2 中添加：

`Add → Image → /camera/image_raw`

![RGB Camera](Images/img.png)

---

## 4. RGB-D Camera

脚本：

`RGBDCameraPublisher.cs`

默认发布：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/camera_info`

RGB 图像编码：

`rgb8`

Depth 图像编码：

`32FC1`

深度单位：

`m`

当前深度图通过 Unity Camera 向每个像素方向发射 Raycast，并根据 `hit.distance` 得到对应像素的深度值。

基本过程：

Camera Pixel → ScreenPointToRay → Physics.Raycast → hit.distance → Depth Image

因此需要被深度相机检测的场景物体具有 Collider。

例如：

- Box Collider
- Mesh Collider
- Sphere Collider

默认参数：

- Width：`320`
- Height：`240`
- Publish Hz：`5`
- Min Depth：`0.1 m`
- Max Depth：`20 m`

由于当前 Depth 使用逐像素 CPU Raycast，高分辨率下计算量较大。

![RGB-D Camera](Images/rgbd.png)

---

## 5. 2D LiDAR

脚本：

`LaserScan2DPublisher.cs`

默认发布：

`/scan`

消息类型：

`sensor_msgs/LaserScan`

默认 Frame：

`base_scan`

主要可调参数：

- Publish Period Seconds
- Range Meters Min
- Range Meters Max
- Scan Angle Start Degrees
- Scan Angle End Degrees
- Num Measurements Per Scan
- Keep Scan Horizontal

例如：

- Scan Angle Start Degrees：`0`
- Scan Angle End Degrees：`-359`
- Num Measurements Per Scan：`180`

表示接近 360° 扫描，共 180 束激光。

### Keep Scan Horizontal

2D 激光雷达增加了 `Keep Scan Horizontal` 开关。

当：

`Keep Scan Horizontal = true`

激光雷达只跟随机器人的 Yaw，忽略 Roll 和 Pitch，使激光扫描平面始终保持水平。

当：

`Keep Scan Horizontal = false`

激光雷达完整跟随机器人 Roll、Pitch 和 Yaw。

该功能主要用于解决移动机器人运动过程中，由于加速、制动或惯性产生俯仰，导致激光束误扫到地面的问题。

### 不考虑机器人俯仰

下图开启 `Keep Scan Horizontal`。

即使机器人发生俯仰，2D LiDAR 的扫描平面仍保持水平，可以减少地面误检测。

![2D LiDAR - Keep Horizontal](Images/2d1.png)

### 考虑机器人俯仰

下图关闭 `Keep Scan Horizontal`。

机器人沿图中的蓝色箭头方向运动时，由于车辆惯性产生俯仰，激光雷达会跟随车体姿态发生倾斜。

此时机器人后部部分激光束会向下照射，并与地面发生碰撞。图中蓝色框区域可以看到后部激光束检测到了地面，从而可能在 ROS2 中形成错误的障碍物信息。

![2D LiDAR - Follow Pitch](Images/2d2.png)

Debug 射线：

- 红色：激光命中物体
- 绿色：最大量程内未检测到物体

RViz2：

`Add → LaserScan → /scan`

---

## 6. 3D LiDAR

脚本：

`LaserScan3DPublisher.cs`

默认发布：

`/points`

消息类型：

`sensor_msgs/PointCloud2`

3D LiDAR 通过水平扫描和垂直多线扫描生成三维点云。

主要参数：

- Horizontal Angle Start Degrees
- Horizontal Angle End Degrees
- Horizontal Measurements
- Vertical Angle Min Degrees
- Vertical Angle Max Degrees
- Vertical Channels
- Range Meters Min
- Range Meters Max
- Keep Scan Horizontal

例如：

- Horizontal Angle Start Degrees：`0`
- Horizontal Angle End Degrees：`-359`
- Horizontal Measurements：`360`
- Vertical Angle Min Degrees：`-15`
- Vertical Angle Max Degrees：`15`
- Vertical Channels：`16`

即可模拟一个接近 360° 扫描的 16 线 3D LiDAR。

`Vertical Channels` 可以根据需要设置为：

- 16
- 32
- 64

当前 PointCloud2 包含：

- x
- y
- z
- intensity

坐标转换关系：

Unity：

- X = Right
- Y = Up
- Z = Forward

ROS：

- X = Forward
- Y = Left
- Z = Up

因此：

- ROS X = Unity Z
- ROS Y = -Unity X
- ROS Z = Unity Y

3D LiDAR 同样支持 `Keep Scan Horizontal`。

Debug 射线：

- 红色：激光命中物体
- 绿色：最大量程内未检测到物体

![3D LiDAR](Images/3d.png)

RViz2：

`Add → PointCloud2 → /points`

---

## 7. ROS2 查看

查看所有话题：

`ros2 topic list`

查看 2D LiDAR：

`ros2 topic echo /scan --once`

查看 3D LiDAR：

`ros2 topic hz /points`

查看 RGB Camera：

`ros2 topic hz /camera/image_raw`

查看 RGB-D：

`ros2 topic hz /camera/color/image_raw`

`ros2 topic hz /camera/depth/image_raw`

---

## 8. 注意事项

- Unity 与 ROS2 需要通过 ROS-TCP-Connector 正常连接。
- Frame ID 需要与 ROS2 TF 中对应的坐标系保持一致。
- 2D LiDAR、3D LiDAR 和 RGB-D Depth 均使用 Physics.Raycast。
- 被 Raycast 检测的物体需要具有 Collider。
- LiDAR 扫描束数量越多，Unity 的计算量越大。
- RGB-D 当前采用 CPU Raycast，高分辨率时性能开销较大。
- `Keep Scan Horizontal` 可以减少机器人 Roll / Pitch 导致的 2D 激光地面误检测。
- 3D LiDAR 发布的是 `PointCloud2`，不是 `LaserScan`。

---

## 9. 项目结构

Unity-ROS2-Sensors/

├── Scripts/

│   ├── CameraImagePublisher.cs

│   ├── RGBDCameraPublisher.cs

│   ├── LaserScan2DPublisher.cs

│   └── LaserScan3DPublisher.cs

├── fig/


└── README.md

---

## License

本项目主要用于学习、科研及机器人仿真开发。

如果这个项目对你有帮助，欢迎 Star ⭐
