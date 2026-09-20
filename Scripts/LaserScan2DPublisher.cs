using System;
using System.Collections.Generic;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;

using UnityEngine;
using UnityEngine.Serialization;


public class LaserScan2DPublisher : MonoBehaviour
{
    // =========================================================
    // ROS
    // =========================================================

    [Header("ROS")]

    [Tooltip("ROS2 LaserScan 话题名称")]
    public string topic = "/scan";

    [Tooltip("LaserScan 消息中的 frame_id")]
    public string FrameId = "base_scan";


    // =========================================================
    // 扫描频率
    // =========================================================

    [Header("Scan Time")]

    [FormerlySerializedAs("TimeBetweenScansSeconds")]
    [Tooltip("每次完整扫描之间的时间，例如 0.1 = 10Hz")]
    public double PublishPeriodSeconds = 0.1;

    [Tooltip("每根激光线之间的采样时间。设为 0 表示一帧内完成全部扫描")]
    public float TimeBetweenMeasurementsSeconds = 0.0f;


    // =========================================================
    // 测距范围
    // =========================================================

    [Header("Range")]

    [Tooltip("最小测量距离，单位 m")]
    public float RangeMetersMin = 0.12f;

    [Tooltip("最大测量距离，单位 m")]
    public float RangeMetersMax = 100.0f;


    // =========================================================
    // 扫描角度
    // =========================================================

    [Header("Scan Angle")]

    [Tooltip("扫描起始角度，单位度")]
    public float ScanAngleStartDegrees = 0.0f;

    [Tooltip("扫描结束角度，单位度，例如 -359 表示接近完整 360°")]
    public float ScanAngleEndDegrees = -359.0f;

    [Tooltip("每次发布后整体旋转扫描区域，一般普通 2D 激光雷达设为 0")]
    public float ScanOffsetAfterPublish = 0.0f;


    // =========================================================
    // 激光束数量
    // =========================================================

    [Header("Measurements")]

    [Tooltip("一次完整扫描中的激光束数量")]
    public int NumMeasurementsPerScan = 180;


    // =========================================================
    // 扫描姿态
    // =========================================================

    [Header("Scan Orientation")]

    [Tooltip(
        "开启：激光始终保持世界水平，只跟随车辆 Yaw，忽略 Roll/Pitch。\n" +
        "关闭：激光完整跟随 laser_link 的 Roll/Pitch/Yaw。"
    )]
    public bool KeepScanHorizontal = true;


    // =========================================================
    // 碰撞检测
    // =========================================================

    [Header("Collision Detection")]

    [Tooltip("需要检测的 Unity Layer 名称")]
    public string LayerMaskName = "Default";

    [Tooltip("是否忽略 Trigger Collider")]
    public bool IgnoreTrigger = true;


    // =========================================================
    // Debug
    // =========================================================

    [Header("Debug")]

    [Tooltip("是否在 Scene 窗口显示激光射线")]
    public bool ShowDebugRays = true;

    [Tooltip("命中障碍物时的射线颜色")]
    public Color HitRayColor = Color.red;

    [Tooltip("未命中障碍物时的射线颜色")]
    public Color MissRayColor = Color.green;

    [Tooltip("Debug 射线显示时间")]
    public float DebugRayDuration = 0.05f;


    // =========================================================
    // 内部变量
    // =========================================================

    private float m_CurrentScanAngleStart;
    private float m_CurrentScanAngleEnd;

    private ROSConnection m_Ros;

    private double m_TimeNextScanSeconds = -1.0;

    private int m_NumMeasurementsTaken = 0;

    private readonly List<float> ranges =
        new List<float>();

    private bool isScanning = false;

    private double m_TimeLastScanBeganSeconds = -1.0;

    private int m_LayerMask;

    private QueryTriggerInteraction m_QueryTrigger;

    private float[] m_EmptyIntensities;


    // =========================================================
    // Start
    // =========================================================

    protected virtual void Start()
    {
        // -----------------------------------------------------
        // 参数保护
        // -----------------------------------------------------

        if (NumMeasurementsPerScan < 2)
        {
            NumMeasurementsPerScan = 2;
        }

        if (PublishPeriodSeconds <= 0)
        {
            PublishPeriodSeconds = 0.1;
        }

        if (RangeMetersMin < 0)
        {
            RangeMetersMin = 0;
        }

        if (RangeMetersMax <= RangeMetersMin)
        {
            RangeMetersMax =
                RangeMetersMin + 1.0f;
        }


        // -----------------------------------------------------
        // ROS
        // -----------------------------------------------------

        m_Ros =
            ROSConnection.GetOrCreateInstance();

        m_Ros.RegisterPublisher<LaserScanMsg>(
            topic
        );


        // -----------------------------------------------------
        // 初始化扫描角度
        // -----------------------------------------------------

        m_CurrentScanAngleStart =
            ScanAngleStartDegrees;

        m_CurrentScanAngleEnd =
            ScanAngleEndDegrees;


        // -----------------------------------------------------
        // Layer Mask
        // -----------------------------------------------------

        m_LayerMask =
            LayerMask.GetMask(
                LayerMaskName
            );

        if (m_LayerMask == 0)
        {
            Debug.LogWarning(
                "LaserScan2DPublisher: LayerMask '" +
                LayerMaskName +
                "' 不存在或没有匹配到 Layer，" +
                "将临时检测所有 Layer。"
            );

            m_LayerMask = ~0;
        }

        m_QueryTrigger =
            IgnoreTrigger
            ? QueryTriggerInteraction.Ignore
            : QueryTriggerInteraction.Collide;

        if (ranges.Capacity < NumMeasurementsPerScan)
        {
            ranges.Capacity = NumMeasurementsPerScan;
        }

        // 强度始终为 0，可跨帧复用同一个只读数组。
        m_EmptyIntensities = new float[NumMeasurementsPerScan];


        // -----------------------------------------------------
        // 第一次扫描时间
        // -----------------------------------------------------

        m_TimeNextScanSeconds =
            Clock.Now +
            PublishPeriodSeconds;
    }


    // =========================================================
    // Begin Scan
    // =========================================================

    private void BeginScan()
    {
        // 保留运行时修改 Inspector 参数的能力。
        m_QueryTrigger =
            IgnoreTrigger
            ? QueryTriggerInteraction.Ignore
            : QueryTriggerInteraction.Collide;

        if (ranges.Capacity < NumMeasurementsPerScan)
        {
            ranges.Capacity = NumMeasurementsPerScan;
        }

        if (m_EmptyIntensities == null ||
            m_EmptyIntensities.Length != NumMeasurementsPerScan)
        {
            m_EmptyIntensities = new float[NumMeasurementsPerScan];
        }

        isScanning = true;

        m_TimeLastScanBeganSeconds =
            Clock.Now;

        m_TimeNextScanSeconds =
            m_TimeLastScanBeganSeconds +
            PublishPeriodSeconds;

        m_NumMeasurementsTaken = 0;

        ranges.Clear();
    }


    // =========================================================
    // End Scan
    // =========================================================

    public void EndScan()
    {
        if (ranges.Count == 0)
        {
            Debug.LogWarning(
                $"LaserScan2DPublisher: 扫描了 {m_NumMeasurementsTaken} 束，" +
                "但 ranges 中没有数据。"
            );
        }
        else if (
            ranges.Count != m_NumMeasurementsTaken ||
            ranges.Count != NumMeasurementsPerScan
        )
        {
            Debug.LogWarning(
                $"LaserScan2DPublisher: 期望 {NumMeasurementsPerScan} 束，" +
                $"实际扫描 {m_NumMeasurementsTaken} 束，" +
                $"ranges 数量为 {ranges.Count}。"
            );
        }


        // =====================================================
        // ROS 时间戳
        // =====================================================

        TimeMsg timestamp =
            GetSystemTimeMessage();


        // =====================================================
        // Unity → ROS 角度转换
        // =====================================================

        float angleStartRos =
            -m_CurrentScanAngleStart *
            Mathf.Deg2Rad;

        float angleEndRos =
            -m_CurrentScanAngleEnd *
            Mathf.Deg2Rad;


        /*
         * ROS LaserScan 要求角度按照逆时针增加。
         *
         * 如果转换之后角度顺序反了，
         * 就交换 angle_min / angle_max，
         * 同时把 ranges 顺序反转。
         */

        if (angleStartRos > angleEndRos)
        {
            float temp =
                angleStartRos;

            angleStartRos =
                angleEndRos;

            angleEndRos =
                temp;

            ranges.Reverse();
        }


        // =====================================================
        // LaserScan 消息
        // =====================================================

        var msg =
            new LaserScanMsg
            {
                header =
                    new HeaderMsg
                    {
                        frame_id =
                            FrameId,

                        stamp =
                            timestamp
                    },

                range_min =
                    RangeMetersMin,

                range_max =
                    RangeMetersMax,

                angle_min =
                    angleStartRos,

                angle_max =
                    angleEndRos,

                // N 个点之间有 N-1 个间隔
                angle_increment =
                    NumMeasurementsPerScan > 1
                    ?
                    (angleEndRos - angleStartRos)
                    /
                    (NumMeasurementsPerScan - 1)
                    :
                    0.0f,

                time_increment =
                    TimeBetweenMeasurementsSeconds,

                scan_time =
                    (float)PublishPeriodSeconds,

                intensities =
                    m_EmptyIntensities,

                ranges =
                    ranges.ToArray()
            };


        // =====================================================
        // 发布
        // =====================================================

        m_Ros.Publish(
            topic,
            msg
        );


        // =====================================================
        // Reset
        // =====================================================

        m_NumMeasurementsTaken = 0;

        ranges.Clear();

        isScanning = false;


        double now =
            Clock.time;

        if (now > m_TimeNextScanSeconds)
        {
            m_TimeNextScanSeconds =
                now;
        }


        // =====================================================
        // 扫描区域偏移
        // =====================================================

        m_CurrentScanAngleStart +=
            ScanOffsetAfterPublish;

        m_CurrentScanAngleEnd +=
            ScanOffsetAfterPublish;


        while (m_CurrentScanAngleStart > 360.0f)
        {
            m_CurrentScanAngleStart -=
                360.0f;
        }

        while (m_CurrentScanAngleEnd > 360.0f)
        {
            m_CurrentScanAngleEnd -=
                360.0f;
        }

        while (m_CurrentScanAngleStart < -360.0f)
        {
            m_CurrentScanAngleStart +=
                360.0f;
        }

        while (m_CurrentScanAngleEnd < -360.0f)
        {
            m_CurrentScanAngleEnd +=
                360.0f;
        }
    }


    // =========================================================
    // Update
    // =========================================================

    public void Update()
    {
        // =====================================================
        // 判断是否应该开始一次新扫描
        // =====================================================

        if (!isScanning)
        {
            if (
                Clock.NowTimeInSeconds <
                m_TimeNextScanSeconds
            )
            {
                return;
            }

            BeginScan();
        }


        // =====================================================
        // 当前时刻理论上应该完成多少束激光
        // =====================================================

        int measurementsSoFar;

        if (TimeBetweenMeasurementsSeconds <= 0)
        {
            // 一帧完成整个扫描
            measurementsSoFar =
                NumMeasurementsPerScan;
        }
        else
        {
            measurementsSoFar =
                1 +
                Mathf.FloorToInt(
                    (float)(
                        Clock.time -
                        m_TimeLastScanBeganSeconds
                    )
                    /
                    TimeBetweenMeasurementsSeconds
                );
        }


        if (
            measurementsSoFar >
            NumMeasurementsPerScan
        )
        {
            measurementsSoFar =
                NumMeasurementsPerScan;
        }


        // =====================================================
        // 逐束扫描
        // =====================================================

        while (
            m_NumMeasurementsTaken <
            measurementsSoFar
        )
        {
            // -------------------------------------------------
            // 当前束在整个扫描范围中的比例
            // -------------------------------------------------

            float t =
                m_NumMeasurementsTaken
                /
                (float)(
                    NumMeasurementsPerScan - 1
                );


            // -------------------------------------------------
            // 当前激光束相对于雷达自身的水平角
            // -------------------------------------------------

            float yawSensorDegrees =
                Mathf.Lerp(
                    m_CurrentScanAngleStart,
                    m_CurrentScanAngleEnd,
                    t
                );


            // =================================================
            // 计算射线方向
            // =================================================

            Vector3 directionVector;


            if (KeepScanHorizontal)
            {
                /*
                 * =================================================
                 * 水平稳定模式
                 * =================================================
                 *
                 * 只继承雷达当前世界坐标系中的 Yaw。
                 *
                 * 不继承车辆 Roll / Pitch。
                 *
                 * 因此即使车辆：
                 *
                 *   加速抬头
                 *   刹车点头
                 *   经过坡面发生俯仰
                 *
                 * 激光扫描面仍然保持世界水平。
                 */

                float yawBaseDegrees =
                    transform.rotation.eulerAngles.y;


                float yawWorldDegrees =
                    yawBaseDegrees +
                    yawSensorDegrees;


                directionVector =
                    Quaternion.Euler(
                        0.0f,
                        yawWorldDegrees,
                        0.0f
                    )
                    *
                    Vector3.forward;
            }
            else
            {
                /*
                 * =================================================
                 * 完整姿态跟随模式
                 * =================================================
                 *
                 * 激光雷达完整继承 laser_link：
                 *
                 * Roll
                 * Pitch
                 * Yaw
                 *
                 * 如果车辆发生俯仰，
                 * 整个扫描平面也会发生倾斜。
                 */

                Vector3 localDirection =
                    Quaternion.Euler(
                        0.0f,
                        yawSensorDegrees,
                        0.0f
                    )
                    *
                    Vector3.forward;


                directionVector =
                    transform.TransformDirection(
                        localDirection
                    );
            }


            directionVector.Normalize();


            // =================================================
            // Raycast 起点
            // =================================================

            /*
             * 从 rangeMin 开始检测，
             * 避免雷达附近非常近的物体进入有效测距范围。
             */

            Vector3 measurementStart =
                transform.position +
                directionVector *
                RangeMetersMin;


            Ray measurementRay =
                new Ray(
                    measurementStart,
                    directionVector
                );


            float rayLength =
                RangeMetersMax -
                RangeMetersMin;


            // =================================================
            // Trigger 设置
            // =================================================

            // =================================================
            // Raycast
            // =================================================

            RaycastHit hit;


            bool foundValidMeasurement =
                Physics.Raycast(
                    measurementRay,
                    out hit,
                    rayLength,
                    m_LayerMask,
                    m_QueryTrigger
                );


            // =================================================
            // 命中障碍物
            // =================================================

            if (foundValidMeasurement)
            {
                /*
                 * hit.distance 是从 measurementStart 开始计算。
                 *
                 * 因为 measurementStart 已经向前移动了 rangeMin，
                 * 所以最终真实雷达距离：
                 *
                 * hit.distance + RangeMetersMin
                 */

                float measuredDistance =
                    hit.distance +
                    RangeMetersMin;


                ranges.Add(
                    measuredDistance
                );


                // ---------------------------------------------
                // Debug 红线
                // ---------------------------------------------

                if (ShowDebugRays)
                {
                    Debug.DrawRay(
                        transform.position,
                        directionVector *
                        measuredDistance,
                        HitRayColor,
                        DebugRayDuration
                    );
                }
            }

            // =================================================
            // 未命中障碍物
            // =================================================

            else
            {
                /*
                 * ROS LaserScan：
                 * 没有检测到有效障碍物时使用 +Infinity。
                 */

                ranges.Add(
                    float.PositiveInfinity
                );


                // ---------------------------------------------
                // Debug 绿线
                // ---------------------------------------------

                if (ShowDebugRays)
                {
                    Debug.DrawRay(
                        transform.position,
                        directionVector *
                        RangeMetersMax,
                        MissRayColor,
                        DebugRayDuration
                    );
                }
            }


            // 当前束完成
            ++m_NumMeasurementsTaken;
        }


        // =====================================================
        // 一次扫描完成
        // =====================================================

        if (
            m_NumMeasurementsTaken >=
            NumMeasurementsPerScan
        )
        {
            if (
                m_NumMeasurementsTaken >
                NumMeasurementsPerScan
            )
            {
                Debug.LogError(
                    $"LaserScan2DPublisher: 扫描数量异常。" +
                    $"当前 {m_NumMeasurementsTaken}，" +
                    $"期望 {NumMeasurementsPerScan}。"
                );
            }


            EndScan();
        }
    }

    // =========================================================
    // ROS system timestamp
    // =========================================================

    private static TimeMsg GetSystemTimeMessage()
    {
        // DateTime uses 100 ns ticks. ROS Time uses seconds + nanoseconds.
        const long UnixEpochTicks = 621355968000000000L;

        long elapsedTicks =
            DateTime.UtcNow.Ticks - UnixEpochTicks;

        return new TimeMsg
        {
            sec =
                (int)(
                    elapsedTicks /
                    TimeSpan.TicksPerSecond
                ),

            nanosec =
                (uint)(
                    (
                        elapsedTicks %
                        TimeSpan.TicksPerSecond
                    )
                    * 100L
                )
        };
    }

}

