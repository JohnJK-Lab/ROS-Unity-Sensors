using System;
using System.Collections.Generic;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;

using UnityEngine;


public class LaserScan3DPublisher : MonoBehaviour
{
    // =========================================================
    // ROS
    // =========================================================

    [Header("ROS")]

    [Tooltip("PointCloud2 发布话题")]
    public string topic = "/points";

    [Tooltip("PointCloud2 消息中的 frame_id")]
    public string FrameId = "base_scan";


    // =========================================================
    // 扫描频率
    // =========================================================

    [Header("Scan Time")]

    [Tooltip("完整点云发布频率，例如 0.1 = 10Hz")]
    public double PublishPeriodSeconds = 0.1;


    // =========================================================
    // 测距范围
    // =========================================================

    [Header("Range")]

    [Tooltip("最小测距，单位 m")]
    public float RangeMetersMin = 0.1f;

    [Tooltip("最大测距，单位 m")]
    public float RangeMetersMax = 50.0f;


    // =========================================================
    // 水平扫描
    // =========================================================

    [Header("Horizontal Scan")]

    [Tooltip("水平扫描起始角度，单位度")]
    public float HorizontalAngleStartDegrees = 0.0f;

    [Tooltip("水平扫描结束角度，单位度")]
    public float HorizontalAngleEndDegrees = -359.0f;

    [Tooltip("水平方向采样数量")]
    public int HorizontalMeasurements = 360;


    // =========================================================
    // 垂直扫描
    // =========================================================

    [Header("Vertical Scan")]

    [Tooltip("垂直最低扫描角度，单位度")]
    public float VerticalAngleMinDegrees = -15.0f;

    [Tooltip("垂直最高扫描角度，单位度")]
    public float VerticalAngleMaxDegrees = 15.0f;

    [Tooltip("垂直线束数量，例如 16、32、64")]
    public int VerticalChannels = 16;


    // =========================================================
    // 姿态模式
    // =========================================================

    [Header("Scan Orientation")]

    [Tooltip(
        "开启：忽略机器人 Roll/Pitch，只跟随 Yaw，使雷达保持水平。\n" +
        "关闭：完整跟随 lidar3d_link 的 Roll/Pitch/Yaw。"
    )]
    public bool KeepScanHorizontal = true;


    // =========================================================
    // 碰撞检测
    // =========================================================

    [Header("Collision Detection")]

    [Tooltip("参与激光检测的 Unity Layer")]
    public string LayerMaskName = "TurtleBot3Manual";

    [Tooltip("是否忽略 Trigger Collider")]
    public bool IgnoreTrigger = true;


    // =========================================================
    // Debug
    // =========================================================

    [Header("Debug")]

    [Tooltip("是否在 Scene 里画出激光")]
    public bool ShowDebugRays = true;

    [Tooltip("命中障碍物的激光颜色")]
    public Color HitRayColor = Color.red;

    [Tooltip("未命中的激光颜色")]
    public Color MissRayColor = Color.green;

    [Tooltip("是否显示未命中的绿色激光。3D 雷达束数很多时建议关闭")]
    public bool ShowMissRays = false;

    [Tooltip("Debug 射线显示时间")]
    public float DebugRayDuration = 0.05f;


    // =========================================================
    // PointCloud
    // =========================================================

    [Header("Point Cloud")]

    [Tooltip("是否在 PointCloud2 中加入 intensity 字段")]
    public bool PublishIntensity = true;

    [Tooltip("命中点的默认 intensity")]
    public float DefaultIntensity = 1.0f;


    // =========================================================
    // 内部变量
    // =========================================================

    private ROSConnection m_Ros;

    private double m_TimeNextScanSeconds;

    private int m_LayerMask;


    // =========================================================
    // 点结构
    // =========================================================

    private struct LidarPoint
    {
        public float x;
        public float y;
        public float z;
        public float intensity;

        public LidarPoint(
            float x,
            float y,
            float z,
            float intensity
        )
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.intensity = intensity;
        }
    }


    // =========================================================
    // Start
    // =========================================================

    void Start()
    {
        // -----------------------------
        // 参数检查
        // -----------------------------

        if (HorizontalMeasurements < 2)
        {
            HorizontalMeasurements = 2;
        }

        if (VerticalChannels < 1)
        {
            VerticalChannels = 1;
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


        // -----------------------------
        // ROS
        // -----------------------------

        m_Ros =
            ROSConnection.GetOrCreateInstance();

        m_Ros.RegisterPublisher<PointCloud2Msg>(
            topic
        );


        // -----------------------------
        // Layer
        // -----------------------------

        m_LayerMask =
            LayerMask.GetMask(
                LayerMaskName
            );

        if (m_LayerMask == 0)
        {
            Debug.LogWarning(
                "LaserScan3DPublisher: LayerMask '" +
                LayerMaskName +
                "' 不存在，将检测所有 Layer。"
            );

            m_LayerMask = ~0;
        }


        m_TimeNextScanSeconds =
            Clock.Now +
            PublishPeriodSeconds;
    }


    // =========================================================
    // Update
    // =========================================================

    void Update()
    {
        if (
            Clock.NowTimeInSeconds <
            m_TimeNextScanSeconds
        )
        {
            return;
        }

        m_TimeNextScanSeconds =
            Clock.Now +
            PublishPeriodSeconds;

        ScanAndPublish();
    }


    // =========================================================
    // 扫描
    // =========================================================

    void ScanAndPublish()
    {
        List<LidarPoint> points =
            new List<LidarPoint>(
                HorizontalMeasurements *
                VerticalChannels
            );


        // Trigger 设置
        QueryTriggerInteraction triggerMode =
            IgnoreTrigger
            ?
            QueryTriggerInteraction.Ignore
            :
            QueryTriggerInteraction.Collide;


        // =====================================================
        // 垂直线
        // =====================================================

        for (
            int verticalIndex = 0;
            verticalIndex < VerticalChannels;
            verticalIndex++
        )
        {
            float verticalT;

            if (VerticalChannels == 1)
            {
                verticalT = 0.5f;
            }
            else
            {
                verticalT =
                    verticalIndex /
                    (float)(VerticalChannels - 1);
            }


            float verticalAngle =
                Mathf.Lerp(
                    VerticalAngleMinDegrees,
                    VerticalAngleMaxDegrees,
                    verticalT
                );


            // =================================================
            // 水平扫描
            // =================================================

            for (
                int horizontalIndex = 0;
                horizontalIndex < HorizontalMeasurements;
                horizontalIndex++
            )
            {
                float horizontalT =
                    horizontalIndex /
                    (float)(HorizontalMeasurements - 1);


                float horizontalAngle =
                    Mathf.Lerp(
                        HorizontalAngleStartDegrees,
                        HorizontalAngleEndDegrees,
                        horizontalT
                    );


                // =============================================
                // 雷达自身局部射线
                // =============================================

                /*
                 * Unity：
                 *
                 * X = 右
                 * Y = 上
                 * Z = 前
                 *
                 * horizontalAngle：
                 * 绕 Y 轴
                 *
                 * verticalAngle：
                 * 绕 X 轴
                 */

                Vector3 localDirection =
                    Quaternion.Euler(
                        -verticalAngle,
                        horizontalAngle,
                        0.0f
                    )
                    *
                    Vector3.forward;


                Vector3 worldDirection;


                // =============================================
                // 水平稳定模式
                // =============================================

                if (KeepScanHorizontal)
                {
                    /*
                     * 只使用雷达的 Yaw。
                     *
                     * 忽略车辆 Roll/Pitch。
                     */

                    float yawDegrees =
                        transform.rotation.eulerAngles.y;


                    Quaternion yawRotation =
                        Quaternion.Euler(
                            0.0f,
                            yawDegrees,
                            0.0f
                        );


                    worldDirection =
                        yawRotation *
                        localDirection;
                }

                // =============================================
                // 完整姿态跟随
                // =============================================

                else
                {
                    worldDirection =
                        transform.TransformDirection(
                            localDirection
                        );
                }


                worldDirection.Normalize();


                // =============================================
                // Raycast 起点
                // =============================================

                Vector3 measurementStart =
                    transform.position +
                    worldDirection *
                    RangeMetersMin;


                float rayLength =
                    RangeMetersMax -
                    RangeMetersMin;


                RaycastHit hit;


                bool foundHit =
                    Physics.Raycast(
                        measurementStart,
                        worldDirection,
                        out hit,
                        rayLength,
                        m_LayerMask,
                        triggerMode
                    );


                // =============================================
                // 命中
                // =============================================

                if (foundHit)
                {
                    float measuredDistance =
                        hit.distance +
                        RangeMetersMin;


                    /*
                     * 先得到世界坐标命中点。
                     */

                    Vector3 hitWorldPosition =
                        transform.position +
                        worldDirection *
                        measuredDistance;


                    /*
                     * 再转换到 LiDAR 自身坐标系。
                     *
                     * PointCloud2 中的点应该相对于 FrameId，
                     * 不能直接发 Unity 世界坐标。
                     */


                    Vector3 localHitPoint;


                    if (KeepScanHorizontal)
                    {
                        /*
                         * 水平稳定模式下，
                         * PointCloud 的坐标也使用仅 Yaw 的雷达坐标系。
                         */

                        float yawDegrees =
                            transform.rotation.eulerAngles.y;


                        Quaternion yawRotation =
                            Quaternion.Euler(
                                0.0f,
                                yawDegrees,
                                0.0f
                            );


                        Vector3 relativeWorld =
                            hitWorldPosition -
                            transform.position;


                        localHitPoint =
                            Quaternion.Inverse(
                                yawRotation
                            )
                            *
                            relativeWorld;
                    }
                    else
                    {
                        /*
                         * 正常刚性安装模式。
                         */

                        localHitPoint =
                            transform.InverseTransformPoint(
                                hitWorldPosition
                            );
                    }


                    // =========================================
                    // Unity → ROS FLU
                    // =========================================

                    /*
                     * Unity：
                     *
                     * X = Right
                     * Y = Up
                     * Z = Forward
                     *
                     *
                     * ROS FLU：
                     *
                     * X = Forward
                     * Y = Left
                     * Z = Up
                     *
                     *
                     * 所以：
                     *
                     * ROS X = Unity Z
                     * ROS Y = -Unity X
                     * ROS Z = Unity Y
                     */

                    float rosX =
                        localHitPoint.z;

                    float rosY =
                        -localHitPoint.x;

                    float rosZ =
                        localHitPoint.y;


                    points.Add(
                        new LidarPoint(
                            rosX,
                            rosY,
                            rosZ,
                            DefaultIntensity
                        )
                    );


                    // =========================================
                    // Debug 红线
                    // =========================================

                    if (ShowDebugRays)
                    {
                        Debug.DrawRay(
                            transform.position,
                            worldDirection *
                            measuredDistance,
                            HitRayColor,
                            DebugRayDuration
                        );
                    }
                }

                // =============================================
                // 未命中
                // =============================================

                else
                {
                    if (
                        ShowDebugRays &&
                        ShowMissRays
                    )
                    {
                        Debug.DrawRay(
                            transform.position,
                            worldDirection *
                            RangeMetersMax,
                            MissRayColor,
                            DebugRayDuration
                        );
                    }
                }
            }
        }


        // =====================================================
        // 发布 PointCloud2
        // =====================================================

        PublishPointCloud(
            points
        );
    }


    // =========================================================
    // PointCloud2
    // =========================================================

    void PublishPointCloud(
        List<LidarPoint> points
    )
    {
        var timestamp =
            new TimeStamp(
                Clock.time
            );


        // =====================================================
        // PointField
        // =====================================================

        PointFieldMsg[] fields;


        uint pointStep;


        if (PublishIntensity)
        {
            /*
             * 每个点：
             *
             * x         float32  4 byte
             * y         float32  4 byte
             * z         float32  4 byte
             * intensity float32  4 byte
             *
             * 总共 16 byte
             */

            fields =
                new PointFieldMsg[]
                {
                    new PointFieldMsg
                    {
                        name = "x",
                        offset = 0,
                        datatype = 7,
                        count = 1
                    },

                    new PointFieldMsg
                    {
                        name = "y",
                        offset = 4,
                        datatype = 7,
                        count = 1
                    },

                    new PointFieldMsg
                    {
                        name = "z",
                        offset = 8,
                        datatype = 7,
                        count = 1
                    },

                    new PointFieldMsg
                    {
                        name = "intensity",
                        offset = 12,
                        datatype = 7,
                        count = 1
                    }
                };


            pointStep = 16;
        }
        else
        {
            fields =
                new PointFieldMsg[]
                {
                    new PointFieldMsg
                    {
                        name = "x",
                        offset = 0,
                        datatype = 7,
                        count = 1
                    },

                    new PointFieldMsg
                    {
                        name = "y",
                        offset = 4,
                        datatype = 7,
                        count = 1
                    },

                    new PointFieldMsg
                    {
                        name = "z",
                        offset = 8,
                        datatype = 7,
                        count = 1
                    }
                };


            pointStep = 12;
        }


        // =====================================================
        // Byte Data
        // =====================================================

        byte[] data =
            new byte[
                points.Count *
                pointStep
            ];


        for (
            int i = 0;
            i < points.Count;
            i++
        )
        {
            int offset =
                i *
                (int)pointStep;


            WriteFloat(
                data,
                offset,
                points[i].x
            );


            WriteFloat(
                data,
                offset + 4,
                points[i].y
            );


            WriteFloat(
                data,
                offset + 8,
                points[i].z
            );


            if (PublishIntensity)
            {
                WriteFloat(
                    data,
                    offset + 12,
                    points[i].intensity
                );
            }
        }


        // =====================================================
        // PointCloud2 消息
        // =====================================================

        PointCloud2Msg msg =
            new PointCloud2Msg
            {
                header =
                    new HeaderMsg
                    {
                        frame_id =
                            FrameId,

                        stamp =
                            new TimeMsg
                            {
                                sec =
                                    timestamp.Seconds,

                                nanosec =
                                    timestamp.NanoSeconds
                            }
                    },


                // 无序点云
                height = 1,

                width =
                    (uint)points.Count,


                fields =
                    fields,


                is_bigendian =
                    false,


                point_step =
                    pointStep,


                row_step =
                    pointStep *
                    (uint)points.Count,


                data =
                    data,


                /*
                 * 我们只把命中的有效点放进点云，
                 * 所以这里可以设 true。
                 */

                is_dense =
                    true
            };


        m_Ros.Publish(
            topic,
            msg
        );
    }


    // =========================================================
    // float → byte[]
    // =========================================================

    private void WriteFloat(
        byte[] destination,
        int offset,
        float value
    )
    {
        byte[] bytes =
            BitConverter.GetBytes(
                value
            );


        /*
         * ROS 一般使用 little-endian。
         */

        if (!BitConverter.IsLittleEndian)
        {
            Array.Reverse(
                bytes
            );
        }


        Buffer.BlockCopy(
            bytes,
            0,
            destination,
            offset,
            4
        );
    }
}