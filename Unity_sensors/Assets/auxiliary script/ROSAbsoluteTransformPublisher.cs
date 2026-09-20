using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityEngine;


/// <summary>
/// 发布指定 Pose Source 相对于 Unity 世界原点的绝对位姿。
///
/// Unity 世界原点被视为 ROS parentFrameId（默认 map）的原点。
/// Pose Source 的世界坐标和旋转被发布为 childFrameId。
/// </summary>
public class ROSAbsoluteTransformPublisher : MonoBehaviour
{
    [Header("ROS TF")]

    [Tooltip("TF 话题。动态物体使用 /tf。")]
    public string topicName = "/tf";

    [Tooltip("代表 Unity 世界原点的 ROS 坐标系，通常填写 map。")]
    public string parentFrameId = "map";

    [Tooltip(
        "当前物体对应的 ROS 坐标系名称。" +
        "留空时自动使用当前 GameObject 的名字。"
    )]
    public string childFrameId = "";

    [Tooltip("TF 发布频率。")]
    public float publishHz = 20.0f;

    [Header("Pose Source")]

    [Tooltip(
        "真正产生传感器数据的 Transform。" +
        "例如雷达脚本挂在 2DOBJ 上，就把 2DOBJ 拖到这里。" +
        "留空时使用本脚本所在物体。"
    )]
    public Transform poseSource;

    [Tooltip(
        "开启时只发布 Pose Source 的世界 Yaw，忽略 Roll/Pitch。" +
        "必须与雷达脚本的 Keep Scan Horizontal 保持一致。"
    )]
    public bool keepFrameHorizontal = true;


    private ROSConnection ros;
    private double nextPublishTimeSeconds;
    private string resolvedParentFrameId;
    private string resolvedChildFrameId;


    void Start()
    {
        if (publishHz <= 0.0f)
        {
            publishHz = 20.0f;
        }

        if (poseSource == null)
        {
            poseSource = transform;
        }

        resolvedParentFrameId =
            NormalizeFrameId(parentFrameId);

        resolvedChildFrameId =
            NormalizeFrameId(
                string.IsNullOrWhiteSpace(childFrameId)
                ? gameObject.name
                : childFrameId
            );

        if (string.IsNullOrEmpty(resolvedParentFrameId))
        {
            Debug.LogError(
                "ROSAbsoluteTransformPublisher: Parent Frame Id 不能为空。"
            );

            enabled = false;
            return;
        }

        if (string.IsNullOrEmpty(resolvedChildFrameId))
        {
            Debug.LogError(
                "ROSAbsoluteTransformPublisher: Child Frame Id 不能为空。"
            );

            enabled = false;
            return;
        }

        if (resolvedParentFrameId == resolvedChildFrameId)
        {
            Debug.LogError(
                "ROSAbsoluteTransformPublisher: Parent 和 Child Frame Id 不能相同。"
            );

            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(topicName);

        nextPublishTimeSeconds = Clock.NowTimeInSeconds;
    }


    void Update()
    {
        if (Clock.NowTimeInSeconds < nextPublishTimeSeconds)
        {
            return;
        }

        // 跳过已经错过的周期，避免 Unity 卡顿后连续补发 TF。
        nextPublishTimeSeconds =
            Clock.NowTimeInSeconds + 1.0 / publishHz;

        PublishAbsoluteTransform();
    }


    void PublishAbsoluteTransform()
    {
        if (ros == null)
        {
            return;
        }

        /*
         * 使用 Pose Source 的世界坐标和世界旋转，并完成
         * Unity RUF 到 ROS FLU 的转换：
         *
         * ROS X =  Unity Z   （前）
         * ROS Y = -Unity X   （左）
         * ROS Z =  Unity Y   （上）
         *
         * 旋转使用四元数进行同一坐标系转换，避免手动修改欧拉角。
         */
        TransformMsg rosTransform;

        if (keepFrameHorizontal)
        {
            // 与两个雷达脚本 KeepScanHorizontal=true 时的算法一致：
            // 坐标使用完整世界坐标，旋转只保留世界 Yaw。
            float yawDegrees =
                poseSource.rotation.eulerAngles.y;

            Quaternion yawRotation =
                Quaternion.Euler(
                    0.0f,
                    yawDegrees,
                    0.0f
                );

            rosTransform =
                new TransformMsg(
                    poseSource.position.To<FLU>(),
                    yawRotation.To<FLU>()
                );
        }
        else
        {
            // 与雷达脚本 KeepScanHorizontal=false 时保持一致，
            // 发布 Pose Source 的完整世界旋转。
            rosTransform =
                poseSource.To<FLU>();
        }

        TransformStampedMsg stampedTransform =
            new TransformStampedMsg(
                new HeaderMsg(
                    new TimeStamp(Clock.time),
                    resolvedParentFrameId
                ),
                resolvedChildFrameId,
                rosTransform
            );

        TFMessageMsg tfMessage =
            new TFMessageMsg(
                new TransformStampedMsg[]
                {
                    stampedTransform
                }
            );

        ros.Publish(
            topicName,
            tfMessage
        );
    }


    static string NormalizeFrameId(string frameId)
    {
        if (string.IsNullOrWhiteSpace(frameId))
        {
            return "";
        }

        string normalized =
            frameId.Trim().Replace(" ", "_");

        // ROS 2 frame_id 通常不使用开头的斜杠。
        while (normalized.StartsWith("/"))
        {
            normalized = normalized.Substring(1);
        }

        return normalized;
    }
}
