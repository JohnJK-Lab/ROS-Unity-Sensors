using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityEngine;


/// <summary>
/// Publishes the absolute TF of the current LiDAR link relative to
/// the Unity world origin.
///
/// Attach this component and the LiDAR publisher component to the
/// same GameObject. Their frame IDs and horizontal settings must match.
/// </summary>
public class LidarTFPublisher : MonoBehaviour
{
    [Header("ROS TF")]

    [Tooltip("Dynamic TF topic. Normally keep this as /tf.")]
    public string topicName = "/tf";

    [Tooltip("ROS frame representing the Unity world origin.")]
    public string parentFrameId = "map";

    [Tooltip(
        "ROS frame of this LiDAR link. " +
        "When empty, the current GameObject name is used."
    )]
    public string childFrameId = "";

    [Tooltip("TF publication frequency in Hz.")]
    public float publishHz = 20.0f;

    [Header("Rotation")]

    [Tooltip(
        "When enabled, publish world yaw only and ignore roll/pitch. " +
        "This must match Keep Scan Horizontal in the LiDAR publisher."
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
                "LidarTFPublisher: Parent Frame Id cannot be empty."
            );

            enabled = false;
            return;
        }

        if (string.IsNullOrEmpty(resolvedChildFrameId))
        {
            Debug.LogError(
                "LidarTFPublisher: Child Frame Id cannot be empty."
            );

            enabled = false;
            return;
        }

        if (resolvedParentFrameId == resolvedChildFrameId)
        {
            Debug.LogError(
                "LidarTFPublisher: Parent and Child Frame Id cannot be identical."
            );

            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(topicName);

        // Publish immediately on the first Update.
        nextPublishTimeSeconds = Clock.NowTimeInSeconds;
    }


    void Update()
    {
        if (Clock.NowTimeInSeconds < nextPublishTimeSeconds)
        {
            return;
        }

        // Skip missed intervals instead of sending a burst after a stall.
        nextPublishTimeSeconds =
            Clock.NowTimeInSeconds + 1.0 / publishHz;

        PublishTransform();
    }


    void PublishTransform()
    {
        if (ros == null)
        {
            return;
        }

        TransformMsg transformMessage;

        if (keepFrameHorizontal)
        {
            // Match KeepScanHorizontal=true in the LiDAR publishers.
            float yawDegrees =
                transform.rotation.eulerAngles.y;

            Quaternion yawRotation =
                Quaternion.Euler(
                    0.0f,
                    yawDegrees,
                    0.0f
                );

            transformMessage =
                new TransformMsg(
                    transform.position.To<FLU>(),
                    yawRotation.To<FLU>()
                );
        }
        else
        {
            // Match KeepScanHorizontal=false in the LiDAR publishers.
            transformMessage =
                transform.To<FLU>();
        }

        /*
         * Unity RUF to ROS FLU conversion:
         *
         * ROS X =  Unity Z
         * ROS Y = -Unity X
         * ROS Z =  Unity Y
         *
         * To<FLU>() also converts the quaternion rotation.
         */
        TransformStampedMsg stampedTransform =
            new TransformStampedMsg(
                new HeaderMsg(
                    GetSystemTimeMessage(),
                    resolvedParentFrameId
                ),
                resolvedChildFrameId,
                transformMessage
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

        // ROS 2 frame IDs should not start with a slash.
        while (normalized.StartsWith("/"))
        {
            normalized = normalized.Substring(1);
        }

        return normalized;
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


