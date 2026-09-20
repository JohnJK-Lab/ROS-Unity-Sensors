using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RGBDCameraPublisher : MonoBehaviour
{
    [Header("ROS Topics")]
    public string colorTopic = "/camera/color/image_raw";
    public string depthTopic = "/camera/depth/image_raw";
    public string colorCameraInfoTopic = "/camera/color/camera_info";
    public string depthCameraInfoTopic = "/camera/depth/camera_info";

    [Header("Frame")]
    public string frameId = "camera_link";

    [Header("Camera")]
    public Camera targetCamera;
    public int imageWidth = 320;
    public int imageHeight = 240;
    public float publishHz = 5.0f;

    [Header("Depth")]
    public float minDepth = 0.1f;
    public float maxDepth = 20.0f;

    private ROSConnection ros;

    private RenderTexture colorRenderTexture;
    private Texture2D colorTexture;

    private byte[] flippedColorData;

    private float[] depthFloatData;
    private byte[] depthByteData;

    private float publishInterval;
    private float timer = 0.0f;

    // 缓存对象，减少每次发布时的重复计算和临时对象创建。
    private Rect colorReadRect;
    private Vector3[] depthLocalRayOrigins;
    private Vector3[] depthLocalRayDirections;
    private Matrix4x4 cachedDepthProjectionMatrix;

    private bool cameraWasEnabled;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<ImageMsg>(colorTopic);
        ros.RegisterPublisher<ImageMsg>(depthTopic);
        ros.RegisterPublisher<CameraInfoMsg>(colorCameraInfoTopic);
        ros.RegisterPublisher<CameraInfoMsg>(depthCameraInfoTopic);

        if (targetCamera == null)
        {
            targetCamera = GetComponent<Camera>();
        }

        if (targetCamera == null)
        {
            Debug.LogError("RGBDCameraPublisher: 没有找到 Camera 组件！");
            enabled = false;
            return;
        }

        // ==============================
        // RGB RenderTexture
        // ==============================

        colorRenderTexture = new RenderTexture(
            imageWidth,
            imageHeight,
            24,
            RenderTextureFormat.ARGB32
        );

        colorRenderTexture.Create();

        colorTexture = new Texture2D(
            imageWidth,
            imageHeight,
            TextureFormat.RGB24,
            false
        );

        targetCamera.targetTexture = colorRenderTexture;

        // 图像只在发布周期到达时手动渲染，避免 Camera 每帧自动渲染
        // 与 targetCamera.Render() 形成重复渲染。
        cameraWasEnabled = targetCamera.enabled;
        targetCamera.enabled = false;

        // RGB24：每个像素 3 字节
        flippedColorData =
            new byte[imageWidth * imageHeight * 3];

        // Depth：每个像素 float32 = 4 字节
        depthFloatData =
            new float[imageWidth * imageHeight];

        depthByteData =
            new byte[imageWidth * imageHeight * sizeof(float)];

        publishInterval = 1.0f / publishHz;

        colorReadRect = new Rect(0, 0, imageWidth, imageHeight);

        CacheDepthRays();
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= publishInterval)
        {
            // 卡顿时不追发已经错过的帧，防止形成连续发布和网络积压。
            timer %= publishInterval;

            PublishColorImage();
            PublishDepthImage();
            PublishCameraInfo();
        }
    }

    // ============================================================
    // RGB IMAGE
    // ============================================================

    void PublishColorImage()
    {
        RenderTexture previousRT = RenderTexture.active;

        RenderTexture.active = colorRenderTexture;

        targetCamera.Render();

        colorTexture.ReadPixels(
            colorReadRect,
            0,
            0,
            false
        );

        // 后续只读取 CPU 端数据，不需要 Apply() 将纹理再次上传到 GPU。

        RenderTexture.active = previousRT;

        byte[] rawData =
            colorTexture.GetRawTextureData();

        // Unity 纹理数据从底部开始，ROS Image 从顶部开始。
        // 这里只做一次上下方向转换，不旋转或左右镜像图像。
        int rowSize = imageWidth * 3;

        for (int y = 0; y < imageHeight; y++)
        {
            int sourceRow =
                y * rowSize;

            int destinationRow =
                (imageHeight - 1 - y) * rowSize;

            System.Buffer.BlockCopy(
                rawData,
                sourceRow,
                flippedColorData,
                destinationRow,
                rowSize
            );
        }

        ImageMsg msg = new ImageMsg();

        msg.header.frame_id = frameId;

        msg.height = (uint)imageHeight;
        msg.width = (uint)imageWidth;

        msg.encoding = "rgb8";

        msg.is_bigendian = 0;

        msg.step = (uint)(imageWidth * 3);

        msg.data = flippedColorData;

        ros.Publish(
            colorTopic,
            msg
        );
    }

    // ============================================================
    // DEPTH IMAGE
    // ============================================================

    void PublishDepthImage()
    {
        /*
         * ROS 的第 0 行是图像顶部，Unity ScreenPoint 的第 0 行
         * 是图像底部，因此在生成深度数据时转换一次纵向坐标。
         */

        // 如果运行时修改了相机 FOV 或投影模式，则自动刷新射线缓存。
        if (targetCamera.projectionMatrix != cachedDepthProjectionMatrix)
        {
            CacheDepthRays();
        }

        Matrix4x4 localToWorld =
            targetCamera.transform.localToWorldMatrix;

        for (int y = 0; y < imageHeight; y++)
        {
            for (int x = 0; x < imageWidth; x++)
            {
                int index =
                    y * imageWidth + x;

                // 局部射线在初始化时计算一次。每帧只做坐标变换，
                // 避免为每个像素重复调用 ScreenPointToRay()。
                Vector3 rayOrigin =
                    localToWorld.MultiplyPoint3x4(
                        depthLocalRayOrigins[index]
                    );

                Vector3 rayDirection =
                    localToWorld.MultiplyVector(
                        depthLocalRayDirections[index]
                    ).normalized;

                /*
                 * 默认值使用 maxDepth。
                 *
                 * 不使用 PositiveInfinity，
                 * 避免 RViz 显示全黑。
                 */
                float depth = maxDepth;

                RaycastHit hit;

                if (
                    Physics.Raycast(
                        rayOrigin,
                        rayDirection,
                        out hit,
                        maxDepth
                    )
                )
                {
                    if (
                        hit.distance >= minDepth &&
                        hit.distance <= maxDepth
                    )
                    {
                        depth = hit.distance;
                    }
                }

                depthFloatData[index] = depth;
            }
        }

        // float[] 转 byte[]
        System.Buffer.BlockCopy(
            depthFloatData,
            0,
            depthByteData,
            0,
            depthByteData.Length
        );

        ImageMsg msg = new ImageMsg();

        msg.header.frame_id = frameId;

        msg.height = (uint)imageHeight;
        msg.width = (uint)imageWidth;

        /*
         * 32FC1:
         * 单通道 float32 深度
         * 单位：米
         */
        msg.encoding = "32FC1";

        msg.is_bigendian = 0;

        msg.step =
            (uint)(imageWidth * sizeof(float));

        msg.data = depthByteData;

        ros.Publish(
            depthTopic,
            msg
        );
    }

    // ============================================================
    // DEPTH RAY CACHE
    // ============================================================

    void CacheDepthRays()
    {
        cachedDepthProjectionMatrix = targetCamera.projectionMatrix;

        int pixelCount = imageWidth * imageHeight;

        depthLocalRayOrigins = new Vector3[pixelCount];
        depthLocalRayDirections = new Vector3[pixelCount];

        Transform cameraTransform = targetCamera.transform;

        for (int y = 0; y < imageHeight; y++)
        {
            int unityY = imageHeight - 1 - y;

            for (int x = 0; x < imageWidth; x++)
            {
                int index = y * imageWidth + x;

                Vector3 viewportPoint = new Vector3(
                    (x + 0.5f) / imageWidth,
                    (unityY + 0.5f) / imageHeight,
                    0.0f
                );

                Ray ray = targetCamera.ViewportPointToRay(viewportPoint);

                depthLocalRayOrigins[index] =
                    cameraTransform.InverseTransformPoint(ray.origin);

                depthLocalRayDirections[index] =
                    cameraTransform.InverseTransformDirection(ray.direction).normalized;
            }
        }
    }

    // ============================================================
    // CAMERA INFO
    // ============================================================

    void PublishCameraInfo()
    {
        /*
         * Unity Camera.fieldOfView 是垂直 FOV。
         */

        double fy =
            imageHeight /
            (
                2.0 *
                Mathf.Tan(
                    targetCamera.fieldOfView *
                    Mathf.Deg2Rad *
                    0.5f
                )
            );

        double fx =
            fy *
            ((double)imageWidth / imageHeight);

        double cx =
            (imageWidth - 1) / 2.0;

        double cy =
            (imageHeight - 1) / 2.0;

        CameraInfoMsg colorInfo =
            CreateCameraInfo(
                fx,
                fy,
                cx,
                cy
            );

        CameraInfoMsg depthInfo =
            CreateCameraInfo(
                fx,
                fy,
                cx,
                cy
            );

        ros.Publish(
            colorCameraInfoTopic,
            colorInfo
        );

        ros.Publish(
            depthCameraInfoTopic,
            depthInfo
        );
    }

    CameraInfoMsg CreateCameraInfo(
        double fx,
        double fy,
        double cx,
        double cy
    )
    {
        CameraInfoMsg info =
            new CameraInfoMsg();

        info.header.frame_id = frameId;

        info.height = (uint)imageHeight;
        info.width = (uint)imageWidth;

        info.distortion_model = "plumb_bob";

        info.d = new double[]
        {
            0, 0, 0, 0, 0
        };

        info.k = new double[]
        {
            fx, 0,  cx,
            0,  fy, cy,
            0,  0,  1
        };

        info.r = new double[]
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

        info.p = new double[]
        {
            fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  1,  0
        };

        return info;
    }

    // ============================================================
    // CLEANUP
    // ============================================================

    void OnDestroy()
    {
        if (targetCamera != null)
        {
            targetCamera.targetTexture = null;
            targetCamera.enabled = cameraWasEnabled;
        }

        if (colorRenderTexture != null)
        {
            colorRenderTexture.Release();
            Destroy(colorRenderTexture);
        }

        if (colorTexture != null)
        {
            Destroy(colorTexture);
        }
    }
}
