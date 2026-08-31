using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraImagePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/camera/image_raw";
    public string frameId = "camera_link";
    public float publishHz = 10.0f;

    [Header("Camera")]
    public Camera targetCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;

    private ROSConnection ros;

    private RenderTexture renderTexture;
    private Texture2D texture2D;

    private float publishInterval;
    private float timer = 0f;

    // 用于保存翻转后的图像数据
    private byte[] flippedImageData;

    void Start()
    {
        // 获取 ROS 连接
        ros = ROSConnection.GetOrCreateInstance();

        // 注册 ROS Image 发布器
        ros.RegisterPublisher<ImageMsg>(topicName);

        // 如果没有手动指定 Camera，就获取当前物体上的 Camera
        if (targetCamera == null)
        {
            targetCamera = GetComponent<Camera>();
        }

        if (targetCamera == null)
        {
            Debug.LogError("CameraImagePublisher: 没有找到 Camera 组件！");
            enabled = false;
            return;
        }

        // 创建 RenderTexture
        renderTexture = new RenderTexture(
            imageWidth,
            imageHeight,
            24,
            RenderTextureFormat.ARGB32
        );

        renderTexture.Create();

        // 创建 CPU 端纹理
        texture2D = new Texture2D(
            imageWidth,
            imageHeight,
            TextureFormat.RGB24,
            false
        );

        // Camera 渲染到 RenderTexture
        targetCamera.targetTexture = renderTexture;

        // RGB24 每个像素 3 字节
        flippedImageData = new byte[
            imageWidth * imageHeight * 3
        ];

        // 计算发布周期
        publishInterval = 1.0f / publishHz;
    }

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= publishInterval)
        {
            timer -= publishInterval;

            PublishImage();
        }
    }

    void PublishImage()
    {
        // 保存当前 RenderTexture
        RenderTexture previousRT = RenderTexture.active;

        // 设置当前读取目标
        RenderTexture.active = renderTexture;

        // 主动渲染相机
        targetCamera.Render();

        // 从 RenderTexture 读取图像
        texture2D.ReadPixels(
            new Rect(
                0,
                0,
                imageWidth,
                imageHeight
            ),
            0,
            0,
            false
        );

        texture2D.Apply(false);

        // 恢复 RenderTexture
        RenderTexture.active = previousRT;

        // 获取 RGB 原始数据
        byte[] rawData = texture2D.GetRawTextureData();

        /*
         * Unity Texture2D 图像方向和 ROS Image 显示方向不同。
         *
         * Unity:
         * 原点通常按左下角处理
         *
         * ROS / RViz:
         * 图像按照左上角进行显示
         *
         * 所以这里进行垂直翻转。
         */

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
                flippedImageData,
                destinationRow,
                rowSize
            );
        }

        // 创建 ROS Image 消息
        ImageMsg imageMsg = new ImageMsg();

        imageMsg.header.frame_id = frameId;

        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;

        // RGB24 对应 rgb8
        imageMsg.encoding = "rgb8";

        imageMsg.is_bigendian = 0;

        // 每行字节数量
        imageMsg.step = (uint)(imageWidth * 3);

        // 图像数据
        imageMsg.data = flippedImageData;

        // 发布
        ros.Publish(
            topicName,
            imageMsg
        );
    }

    void OnDestroy()
    {
        if (targetCamera != null)
        {
            targetCamera.targetTexture = null;
        }

        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }

        if (texture2D != null)
        {
            Destroy(texture2D);
        }
    }
}