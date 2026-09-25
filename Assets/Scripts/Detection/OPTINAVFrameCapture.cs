using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace OPTINAV.Detection
{
    /// <summary>
    /// Captures post-processed frames from the OPTINAV virtual camera, encodes them to JPEG,
    /// and streams them over a localhost TCP socket to external computer vision pipelines (Python/OpenCV).
    /// Uses AsyncGPUReadback to avoid GPU/CPU stalls and offloads network I/O to a background thread.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [DisallowMultipleComponent]
    public class OPTINAVFrameCapture : MonoBehaviour
    {
        [Header("Capture Configuration")]
        [Tooltip("Width of the captured frame for the external CV pipeline")]
        [SerializeField] private int captureWidth = 640;

        [Tooltip("Height of the captured frame for the external CV pipeline")]
        [SerializeField] private int captureHeight = 360;

        [Tooltip("Target capture rate in frames per second")]
        [Range(1, 120)]
        [SerializeField] private int targetCaptureFPS = 30;

        [Tooltip("JPEG compression quality (1-100)")]
        [Range(1, 100)]
        [SerializeField] private int jpegQuality = 75;

        [Tooltip("Flip captured image vertically if GPU coordinate convention is inverted")]
        [SerializeField] private bool flipVertical = false;

        [Header("Localhost Streaming")]
        [Tooltip("Localhost TCP port to listen on")]
        [SerializeField] private int streamPort = 9001;

        [Tooltip("Maximum frames queued for transmission before dropping oldest to prevent latency lag")]
        [SerializeField] private int maxQueueSize = 2;

        [Header("Telemetry (Live)")]
        [SerializeField] private float currentCaptureFPS = 0f;
        [SerializeField] private uint totalCapturedFrames = 0;
        [SerializeField] private uint totalDroppedFrames = 0;
        [SerializeField] private float lastCaptureLatencyMs = 0f;
        [SerializeField] private double lastCaptureTimestamp = 0;
        [SerializeField] private Vector2Int currentCaptureResolution;
        [SerializeField] private bool clientConnected = false;

        // Public Telemetry APIs for M6 Telemetry / CV inspection
        public float CaptureFPS => currentCaptureFPS;
        public uint CapturedFrameCount => totalCapturedFrames;
        public uint DroppedFrameCount => totalDroppedFrames;
        public float CaptureLatencyMs => lastCaptureLatencyMs;
        public double LastFrameTimestamp => lastCaptureTimestamp;
        public Vector2Int CurrentCaptureResolution => currentCaptureResolution;
        public bool IsClientConnected => clientConnected;

        // Camera and graphics resources
        private Camera targetCamera;
        private RenderTexture captureRT;
        private Texture2D captureTexture;
        private byte[] flippedRowBuffer;
        private readonly byte[] headerBuffer = new byte[24];

        // State & Timing
        private bool isReadbackPending = false;
        private double nextCaptureTime = 0;
        private double pendingCaptureTimestamp = 0;
        private int fpsCounter = 0;
        private double fpsWindowStart = 0;

        // Network streaming thread & queue
        private struct QueuedFrame
        {
            public uint FrameId;
            public double Timestamp;
            public ushort Width;
            public ushort Height;
            public byte[] JpegBytes;
        }

        private readonly ConcurrentQueue<QueuedFrame> frameQueue = new ConcurrentQueue<QueuedFrame>();
        private Thread networkThread;
        private volatile bool isNetworkRunning = false;
        private TcpListener tcpListener;
        private TcpClient activeClient;
        private NetworkStream activeStream;
        private readonly AutoResetEvent frameAvailableSignal = new AutoResetEvent(false);

        [Tooltip("Master toggle for capturing frames without stopping network server")]
        [SerializeField] private bool captureEnabled = true;

        public bool CaptureEnabled { get => captureEnabled; set => captureEnabled = value; }

        private void Awake()
        {
            Application.runInBackground = true;
            targetCamera = GetComponent<Camera>();
            currentCaptureResolution = new Vector2Int(captureWidth, captureHeight);
            InitializeGraphicsBuffers();
        }

        private void OnEnable()
        {
            if (targetCamera == null)
            {
                targetCamera = GetComponent<Camera>();
            }

            InitializeGraphicsBuffers();
            StartNetworkServer();

            // Register frame capture action with Unity SRP's CameraCaptureBridge
            CameraCaptureBridge.enabled = true;
            CameraCaptureBridge.AddCaptureAction(targetCamera, OnCameraCaptureAction);
        }

        private void OnDisable()
        {
            isReadbackPending = false;
            // Unregister frame capture action
            if (targetCamera != null)
            {
                CameraCaptureBridge.RemoveCaptureAction(targetCamera, OnCameraCaptureAction);
            }

            StopNetworkServer();
            ReleaseGraphicsBuffers();
        }

        private void OnDestroy()
        {
            StopNetworkServer();
            ReleaseGraphicsBuffers();
        }

        private void InitializeGraphicsBuffers()
        {
            // Allocate downscaled RenderTexture if not already allocated or if resolution changed
            if (captureRT == null || captureRT.width != captureWidth || captureRT.height != captureHeight)
            {
                if (captureRT != null)
                {
                    captureRT.Release();
                    Destroy(captureRT);
                }

                captureRT = new RenderTexture(captureWidth, captureHeight, 0, RenderTextureFormat.ARGB32)
                {
                    name = "OPTINAV_CaptureRT",
                    filterMode = FilterMode.Bilinear,
                    wrapMode = TextureWrapMode.Clamp,
                    useMipMap = false,
                    autoGenerateMips = false
                };
                captureRT.Create();
            }

            // Allocate reusable Texture2D for JPEG encoding
            if (captureTexture == null || captureTexture.width != captureWidth || captureTexture.height != captureHeight)
            {
                if (captureTexture != null)
                {
                    Destroy(captureTexture);
                }

                captureTexture = new Texture2D(captureWidth, captureHeight, TextureFormat.RGBA32, false)
                {
                    name = "OPTINAV_CaptureTexture",
                    filterMode = FilterMode.Point
                };

                flippedRowBuffer = new byte[captureWidth * captureHeight * 4];
            }

            currentCaptureResolution = new Vector2Int(captureWidth, captureHeight);
        }

        private void ReleaseGraphicsBuffers()
        {
            if (captureRT != null)
            {
                captureRT.Release();
                Destroy(captureRT);
                captureRT = null;
            }

            if (captureTexture != null)
            {
                Destroy(captureTexture);
                captureTexture = null;
            }

            flippedRowBuffer = null;
        }

        /// <summary>
        /// Hooked directly into Universal RP rendering pass via CameraCaptureBridge.
        /// Invoked after post-processing and full-screen disturbance passes have rendered.
        /// </summary>
        private void OnCameraCaptureAction(RenderTargetIdentifier source, CommandBuffer cmd)
        {
            if (!Application.isPlaying || !enabled || !captureEnabled) return;

            double currentTime = Time.unscaledTimeAsDouble;
            if (currentTime < nextCaptureTime)
            {
                return;
            }

            // If a previous readback is still in progress, drop this frame to prevent queue buildup
            if (isReadbackPending)
            {
                totalDroppedFrames++;
                return;
            }

            // Calculate next capture trigger time
            float interval = targetCaptureFPS > 0 ? (1f / targetCaptureFPS) : 0.0333f;
            nextCaptureTime = currentTime + interval;

            // Blit the rendered camera output (including post-processing) into downscaled capture RT
            cmd.Blit(source, captureRT);

            isReadbackPending = true;
            pendingCaptureTimestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

            // Request asynchronous GPU readback without CPU stall
            cmd.RequestAsyncReadback(captureRT, 0, TextureFormat.RGBA32, OnAsyncReadbackComplete);
        }

        private void OnAsyncReadbackComplete(AsyncGPUReadbackRequest request)
        {
            if (this == null) return;
            isReadbackPending = false;

            if (request.hasError || !request.done || !isActiveAndEnabled || !captureEnabled)
            {
                totalDroppedFrames++;
                return;
            }

            double now = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
            lastCaptureLatencyMs = (float)((now - pendingCaptureTimestamp) * 1000.0);
            lastCaptureTimestamp = pendingCaptureTimestamp;
            totalCapturedFrames++;

            // Measure capture FPS
            fpsCounter++;
            if (now - fpsWindowStart >= 1.0)
            {
                currentCaptureFPS = (float)(fpsCounter / (now - fpsWindowStart));
                fpsCounter = 0;
                fpsWindowStart = now;
            }

            // Retrieve raw RGBA32 pixels from GPU memory
            NativeArray<byte> rawData = request.GetData<byte>();

            if (flipVertical && flippedRowBuffer != null)
            {
                int stride = captureWidth * 4;
                for (int y = 0; y < captureHeight; y++)
                {
                    int srcRow = y * stride;
                    int dstRow = (captureHeight - 1 - y) * stride;
                    NativeArray<byte>.Copy(rawData, srcRow, flippedRowBuffer, dstRow, stride);
                }
                captureTexture.LoadRawTextureData(flippedRowBuffer);
            }
            else
            {
                captureTexture.LoadRawTextureData(rawData);
            }

            captureTexture.Apply(false, false);

            // Encode to JPEG using reusable Texture2D
            byte[] jpegBytes = captureTexture.EncodeToJPG(jpegQuality);

            // Queue frame for network transmission if client is connected
            if (clientConnected)
            {
                while (frameQueue.Count >= maxQueueSize && frameQueue.TryDequeue(out _))
                {
                    totalDroppedFrames++;
                }

                frameQueue.Enqueue(new QueuedFrame
                {
                    FrameId = totalCapturedFrames,
                    Timestamp = lastCaptureTimestamp,
                    Width = (ushort)captureWidth,
                    Height = (ushort)captureHeight,
                    JpegBytes = jpegBytes
                });

                frameAvailableSignal.Set();
            }
        }

        #region Localhost TCP Transport

        private void StartNetworkServer()
        {
            if (isNetworkRunning) return;

            isNetworkRunning = true;
            try
            {
                tcpListener = new TcpListener(IPAddress.Loopback, streamPort);
                tcpListener.Start();
                Debug.Log($"[OPTINAVFrameCapture] Localhost TCP streaming server listening on 127.0.0.1:{streamPort}");

                networkThread = new Thread(NetworkStreamingWorker)
                {
                    Name = "OPTINAV_TCP_Streamer",
                    IsBackground = true
                };
                networkThread.Start();
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OPTINAVFrameCapture] Failed to start TCP listener on port {streamPort}: {ex.Message}");
            }
        }

        private void StopNetworkServer()
        {
            isNetworkRunning = false;
            frameAvailableSignal.Set();

            try
            {
                activeStream?.Close();
                activeStream = null;
                activeClient?.Close();
                activeClient = null;
                tcpListener?.Stop();
                tcpListener = null;
            }
            catch (Exception)
            {
                // Silent catch on shutdown
            }

            if (networkThread != null && networkThread.IsAlive)
            {
                networkThread.Join(500);
                networkThread = null;
            }

            clientConnected = false;
            while (frameQueue.TryDequeue(out _)) { }
        }

        private void NetworkStreamingWorker()
        {
            while (isNetworkRunning)
            {
                try
                {
                    if (activeClient == null || !activeClient.Connected)
                    {
                        clientConnected = false;
                        if (tcpListener != null && tcpListener.Pending())
                        {
                            activeClient = tcpListener.AcceptTcpClient();
                            activeClient.NoDelay = true;
                            activeClient.SendBufferSize = 65536;
                            activeStream = activeClient.GetStream();
                            clientConnected = true;
                            Debug.Log($"[OPTINAVFrameCapture] Python CV receiver connected from {activeClient.Client.RemoteEndPoint}");
                        }
                        else
                        {
                            Thread.Sleep(50);
                            continue;
                        }
                    }

                    // Wait for new frame or timeout
                    if (frameAvailableSignal.WaitOne(50))
                    {
                        while (frameQueue.TryDequeue(out QueuedFrame frame))
                        {
                            if (!isNetworkRunning || activeStream == null) break;

                            WriteFrameHeader(headerBuffer, frame.FrameId, frame.Timestamp, frame.Width, frame.Height, (uint)frame.JpegBytes.Length);

                            // Send fixed 24-byte header
                            activeStream.Write(headerBuffer, 0, 24);

                            // Send JPEG payload
                            activeStream.Write(frame.JpegBytes, 0, frame.JpegBytes.Length);
                            activeStream.Flush();
                        }
                    }
                }
                catch (IOException)
                {
                    // Client disconnected or pipe broke
                    CleanupActiveConnection();
                }
                catch (SocketException)
                {
                    // Client disconnected
                    CleanupActiveConnection();
                }
                catch (Exception ex)
                {
                    if (isNetworkRunning)
                    {
                        Debug.LogWarning($"[OPTINAVFrameCapture] Network streaming notice: {ex.Message}");
                        CleanupActiveConnection();
                    }
                }
            }
        }

        private void CleanupActiveConnection()
        {
            clientConnected = false;
            try
            {
                activeStream?.Close();
                activeStream = null;
                activeClient?.Close();
                activeClient = null;
            }
            catch (Exception) { }

            Debug.Log("[OPTINAVFrameCapture] Receiver disconnected. Waiting for reconnection...");
        }

        /// <summary>
        /// Writes 24-byte binary frame header in big-endian network byte order.
        /// Format: "!4sIdHHI"
        /// Magic (4), FrameID (4), Timestamp (8), Width (2), Height (2), PayloadLength (4)
        /// </summary>
        private static void WriteFrameHeader(byte[] buffer, uint frameId, double timestamp, ushort width, ushort height, uint payloadLength)
        {
            // Magic: "OPT1"
            buffer[0] = (byte)'O';
            buffer[1] = (byte)'P';
            buffer[2] = (byte)'T';
            buffer[3] = (byte)'1';

            // Frame ID (uint32 big-endian)
            buffer[4] = (byte)((frameId >> 24) & 0xFF);
            buffer[5] = (byte)((frameId >> 16) & 0xFF);
            buffer[6] = (byte)((frameId >> 8) & 0xFF);
            buffer[7] = (byte)(frameId & 0xFF);

            // Timestamp (double, IEEE 754, 8 bytes big-endian)
            ulong timeBits = (ulong)BitConverter.DoubleToInt64Bits(timestamp);
            buffer[8] = (byte)((timeBits >> 56) & 0xFF);
            buffer[9] = (byte)((timeBits >> 48) & 0xFF);
            buffer[10] = (byte)((timeBits >> 40) & 0xFF);
            buffer[11] = (byte)((timeBits >> 32) & 0xFF);
            buffer[12] = (byte)((timeBits >> 24) & 0xFF);
            buffer[13] = (byte)((timeBits >> 16) & 0xFF);
            buffer[14] = (byte)((timeBits >> 8) & 0xFF);
            buffer[15] = (byte)(timeBits & 0xFF);

            // Width (uint16 big-endian)
            buffer[16] = (byte)((width >> 8) & 0xFF);
            buffer[17] = (byte)(width & 0xFF);

            // Height (uint16 big-endian)
            buffer[18] = (byte)((height >> 8) & 0xFF);
            buffer[19] = (byte)(height & 0xFF);

            // Payload length (uint32 big-endian)
            buffer[20] = (byte)((payloadLength >> 24) & 0xFF);
            buffer[21] = (byte)((payloadLength >> 16) & 0xFF);
            buffer[22] = (byte)((payloadLength >> 8) & 0xFF);
            buffer[23] = (byte)(payloadLength & 0xFF);
        }

        #endregion
    }
}
