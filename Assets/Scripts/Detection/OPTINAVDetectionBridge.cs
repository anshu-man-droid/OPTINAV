using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace OPTINAV.Detection
{
    /// <summary>
    /// Contract for optical beacon detection results produced by the computer-vision detector (Member 3).
    /// Used by Unity visualization, benchmarking, and the Member 4 tracking/Kalman subsystem.
    /// Coordinate convention:
    /// - Pixel coordinates: [0, width) x [0, height) with (0,0) at top-left.
    /// - Normalized coordinates: [0, 1] x [0, 1] with (0,0) at top-left.
    /// When detected is false, pixelCenterX/Y and normalizedCenterX/Y are set to -1, and confidence is 0.
    /// </summary>
    [Serializable]
    public struct DetectionResult
    {
        public long frameId;
        public bool detected;
        public float pixelCenterX;
        public float pixelCenterY;
        public float normalizedCenterX;
        public float normalizedCenterY;
        public float boundingBoxX;
        public float boundingBoxY;
        public float boundingBoxWidth;
        public float boundingBoxHeight;
        public float confidence;
        public float latencyMs;
        public double captureTimestamp;
        public double detectionTimestamp;
        public float spotRadius;

        public static DetectionResult CreateEmpty(long fid = 0)
        {
            return new DetectionResult
            {
                frameId = fid,
                detected = false,
                pixelCenterX = -1f,
                pixelCenterY = -1f,
                normalizedCenterX = -1f,
                normalizedCenterY = -1f,
                boundingBoxX = 0f,
                boundingBoxY = 0f,
                boundingBoxWidth = 0f,
                boundingBoxHeight = 0f,
                confidence = 0f,
                latencyMs = 0f,
                captureTimestamp = 0,
                detectionTimestamp = 0,
                spotRadius = 0f
            };
        }
    }

    /// <summary>
    /// High-performance localhost TCP bridge for receiving computer-vision detection results from Python.
    /// Operates asynchronously on a background thread and surfaces thread-safe results to the Unity main thread.
    /// </summary>
    [DisallowMultipleComponent]
    public class OPTINAVDetectionBridge : MonoBehaviour
    {
        public static OPTINAVDetectionBridge Instance { get; private set; }

        [Header("Network Configuration")]
        [Tooltip("Localhost TCP port to listen for Python detection results")]
        [SerializeField] private int listenPort = 9002;

        [Header("Telemetry (Live)")]
        [SerializeField] private bool clientConnected = false;
        [SerializeField] private long totalResultsReceived = 0;
        [SerializeField] private float detectionFPS = 0f;
        [SerializeField] private DetectionResult latestResult = DetectionResult.CreateEmpty();

        // Public APIs for Member 4 (Kalman / Tracking) and Visualizers
        public DetectionResult LatestResult => latestResult;
        public bool IsDetected => latestResult.detected;
        public Vector2 PixelCenter => new Vector2(latestResult.pixelCenterX, latestResult.pixelCenterY);
        public Vector2 NormalizedCenter => new Vector2(latestResult.normalizedCenterX, latestResult.normalizedCenterY);
        public Rect BoundingBox => new Rect(latestResult.boundingBoxX, latestResult.boundingBoxY, latestResult.boundingBoxWidth, latestResult.boundingBoxHeight);
        public float Confidence => latestResult.confidence;
        public float DetectionLatencyMs => latestResult.latencyMs;
        public long FrameId => latestResult.frameId;
        public double Timestamp => latestResult.detectionTimestamp;
        public float DetectionFPS => detectionFPS;
        public long TotalResultsReceived => totalResultsReceived;
        public bool IsClientConnected => clientConnected;

        public event Action<DetectionResult> OnDetectionReceived;

        // Background worker state
        private TcpListener tcpListener;
        private Thread networkThread;
        private volatile bool isRunning = false;
        private readonly ConcurrentQueue<DetectionResult> incomingResults = new ConcurrentQueue<DetectionResult>();

        // FPS calculation state
        private int fpsCounter = 0;
        private float lastFpsTime = 0f;

        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
            }
            else if (Instance != this)
            {
                if (Application.isPlaying)
                {
                    Destroy(gameObject);
                    return;
                }
            }

            StartNetworkListener();
        }

        private void StartNetworkListener()
        {
            if (isRunning) return;

            isRunning = true;
            networkThread = new Thread(NetworkListenLoop)
            {
                Name = "OPTINAV_DetectionBridge_Thread",
                IsBackground = true
            };
            networkThread.Start();
        }

        private void StopNetworkListener()
        {
            isRunning = false;

            try
            {
                tcpListener?.Stop();
            }
            catch (Exception) { }

            if (networkThread != null && networkThread.IsAlive)
            {
                networkThread.Join(500);
            }

            tcpListener = null;
            networkThread = null;
            clientConnected = false;
        }

        private void NetworkListenLoop()
        {
            try
            {
                tcpListener = new TcpListener(IPAddress.Loopback, listenPort);
                tcpListener.Server.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                tcpListener.Start();
                Debug.Log($"[OPTINAV Detection Bridge] Listening for CV detection results on 127.0.0.1:{listenPort}");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OPTINAV Detection Bridge] Failed to bind to port {listenPort}: {ex.Message}");
                isRunning = false;
                return;
            }

            while (isRunning)
            {
                TcpClient client = null;
                try
                {
                    client = tcpListener.AcceptTcpClient();
                    clientConnected = true;
                    Debug.Log($"[OPTINAV Detection Bridge] Python detector connected from {client.Client.RemoteEndPoint}");

                    using (var stream = client.GetStream())
                    using (var reader = new StreamReader(stream, Encoding.UTF8))
                    {
                        while (isRunning && client.Connected)
                        {
                            string jsonLine = reader.ReadLine();
                            if (jsonLine == null)
                            {
                                // Client disconnected
                                break;
                            }

                            if (string.IsNullOrWhiteSpace(jsonLine)) continue;

                            try
                            {
                                DetectionResult res = JsonUtility.FromJson<DetectionResult>(jsonLine);
                                incomingResults.Enqueue(res);
                            }
                            catch (Exception parseEx)
                            {
                                Debug.LogWarning($"[OPTINAV Detection Bridge] Malformed JSON received: {parseEx.Message}");
                            }
                        }
                    }
                }
                catch (SocketException sex)
                {
                    if (!isRunning) break;
                    Debug.LogWarning($"[OPTINAV Detection Bridge] Socket error: {sex.Message}");
                }
                catch (Exception ex)
                {
                    if (!isRunning) break;
                    Debug.LogWarning($"[OPTINAV Detection Bridge] Reader error: {ex.Message}");
                }
                finally
                {
                    clientConnected = false;
                    client?.Close();
                }

                if (isRunning)
                {
                    Thread.Sleep(200);
                }
            }
        }

        private void Update()
        {
            bool hasNew = false;
            while (incomingResults.TryDequeue(out DetectionResult res))
            {
                latestResult = res;
                totalResultsReceived++;
                fpsCounter++;
                hasNew = true;
            }

            if (hasNew)
            {
                OnDetectionReceived?.Invoke(latestResult);
            }

            float now = Time.unscaledTime;
            if (now - lastFpsTime >= 1.0f)
            {
                detectionFPS = fpsCounter / (now - lastFpsTime);
                fpsCounter = 0;
                lastFpsTime = now;
            }
        }

        private void OnDestroy()
        {
            StopNetworkListener();
            if (Instance == this)
            {
                Instance = null;
            }
        }

        private void OnApplicationQuit()
        {
            StopNetworkListener();
        }
    }
}
