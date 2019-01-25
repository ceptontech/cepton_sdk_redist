using System;
using System.Threading;
using System.Threading.Tasks;

namespace Cepton.SDK
{
    public class Sensor
    {
        private SensorInformation info;

        protected internal Sensor(ref SensorInformation information) => info = information;

        // NOTE: Shortcuts only for constant values inside information structure
        // Use GetInformation() to get volatile results
        public ulong SerialNumber => info.serial_number;
        public SensorModel Model => info.model;
        public string ModelName => info.model_name;
        public ushort FirmwareVersion => info.FirmwareVersion;
        public ushort SegmentCount => info.segment_count;
        public ushort ReturnCount => info.return_count;
        public ushort PointStride => (ushort)(info.segment_count * info.return_count);
        public IntPtr Handle => info.handle;

        public string GetFirmwareVersionString() => string.Format("V{0:X}", FirmwareVersion);

        public async Task<SensorImagePoint[]> GetImagePointsAsync(CancellationToken token = default(CancellationToken))
        {
            return await frameCache.GetPointsAsync(token);
        }
        public void ClearImagePointBuffer() => frameCache.Clear();
        public SensorImagePoint[] GetImagePoints(int timeout_ms)
        {
            var task = GetImagePointsAsync();
            if (task.Wait(timeout_ms))
                return task.Result;
            return null;
        }

        // Get sensor information directly from SDK and update the sensor object
        public SensorInformation GetInformation()
        {
            info = CeptonSDK.GetSensorInformation(Handle);
            return info;
        }

        private const int MAX_IMAGE_POINT_FRAMES = 1000;
        private readonly FrameCache<SensorImagePoint> frameCache = new FrameCache<SensorImagePoint>(MAX_IMAGE_POINT_FRAMES);

        public long LastImagePointTimestamp { get; private set; }

        public delegate void ImagePointCallback(Sensor sensor, SensorImagePoint[] points);
        public event ImagePointCallback OnImagePoints;

        internal void AddImagePoints(SensorImagePoint[] points)
        {
            var last_point = points[points.Length - 1];
            LastImagePointTimestamp = last_point.timestamp;
            OnImagePoints?.Invoke(this, points);
            frameCache.Add(points);
        }

        public bool IsActive
        {
            get
            {
                var epoch_time = DateTime.UtcNow - new DateTime(1970, 1, 1);
                return LastImagePointTimestamp > epoch_time.TotalMilliseconds * 1000 - 1000000.0;
            }
        }
    }
}
