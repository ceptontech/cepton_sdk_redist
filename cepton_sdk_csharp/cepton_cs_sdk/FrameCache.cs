using System.Threading;
using System.Threading.Tasks;

namespace Cepton.SDK
{
    public class FrameCache<PointType>
    {
        private readonly AsyncQueue<PointType[]> cache = new AsyncQueue<PointType[]>();
        private readonly int max_frames;
        private readonly object locker = new object();

        public ulong LastTimestamp { get; set; }

        public FrameCache(int max_frames)
        {
            this.max_frames = max_frames;
        }

        public void Clear() => cache.Clear();

        public void Add(PointType[] points)
        {
            if (points.Length == 0)
                return;
            cache.Enqueue(points);
            if (cache.Count > max_frames)
            {
                lock (locker)
                {
                    if (cache.Count > max_frames)
                        cache.TryDequeue(out var discarded);
                }
            }
        }

        public PointType[] GetAnyPoints()
        {
            cache.TryDequeue(out var points);
            return points;
        }

        public async Task<PointType[]> GetPointsAsync(CancellationToken token)
        {
            return await cache.DequeueAsync(token);
        }
    }
}