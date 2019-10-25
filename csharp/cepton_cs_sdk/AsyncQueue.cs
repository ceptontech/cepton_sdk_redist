using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;

namespace Cepton.SDK
{
    class AsyncQueue<T>
    {
        private readonly ConcurrentQueue<T> queue;
        private readonly ConcurrentQueue<TaskCompletionSource<T>> promise_queue;
        private readonly object locker = new object();

        public AsyncQueue()
        {
            queue = new ConcurrentQueue<T>();
            promise_queue = new ConcurrentQueue<TaskCompletionSource<T>>();
        }

        private bool TryFulfillPromise(T item)
        {
            TaskCompletionSource<T> promise;
            do
            {
                if (promise_queue.TryDequeue(out promise) &&
                    promise.TrySetResult(item))
                {
                    return true;
                }
            }
            while (promise != null);
            return false;
        }

        public void Enqueue(T item)
        {
            if (TryFulfillPromise(item))
                return;

            lock (locker)
            {
                if (TryFulfillPromise(item))
                    return;

                queue.Enqueue(item);
                if (queue.Count > 10000)
                    queue.TryDequeue(out _); // Fifo Discard
            }
        }

        public void Clear()
        {
            lock(locker)
            {
                while (queue.TryDequeue(out _)) ;
            }
        }

        public bool TryDequeue(out T item)
        {
            return queue.TryDequeue(out item);
        }

        public Task<T> DequeueAsync(CancellationToken cancellationToken)
        {
            if (!queue.TryDequeue(out T item))
            {
                lock (locker)
                {
                    if (!queue.TryDequeue(out item))
                    {
                        var promise = new TaskCompletionSource<T>();
                        cancellationToken.Register(() => promise.TrySetCanceled());

                        promise_queue.Enqueue(promise);
                        return promise.Task;
                    }
                }
            }

            return Task.FromResult(item);
        }

        public int Count => queue.Count;
    }
}
