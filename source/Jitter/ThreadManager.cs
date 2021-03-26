using System;
using System.Collections.Generic;
using System.Threading;

#if PORTABLE
using System.Threading.Tasks;
using Thread = System.Threading.Tasks.Task;
#endif

namespace Jitter
{
    public sealed class ThreadManager
    {
        public const int ThreadsPerProcessor = 1;

        private ManualResetEvent waitHandleA, waitHandleB;
        private ManualResetEvent currentWaitHandle;
        private readonly List<Action<object>> tasks = new List<Action<object>>();
        private readonly List<object> parameters = new List<object>();

        private Thread[] threads;
        private int currentTaskIndex, waitingThreadCount;

        internal int threadCount;

        public int ThreadCount { private set => threadCount = value; get => threadCount; }

        private static ThreadManager instance;

        public static ThreadManager Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new ThreadManager();
                    instance.Initialize();
                }

                return instance;
            }
        }

        private ThreadManager() { }

        private void Initialize()
        {
            threadCount = Environment.ProcessorCount * ThreadsPerProcessor;

            threads = new Thread[threadCount];
            waitHandleA = new ManualResetEvent(false);
            waitHandleB = new ManualResetEvent(false);

            currentWaitHandle = waitHandleA;

            var initWaitHandle = new AutoResetEvent(false);

            for (int i = 1; i < threads.Length; i++)
            {
                threads[i] = NewThread(() =>
                {
                    initWaitHandle.Set();
                    ThreadProc();
                });

                threads[i].Start();
                initWaitHandle.WaitOne();
            }
        }

        public void Execute()
        {
            currentTaskIndex = 0;
            waitingThreadCount = 0;

            currentWaitHandle.Set();
            PumpTasks();

            while (waitingThreadCount < threads.Length - 1)
            {
                ThreadSleep(0);
            }

            currentWaitHandle.Reset();
            currentWaitHandle = (currentWaitHandle == waitHandleA) ? waitHandleB : waitHandleA;

            tasks.Clear();
            parameters.Clear();
        }

        public void AddTask(Action<object> task, object param)
        {
            tasks.Add(task);
            parameters.Add(param);
        }

        private void ThreadProc()
        {
            while (true)
            {
                Interlocked.Increment(ref waitingThreadCount);
                waitHandleA.WaitOne();
                PumpTasks();

                Interlocked.Increment(ref waitingThreadCount);
                waitHandleB.WaitOne();
                PumpTasks();
            }
        }

        private void PumpTasks()
        {
            int count = tasks.Count;

            while (currentTaskIndex < count)
            {
                int taskIndex = currentTaskIndex;

                if (taskIndex == Interlocked.CompareExchange(ref currentTaskIndex, taskIndex + 1, taskIndex)
                    && taskIndex < count)
                {
                    tasks[taskIndex](parameters[taskIndex]);
                }
            }
        }

        private static void ThreadSleep(int dueTime)
        {
#if PORTABLE
            Task.Delay(dueTime).Wait();
#else
            Thread.Sleep(dueTime);
#endif
        }

#if PORTABLE
        private delegate void ThreadStart();
#endif

        private static Thread NewThread(ThreadStart action)
        {
#if PORTABLE
            return new Thread(action.Invoke, TaskCreationOptions.LongRunning);
#else
            return new Thread(action) { IsBackground = true };
#endif
        }
    }
}
