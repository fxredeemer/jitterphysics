using System.Collections.Generic;

namespace Jitter
{
    public class ArrayResourcePool<T>
    {
        private readonly Stack<T[]> stack = new Stack<T[]>();

        private readonly int arrayLength;

        public ArrayResourcePool(int arrayLength)
        {
            this.arrayLength = arrayLength;
        }

        public void ResetResourcePool()
        {
            lock (stack) { stack.Clear(); }
        }

        public int Count => stack.Count;

        public void GiveBack(T[] obj)
        {
            lock (stack) { stack.Push(obj); }
        }

        public T[] GetNew()
        {
            T[] freeObj;

            lock (stack)
            {
                if (stack.Count == 0)
                {
                    freeObj = new T[arrayLength];
                    stack.Push(freeObj);
                }

                freeObj = stack.Pop();
            }

            return freeObj;
        }
    }
}
