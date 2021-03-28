using System;
using System.Collections.Generic;

namespace Jitter
{
    public class ResourcePool<T>
    {
        private readonly Stack<T> stack = new Stack<T>();

        public void ResetResourcePool()
        {
            lock (stack)
            {
                stack.Clear();
            }
        }

        public int Count => stack.Count;

        public void GiveBack(T obj)
        {
            lock (stack)
            {
                stack.Push(obj);
            }
        }

        public T GetNew()
        {
            T freeObj;

            lock (stack)
            {
                if (stack.Count == 0)
                {
                    freeObj = Activator.CreateInstance<T>();
                    stack.Push(freeObj);
                }

                freeObj = stack.Pop();
            }

            return freeObj;
        }
    }
}
