using System.Collections;
using System.Collections.Generic;

namespace Jitter.DataStructures
{
    public class ReadOnlyHashset<T> : IEnumerable, IEnumerable<T>
    {
        private readonly HashSet<T> hashset;

        public ReadOnlyHashset(HashSet<T> hashset) { this.hashset = hashset; }

        public IEnumerator GetEnumerator()
        {
            return hashset.GetEnumerator();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return hashset.GetEnumerator();
        }

        public int Count => hashset.Count;

        public bool Contains(T item) { return hashset.Contains(item); }
    }
}
