using System.Collections;
using System.Collections.Generic;

namespace Jitter.Dynamics
{
    public class ArbiterMap : IEnumerable
    {
        private readonly Dictionary<ArbiterKey, Arbiter> dictionary =
            new Dictionary<ArbiterKey, Arbiter>(2048, arbiterKeyComparer);

        private ArbiterKey lookUpKey;
        private static readonly ArbiterKeyComparer arbiterKeyComparer = new ArbiterKeyComparer();

        public ArbiterMap()
        {
            lookUpKey = new ArbiterKey(null, null);
        }

        public bool LookUpArbiter(RigidBody body1, RigidBody body2, out Arbiter arbiter)
        {
            lookUpKey.SetBodies(body1, body2);
            return dictionary.TryGetValue(lookUpKey, out arbiter);
        }

        public Dictionary<ArbiterKey, Arbiter>.ValueCollection Arbiters => dictionary.Values;

        internal void Add(ArbiterKey key, Arbiter arbiter)
        {
            dictionary.Add(key, arbiter);
        }

        internal void Clear()
        {
            dictionary.Clear();
        }

        internal void Remove(Arbiter arbiter)
        {
            lookUpKey.SetBodies(arbiter.body1, arbiter.body2);
            dictionary.Remove(lookUpKey);
        }

        public bool ContainsArbiter(RigidBody body1, RigidBody body2)
        {
            lookUpKey.SetBodies(body1, body2);
            return dictionary.ContainsKey(lookUpKey);
        }

        public IEnumerator GetEnumerator()
        {
            return dictionary.Values.GetEnumerator();
        }
    }
}
