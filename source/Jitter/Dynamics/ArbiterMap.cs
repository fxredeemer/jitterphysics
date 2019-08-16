using System.Collections;
using System.Collections.Generic;

namespace Jitter.Dynamics
{
    public struct ArbiterKey
    {
        internal RigidBody body1, body2;

        public ArbiterKey(RigidBody body1, RigidBody body2)
        {
            this.body1 = body1;
            this.body2 = body2;
        }

        internal void SetBodies(RigidBody body1, RigidBody body2)
        {
            this.body1 = body1;
            this.body2 = body2;
        }

        public override bool Equals(object obj)
        {
            var other = (ArbiterKey)obj;
            return ((other.body1.Equals(body1) && other.body2.Equals(body2)) ||
                (other.body1.Equals(body2) && other.body2.Equals(body1)));
        }

        public override int GetHashCode()
        {
            return body1.GetHashCode() + body2.GetHashCode();
        }
    }

    internal class ArbiterKeyComparer : IEqualityComparer<ArbiterKey>
    {
        public bool Equals(ArbiterKey x, ArbiterKey y)
        {
            return ((x.body1.Equals(y.body1) && x.body2.Equals(y.body2)) ||
                (x.body1.Equals(y.body2) && x.body2.Equals(y.body1)));
        }

        public int GetHashCode(ArbiterKey obj)
        {
            return obj.body1.GetHashCode() + obj.body2.GetHashCode();
        }
    }

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
