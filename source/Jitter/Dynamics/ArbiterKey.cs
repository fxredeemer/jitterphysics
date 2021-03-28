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
            return (other.body1.Equals(body1) && other.body2.Equals(body2))
                || (other.body1.Equals(body2) && other.body2.Equals(body1));
        }

        public override int GetHashCode()
        {
            return body1.GetHashCode() + body2.GetHashCode();
        }
    }
}
