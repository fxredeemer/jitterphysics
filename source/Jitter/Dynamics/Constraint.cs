using System;
using System.Threading;

namespace Jitter.Dynamics.Constraints
{
    public abstract class Constraint : IConstraint, IDebugDrawable, IComparable<Constraint>
    {
        internal RigidBody body1;
        internal RigidBody body2;

        public RigidBody Body1 => body1;

        public RigidBody Body2 => body2;

        private static int instanceCount;
        private readonly int instance;

        public Constraint(RigidBody body1, RigidBody body2)
        {
            this.body1 = body1;
            this.body2 = body2;

            instance = Interlocked.Increment(ref instanceCount);

            body1?.Update();
            body2?.Update();
        }

        public abstract void PrepareForIteration(float timestep);

        public abstract void Iterate();

        public int CompareTo(Constraint other)
        {
            if (other.instance < instance)
            {
                return -1;
            }
            else if (other.instance > instance)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }

        public virtual void DebugDraw(IDebugDrawer drawer)
        {
        }
    }
}
