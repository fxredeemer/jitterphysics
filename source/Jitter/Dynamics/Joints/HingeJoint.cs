using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;

namespace Jitter.Dynamics.Joints
{
    public class HingeJoint : Joint
    {
        private readonly PointOnPoint[] worldPointConstraint;

        public PointOnPoint PointConstraint1 => worldPointConstraint[0];
        public PointOnPoint PointConstraint2 => worldPointConstraint[1];

        public HingeJoint(World world, RigidBody body1, RigidBody body2, JVector position, JVector hingeAxis) : base(world)
        {
            worldPointConstraint = new PointOnPoint[2];

            hingeAxis *= 0.5f;

            var pos1 = position; JVector.Add(ref pos1, ref hingeAxis, out pos1);
            var pos2 = position; JVector.Subtract(ref pos2, ref hingeAxis, out pos2);

            worldPointConstraint[0] = new PointOnPoint(body1, body2, pos1);
            worldPointConstraint[1] = new PointOnPoint(body1, body2, pos2);
        }

        public PointOnPoint PointOnPointConstraint1 => worldPointConstraint[0];

        public PointOnPoint PointOnPointConstraint2 => worldPointConstraint[1];

        public float AppliedImpulse => worldPointConstraint[0].AppliedImpulse + worldPointConstraint[1].AppliedImpulse;

        public override void Activate()
        {
            World.AddConstraint(worldPointConstraint[0]);
            World.AddConstraint(worldPointConstraint[1]);
        }

        public override void Deactivate()
        {
            World.RemoveConstraint(worldPointConstraint[0]);
            World.RemoveConstraint(worldPointConstraint[1]);
        }
    }
}
