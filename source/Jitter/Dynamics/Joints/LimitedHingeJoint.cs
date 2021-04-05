using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;

namespace Jitter.Dynamics.Joints
{
    public class LimitedHingeJoint : Joint
    {
        private readonly PointOnPoint[] worldPointConstraint;

        public PointOnPoint PointConstraint1 => worldPointConstraint[0];
        public PointOnPoint PointConstraint2 => worldPointConstraint[1];

        public PointPointDistance DistanceConstraint { get; }

        public LimitedHingeJoint(World world, RigidBody body1, RigidBody body2, JVector position, JVector hingeAxis, float hingeFwdAngle, float hingeBckAngle) : base(world)
        {
            worldPointConstraint = new PointOnPoint[2];

            hingeAxis *= 0.5f;

            var pos1 = position;
            JVector.Add(pos1, hingeAxis, out pos1);
            var pos2 = position;
            JVector.Subtract(pos2, hingeAxis, out pos2);

            worldPointConstraint[0] = new PointOnPoint(body1, body2, pos1);
            worldPointConstraint[1] = new PointOnPoint(body1, body2, pos2);

            hingeAxis = JVector.Normalize(hingeAxis);

            var perpDir = JVector.Up;

            if (JVector.Dot(perpDir, hingeAxis) > 0.1f)
            {
                perpDir = JVector.Right;
            }

            var sideAxis = JVector.Cross(hingeAxis, perpDir);
            perpDir = JVector.Cross(sideAxis, hingeAxis);
            perpDir = JVector.Normalize(perpDir);

            float len = 10.0f * 3;

            var hingeRelAnchorPos0 = perpDir * len;

            float angleToMiddle = 0.5f * (hingeFwdAngle - hingeBckAngle);
            var hingeRelAnchorPos1 = JVector.Transform(hingeRelAnchorPos0, JMatrix.CreateFromAxisAngle(hingeAxis, -angleToMiddle / 360.0f * 2.0f * JMath.Pi));

            float hingeHalfAngle = 0.5f * (hingeFwdAngle + hingeBckAngle);
            float allowedDistance = len * 2.0f * (float)System.Math.Sin(hingeHalfAngle * 0.5f / 360.0f * 2.0f * JMath.Pi);

            var hingePos = body1.Position;
            var relPos0c = hingePos + hingeRelAnchorPos0;
            var relPos1c = hingePos + hingeRelAnchorPos1;

            DistanceConstraint = new PointPointDistance(body1, body2, relPos0c, relPos1c)
            {
                Distance = allowedDistance,
                Behavior = PointPointDistance.DistanceBehavior.LimitMaximumDistance
            };
        }

        public override void Activate()
        {
            World.AddConstraint(worldPointConstraint[0]);
            World.AddConstraint(worldPointConstraint[1]);
            World.AddConstraint(DistanceConstraint);
        }

        public override void Deactivate()
        {
            World.RemoveConstraint(worldPointConstraint[0]);
            World.RemoveConstraint(worldPointConstraint[1]);
            World.RemoveConstraint(DistanceConstraint);
        }
    }
}
