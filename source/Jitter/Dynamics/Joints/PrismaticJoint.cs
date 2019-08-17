
using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;

namespace Jitter.Dynamics.Joints
{
    public class PrismaticJoint : Joint
    {
        public PointPointDistance MaximumDistanceConstraint { get; }
        public PointPointDistance MinimumDistanceConstraint { get; }

        public FixedAngle FixedAngleConstraint { get; }
        public PointOnLine PointOnLineConstraint { get; }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2) : base(world)
        {
            FixedAngleConstraint = new FixedAngle(body1, body2);
            PointOnLineConstraint = new PointOnLine(body1, body2, body1.position, body2.position);
        }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2, float minimumDistance, float maximumDistance)
            : base(world)
        {
            FixedAngleConstraint = new FixedAngle(body1, body2);
            PointOnLineConstraint = new PointOnLine(body1, body2, body1.position, body2.position);

            MinimumDistanceConstraint = new PointPointDistance(body1, body2, body1.position, body2.position)
            {
                Behavior = PointPointDistance.DistanceBehavior.LimitMinimumDistance,
                Distance = minimumDistance
            };

            MaximumDistanceConstraint = new PointPointDistance(body1, body2, body1.position, body2.position)
            {
                Behavior = PointPointDistance.DistanceBehavior.LimitMaximumDistance,
                Distance = maximumDistance
            };
        }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector pointOnBody1, JVector pointOnBody2)
            : base(world)
        {
            FixedAngleConstraint = new FixedAngle(body1, body2);
            PointOnLineConstraint = new PointOnLine(body1, body2, pointOnBody1, pointOnBody2);
        }

        public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector pointOnBody1, JVector pointOnBody2, float maximumDistance, float minimumDistance)
            : base(world)
        {
            FixedAngleConstraint = new FixedAngle(body1, body2);
            PointOnLineConstraint = new PointOnLine(body1, body2, pointOnBody1, pointOnBody2);
        }

        public override void Activate()
        {
            if (MaximumDistanceConstraint != null)
            {
                World.AddConstraint(MaximumDistanceConstraint);
            }

            if (MinimumDistanceConstraint != null)
            {
                World.AddConstraint(MinimumDistanceConstraint);
            }

            World.AddConstraint(FixedAngleConstraint);
            World.AddConstraint(PointOnLineConstraint);
        }

        public override void Deactivate()
        {
            if (MaximumDistanceConstraint != null)
            {
                World.RemoveConstraint(MaximumDistanceConstraint);
            }

            if (MinimumDistanceConstraint != null)
            {
                World.RemoveConstraint(MinimumDistanceConstraint);
            }

            World.RemoveConstraint(FixedAngleConstraint);
            World.RemoveConstraint(PointOnLineConstraint);
        }
    }
}
