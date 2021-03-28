using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints
{
    public class PointPointDistance : Constraint
    {
        public enum DistanceBehavior
        {
            LimitDistance,
            LimitMaximumDistance,
            LimitMinimumDistance,
        }

        private JVector localAnchor1;
        private JVector localAnchor2;
        private JVector r1;
        private JVector r2;

        public PointPointDistance(RigidBody body1, RigidBody body2, JVector anchor1, JVector anchor2) : base(body1, body2)
        {
            JVector.Subtract(ref anchor1, ref body1.position, out localAnchor1);
            JVector.Subtract(ref anchor2, ref body2.position, out localAnchor2);

            JVector.Transform(ref localAnchor1, ref body1.invOrientation, out localAnchor1);
            JVector.Transform(ref localAnchor2, ref body2.invOrientation, out localAnchor2);

            Distance = (anchor1 - anchor2).Length();
        }

        public float AppliedImpulse { get; private set; }

        public float Distance { get; set; }

        public DistanceBehavior Behavior { get; set; }

        public JVector LocalAnchor1
        {
            get => localAnchor1;
            set => localAnchor1 = value;
        }

        public JVector LocalAnchor2
        {
            get => localAnchor2;
            set => localAnchor2 = value;
        }

        public float Softness { get; set; } = 0.01f;

        public float BiasFactor { get; set; } = 0.1f;

        private float effectiveMass;
        private float bias;
        private float softnessOverDt;
        private readonly JVector[] jacobian = new JVector[4];

        private bool skipConstraint;

        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            JVector.Transform(ref localAnchor2, ref body2.orientation, out r2);

            JVector.Add(ref body1.position, ref r1, out var p1);
            JVector.Add(ref body2.position, ref r2, out var p2);

            JVector.Subtract(ref p2, ref p1, out var dp);

            float deltaLength = dp.Length() - Distance;

            if (Behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= 0.0f)
            {
                skipConstraint = true;
            }
            else if (Behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= 0.0f)
            {
                skipConstraint = true;
            }
            else
            {
                skipConstraint = false;

                var n = p2 - p1;
                if (n.LengthSquared() != 0.0f)
                {
                    n.Normalize();
                }

                jacobian[0] = -1.0f * n;
                jacobian[1] = -1.0f * (r1 % n);
                jacobian[2] = 1.0f * n;
                jacobian[3] = r2 % n;

                effectiveMass = body1.inverseMass + body2.inverseMass
                    + (JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1])
                    + (JVector.Transform(jacobian[3], body2.invInertiaWorld) * jacobian[3]);

                softnessOverDt = Softness / timestep;
                effectiveMass += softnessOverDt;

                effectiveMass = 1.0f / effectiveMass;

                bias = deltaLength * BiasFactor * (1.0f / timestep);

                if (!body1.isStatic)
                {
                    body1.linearVelocity += body1.inverseMass * AppliedImpulse * jacobian[0];
                    body1.angularVelocity += JVector.Transform(AppliedImpulse * jacobian[1], body1.invInertiaWorld);
                }

                if (!body2.isStatic)
                {
                    body2.linearVelocity += body2.inverseMass * AppliedImpulse * jacobian[2];
                    body2.angularVelocity += JVector.Transform(AppliedImpulse * jacobian[3], body2.invInertiaWorld);
                }
            }
        }

        public override void Iterate()
        {
            if (skipConstraint)
            {
                return;
            }

            float jv =
                (body1.linearVelocity * jacobian[0])
                + (body1.angularVelocity * jacobian[1])
                + (body2.linearVelocity * jacobian[2])
                + (body2.angularVelocity * jacobian[3]);

            float softnessScalar = AppliedImpulse * softnessOverDt;

            float lambda = -effectiveMass * (jv + bias + softnessScalar);

            if (Behavior == DistanceBehavior.LimitMinimumDistance)
            {
                float previousAccumulatedImpulse = AppliedImpulse;
                AppliedImpulse = JMath.Max(AppliedImpulse + lambda, 0);
                lambda = AppliedImpulse - previousAccumulatedImpulse;
            }
            else if (Behavior == DistanceBehavior.LimitMaximumDistance)
            {
                float previousAccumulatedImpulse = AppliedImpulse;
                AppliedImpulse = JMath.Min(AppliedImpulse + lambda, 0);
                lambda = AppliedImpulse - previousAccumulatedImpulse;
            }
            else
            {
                AppliedImpulse += lambda;
            }

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * lambda * jacobian[2];
                body2.angularVelocity += JVector.Transform(lambda * jacobian[3], body2.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(body1.position + r1, body2.position + r2);
        }
    }
}
