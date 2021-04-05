using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints.SingleBody
{
    public class PointOnPoint : Constraint
    {
        private JVector localAnchor1;
        private JVector anchor;

        private JVector r1;

        public PointOnPoint(RigidBody body, JVector localAnchor) : base(body, null)
        {
            localAnchor1 = localAnchor;

            anchor = body.position + JVector.Transform(localAnchor, body.orientation);
        }

        public float AppliedImpulse { get; private set; }

        public float Softness { get; set; } = 0.01f;

        public JVector Anchor { get => anchor; set => anchor = value; }

        public float BiasFactor { get; set; } = 0.1f;

        private float effectiveMass;
        private float bias;
        private float softnessOverDt;
        private readonly JVector[] jacobian = new JVector[2];

        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            JVector.Add(ref body1.position, ref r1, out var p1);

            JVector.Subtract(ref p1, ref anchor, out var dp);
            float deltaLength = dp.Length();

            var n = anchor - p1;
            if (n.LengthSquared() != 0.0f)
            {
                n = JVector.Normalize(n);
            }

            jacobian[0] = -1.0f * n;
            jacobian[1] = -1.0f * (r1 % n);

            effectiveMass = body1.inverseMass + (JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1]);

            softnessOverDt = Softness / timestep;
            effectiveMass += softnessOverDt;

            effectiveMass = 1.0f / effectiveMass;

            bias = deltaLength * BiasFactor * (1.0f / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * AppliedImpulse * jacobian[0];
                body1.angularVelocity += JVector.Transform(AppliedImpulse * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void Iterate()
        {
            float jv =
                (body1.linearVelocity * jacobian[0])
                + (body1.angularVelocity * jacobian[1]);

            float softnessScalar = AppliedImpulse * softnessOverDt;

            float lambda = -effectiveMass * (jv + bias + softnessScalar);

            AppliedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawPoint(anchor);
        }
    }
}
