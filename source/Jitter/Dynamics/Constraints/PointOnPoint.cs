using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints
{
    public class PointOnPoint : Constraint
    {
        private JVector localAnchor1, localAnchor2;
        private JVector r1, r2;

        public PointOnPoint(RigidBody body1, RigidBody body2, JVector anchor) : base(body1, body2)
        {
            JVector.Subtract(ref anchor, ref body1.position, out localAnchor1);
            JVector.Subtract(ref anchor, ref body2.position, out localAnchor2);

            JVector.Transform(ref localAnchor1, ref body1.invOrientation, out localAnchor1);
            JVector.Transform(ref localAnchor2, ref body2.invOrientation, out localAnchor2);
        }

        public float AppliedImpulse { get; private set; } = 0.0f;

        public float Softness { get; set; } = 0.01f;

        public float BiasFactor { get; set; } = 0.05f;

        private float effectiveMass = 0.0f;
        private float bias;
        private float softnessOverDt;
        private readonly JVector[] jacobian = new JVector[4];

        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            JVector.Transform(ref localAnchor2, ref body2.orientation, out r2);

            JVector.Add(ref body1.position, ref r1, out var p1);
            JVector.Add(ref body2.position, ref r2, out var p2);

            JVector.Subtract(ref p2, ref p1, out var dp);

            float deltaLength = dp.Length();

            var n = p2 - p1;
            if (n.LengthSquared() != 0.0f) n.Normalize();

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

        public override void Iterate()
        {
            float jv =
                (body1.linearVelocity * jacobian[0])
                + (body1.angularVelocity * jacobian[1])
                + (body2.linearVelocity * jacobian[2])
                + (body2.angularVelocity * jacobian[3]);

            float softnessScalar = AppliedImpulse * softnessOverDt;

            float lambda = -effectiveMass * (jv + bias + softnessScalar);

            AppliedImpulse += lambda;

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
            drawer.DrawLine(body1.position, body1.position + r1);
            drawer.DrawLine(body2.position, body2.position + r2);
        }
    }
}
