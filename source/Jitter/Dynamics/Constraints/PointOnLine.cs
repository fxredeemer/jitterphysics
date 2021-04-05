using Jitter.LinearMath;

namespace Jitter.Dynamics.Constraints
{
    public class PointOnLine : Constraint
    {
        private JVector lineNormal;
        private JVector localAnchor1;
        private JVector localAnchor2;
        private JVector r1;
        private JVector r2;

        public PointOnLine(RigidBody body1, RigidBody body2, JVector lineStartPointBody1, JVector pointBody2) : base(body1, body2)
        {
            JVector.Subtract(lineStartPointBody1, body1.position, out localAnchor1);
            JVector.Subtract(pointBody2, body2.position, out localAnchor2);

            JVector.Transform(localAnchor1, body1.invOrientation, out localAnchor1);
            JVector.Transform(localAnchor2, body2.invOrientation, out localAnchor2);

            lineNormal = JVector.Normalize(lineStartPointBody1 - pointBody2);
        }

        public float AppliedImpulse { get; private set; }

        public float Softness { get; set; }

        public float BiasFactor { get; set; } = 0.5f;

        private float effectiveMass;
        private float bias;
        private float softnessOverDt;
        private readonly JVector[] jacobian = new JVector[4];

        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(localAnchor1, body1.orientation, out r1);
            JVector.Transform(localAnchor2, body2.orientation, out r2);

            JVector.Add(body1.position, r1, out var p1);
            JVector.Add(body2.position, r2, out var p2);
            JVector.Subtract(p2, p1, out _);

            var l = JVector.Transform(lineNormal, body1.orientation);
            l = JVector.Normalize(l);

            var t = (p1 - p2) % l;
            if (t.LengthSquared() != 0.0f)
            {
                t = JVector.Normalize(t);
            }

            t %= l;

            jacobian[0] = t;
            jacobian[1] = (r1 + p2 - p1) % t;
            jacobian[2] = -1.0f * t;
            jacobian[3] = -1.0f * r2 % t;

            effectiveMass = body1.inverseMass + body2.inverseMass
                + (JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1])
                + (JVector.Transform(jacobian[3], body2.invInertiaWorld) * jacobian[3]);

            softnessOverDt = Softness / timestep;
            effectiveMass += softnessOverDt;

            if (effectiveMass != 0)
            {
                effectiveMass = 1.0f / effectiveMass;
            }

            bias = -(l % (p2 - p1)).Length() * BiasFactor * (1.0f / timestep);

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
            drawer.DrawLine(body1.position + r1,
                body1.position + r1 + (JVector.Transform(lineNormal, body1.orientation) * 100.0f));
        }
    }
}
