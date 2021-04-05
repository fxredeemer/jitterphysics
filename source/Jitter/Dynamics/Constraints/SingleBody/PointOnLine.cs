using Jitter.LinearMath;
using System;

namespace Jitter.Dynamics.Constraints.SingleBody
{
    public class PointOnLine : Constraint
    {
        private JVector localAnchor1;
        private JVector r1;

        private JVector lineNormal = JVector.Right;
        private JVector anchor;

        public PointOnLine(RigidBody body, JVector localAnchor, JVector lineDirection) : base(body, null)
        {
            if (lineDirection.LengthSquared() == 0.0f)
            {
                throw new ArgumentException("Line direction can't be zero", nameof(lineDirection));
            }

            localAnchor1 = localAnchor;
            anchor = body.position + JVector.Transform(localAnchor, body.orientation);

            lineNormal = lineDirection;
            lineNormal = JVector.Normalize(lineNormal);
        }

        public JVector Anchor { get => anchor; set => anchor = value; }

        public JVector Axis
        {
            get => lineNormal;
            set
            {
                lineNormal = value;
                lineNormal = JVector.Normalize(lineNormal);
            }
        }

        public float Softness { get; set; }

        public float BiasFactor { get; set; } = 0.5f;

        private float effectiveMass;
        private float accumulatedImpulse;
        private float bias;
        private float softnessOverDt;
        private readonly JVector[] jacobian = new JVector[2];

        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);

            JVector.Add(ref body1.position, ref r1, out var p1);

            JVector.Subtract(ref p1, ref anchor, out var dp);

            var l = lineNormal;

            var t = (p1 - anchor) % l;
            if (t.LengthSquared() != 0.0f)
            {
                t = JVector.Normalize(t);
            }

            t %= l;

            jacobian[0] = t;
            jacobian[1] = r1 % t;

            effectiveMass = body1.inverseMass
                + (JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1]);

            softnessOverDt = Softness / timestep;
            effectiveMass += softnessOverDt;

            if (effectiveMass != 0)
            {
                effectiveMass = 1.0f / effectiveMass;
            }

            bias = -(l % (p1 - anchor)).Length() * BiasFactor * (1.0f / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += JVector.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void Iterate()
        {
            float jv =
                (body1.linearVelocity * jacobian[0])
                + (body1.angularVelocity * jacobian[1]);

            float softnessScalar = accumulatedImpulse * softnessOverDt;

            float lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(anchor - (lineNormal * 50.0f), anchor + (lineNormal * 50.0f));
            drawer.DrawLine(body1.position, body1.position + r1);
        }
    }
}
