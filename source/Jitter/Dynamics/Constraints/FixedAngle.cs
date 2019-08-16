using Jitter.LinearMath;
using System;

namespace Jitter.Dynamics.Constraints
{
    public class FixedAngle : Constraint
    {
        private JVector accumulatedImpulse;

        private JMatrix initialOrientation1, initialOrientation2;

        public FixedAngle(RigidBody body1, RigidBody body2) : base(body1, body2)
        {
            initialOrientation1 = body1.orientation;
            initialOrientation2 = body2.orientation;
        }

        public JVector AppliedImpulse => accumulatedImpulse;

        public JMatrix InitialOrientationBody1 { get => initialOrientation1; set => initialOrientation1 = value; }
        public JMatrix InitialOrientationBody2 { get => initialOrientation2; set => initialOrientation2 = value; }

        public float Softness { get; set; }

        public float BiasFactor { get; set; } = 0.05f;

        private JMatrix effectiveMass;
        private JVector bias;
        private float softnessOverDt;

        public override void PrepareForIteration(float timestep)
        {
            effectiveMass = body1.invInertiaWorld + body2.invInertiaWorld;

            softnessOverDt = Softness / timestep;

            effectiveMass.M11 += softnessOverDt;
            effectiveMass.M22 += softnessOverDt;
            effectiveMass.M33 += softnessOverDt;

            JMatrix.Inverse(ref effectiveMass, out effectiveMass);

            JMatrix.Multiply(ref initialOrientation1, ref initialOrientation2, out var orientationDifference);
            JMatrix.Transpose(ref orientationDifference, out orientationDifference);

            var q = orientationDifference * body2.invOrientation * body1.orientation;
            JVector axis;

            float x = q.M32 - q.M23;
            float y = q.M13 - q.M31;
            float z = q.M21 - q.M12;

            float r = JMath.Sqrt((x * x) + (y * y) + (z * z));
            float t = q.M11 + q.M22 + q.M33;

            float angle = (float)Math.Atan2(r, t - 1);
            axis = new JVector(x, y, z) * angle;

            if (r != 0.0f)
            {
                axis = axis * (1.0f / r);
            }

            bias = axis * BiasFactor * (-1.0f / timestep);

            if (!body1.IsStatic)
            {
                body1.angularVelocity += JVector.Transform(accumulatedImpulse, body1.invInertiaWorld);
            }

            if (!body2.IsStatic)
            {
                body2.angularVelocity += JVector.Transform(-1.0f * accumulatedImpulse, body2.invInertiaWorld);
            }
        }

        public override void Iterate()
        {
            var jv = body1.angularVelocity - body2.angularVelocity;

            var softnessVector = accumulatedImpulse * softnessOverDt;

            var lambda = -1.0f * JVector.Transform(jv + bias + softnessVector, effectiveMass);

            accumulatedImpulse += lambda;

            if (!body1.IsStatic)
            {
                body1.angularVelocity += JVector.Transform(lambda, body1.invInertiaWorld);
            }

            if (!body2.IsStatic)
            {
                body2.angularVelocity += JVector.Transform(-1.0f * lambda, body2.invInertiaWorld);
            }
        }
    }
}
