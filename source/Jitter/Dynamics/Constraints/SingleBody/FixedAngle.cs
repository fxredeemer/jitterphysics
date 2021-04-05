using Jitter.LinearMath;
using System;

namespace Jitter.Dynamics.Constraints.SingleBody
{
    public class FixedAngle : Constraint
    {
        private JMatrix orientation;
        private JVector accumulatedImpulse;

        public FixedAngle(RigidBody body1) : base(body1, null)
        {
            orientation = body1.orientation;
        }

        public float Softness { get; set; }

        public float BiasFactor { get; set; } = 0.05f;

        public JMatrix InitialOrientation { get => orientation; set => orientation = value; }

        private JMatrix effectiveMass;
        private JVector bias;
        private float softnessOverDt;

        public override void PrepareForIteration(float timestep)
        {
            effectiveMass = body1.invInertiaWorld;

            softnessOverDt = Softness / timestep;

            effectiveMass = JMatrix.FromDiagonal(
                m11: effectiveMass.M11 + softnessOverDt,
                m22: effectiveMass.M22 + softnessOverDt,
                m33: effectiveMass.M33 + softnessOverDt);

            JMatrix.Inverse(effectiveMass, out effectiveMass);

            var q = JMatrix.Transpose(orientation) * body1.orientation;

            float x = q.M32 - q.M23;
            float y = q.M13 - q.M31;
            float z = q.M21 - q.M12;

            float r = JMath.Sqrt((x * x) + (y * y) + (z * z));
            float t = q.M11 + q.M22 + q.M33;

            float angle = (float)Math.Atan2(r, t - 1);
            var axis = new JVector(x, y, z) * angle;

            if (r != 0.0f)
            {
                axis *= 1.0f / r;
            }

            bias = axis * BiasFactor * (-1.0f / timestep);

            if (!body1.IsStatic)
            {
                body1.angularVelocity += JVector.Transform(accumulatedImpulse, body1.invInertiaWorld);
            }
        }

        public override void Iterate()
        {
            var jv = body1.angularVelocity;

            var softnessVector = accumulatedImpulse * softnessOverDt;

            var lambda = -1.0f * JVector.Transform(jv + bias + softnessVector, effectiveMass);

            accumulatedImpulse += lambda;

            if (!body1.IsStatic)
            {
                body1.angularVelocity += JVector.Transform(lambda, body1.invInertiaWorld);
            }
        }
    }
}
