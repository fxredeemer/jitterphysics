﻿using Jitter.LinearMath;
using System;

namespace Jitter.Dynamics
{
    public class Contact : IConstraint
    {
        private ContactSettings settings;

        internal RigidBody body1, body2;

        internal JVector normal, tangent;

        internal JVector realRelPos1, realRelPos2;
        internal JVector relativePos1, relativePos2;
        internal JVector p1, p2;

        internal float accumulatedNormalImpulse;
        internal float accumulatedTangentImpulse;

        internal float penetration;
        internal float initialPen;
        private float friction;

        private float massNormal, massTangent;
        private float restitutionBias;

        private bool newContact;

        private bool treatBody1AsStatic;
        private bool treatBody2AsStatic;

        private bool body1IsMassPoint;
        private bool body2IsMassPoint;

        private float lostSpeculativeBounce;
        private float speculativeVelocity;

        public static readonly ResourcePool<Contact> Pool = new ResourcePool<Contact>();

        private float lastTimeStep = float.PositiveInfinity;

        public float Restitution { get; set; }

        public float StaticFriction { get; set; }

        public float DynamicFriction { get; set; }

        public RigidBody Body1 => body1;

        public RigidBody Body2 => body2;

        public float Penetration => penetration;

        public JVector Position1 => p1;

        public JVector Position2 => p2;

        public JVector Tangent => tangent;

        public JVector Normal => normal;

        public JVector CalculateRelativeVelocity()
        {
            float x, y, z;

            x = (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y) + body2.linearVelocity.X;
            y = (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z) + body2.linearVelocity.Y;
            z = (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X) + body2.linearVelocity.Z;

            return new JVector(
                x - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y) - body1.linearVelocity.X,
                y - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z) - body1.linearVelocity.Y,
                z - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X) - body1.linearVelocity.Z);
        }

        public void Iterate()
        {
            if (treatBody1AsStatic && treatBody2AsStatic)
            {
                return;
            }

            float dvx, dvy, dvz;

            dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;
            dvz = body2.linearVelocity.Z - body1.linearVelocity.Z;

            if (!body1IsMassPoint)
            {
                dvx = dvx - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y);
                dvy = dvy - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z);
                dvz = dvz - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X);
            }

            if (!body2IsMassPoint)
            {
                dvx = dvx + (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y);
                dvy = dvy + (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z);
                dvz = dvz + (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X);
            }

            if ((dvx * dvx) + (dvy * dvy) + (dvz * dvz) < settings.minVelocity * settings.minVelocity)
            { return; }

            var vn = (normal.X * dvx) + (normal.Y * dvy) + (normal.Z * dvz);
            var normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            var oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < 0.0f)
            {
                accumulatedNormalImpulse = 0.0f;
            }

            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            var vt = (dvx * tangent.X) + (dvy * tangent.Y) + (dvz * tangent.Z);
            var maxTangentImpulse = friction * accumulatedNormalImpulse;
            var tangentImpulse = massTangent * (-vt);

            var oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse)
            {
                accumulatedTangentImpulse = -maxTangentImpulse;
            }
            else if (accumulatedTangentImpulse > maxTangentImpulse)
            {
                accumulatedTangentImpulse = maxTangentImpulse;
            }

            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            var impulse = new JVector(
                (normal.X * normalImpulse) + (tangent.X * tangentImpulse),
                (normal.Y * normalImpulse) + (tangent.Y * tangentImpulse),
                (normal.Z * normalImpulse) + (tangent.Z * tangentImpulse));

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity = new JVector(
                    body1.linearVelocity.X - (impulse.X * body1.inverseMass),
                    body1.linearVelocity.Y - (impulse.Y * body1.inverseMass),
                    body1.linearVelocity.Z - (impulse.Z * body1.inverseMass));

                if (!body1IsMassPoint)
                {
                    float num0, num1, num2;
                    num0 = (relativePos1.Y * impulse.Z) - (relativePos1.Z * impulse.Y);
                    num1 = (relativePos1.Z * impulse.X) - (relativePos1.X * impulse.Z);
                    num2 = (relativePos1.X * impulse.Y) - (relativePos1.Y * impulse.X);

                    var num3 =
                        (num0 * body1.invInertiaWorld.M11)
                        + (num1 * body1.invInertiaWorld.M21)
                        + (num2 * body1.invInertiaWorld.M31);
                    var num4 =
                        (num0 * body1.invInertiaWorld.M12)
                        + (num1 * body1.invInertiaWorld.M22)
                        + (num2 * body1.invInertiaWorld.M32);
                    var num5 =
                        (num0 * body1.invInertiaWorld.M13)
                        + (num1 * body1.invInertiaWorld.M23)
                        + (num2 * body1.invInertiaWorld.M33);

                    body1.angularVelocity = new JVector(
                        body1.angularVelocity.X - num3,
                        body1.angularVelocity.Y - num4,
                        body1.angularVelocity.Z - num5);
                }
            }

            if (!treatBody2AsStatic)
            {
                body2.linearVelocity = new JVector(
                    body2.linearVelocity.X + (impulse.X * body2.inverseMass),
                    body2.linearVelocity.Y + (impulse.Y * body2.inverseMass),
                    body2.linearVelocity.Z + (impulse.Z * body2.inverseMass));

                if (!body2IsMassPoint)
                {
                    var num0 = (relativePos2.Y * impulse.Z) - (relativePos2.Z * impulse.Y);
                    var num1 = (relativePos2.Z * impulse.X) - (relativePos2.X * impulse.Z);
                    var num2 = (relativePos2.X * impulse.Y) - (relativePos2.Y * impulse.X);

                    var num3 =
                        (num0 * body2.invInertiaWorld.M11)
                        + (num1 * body2.invInertiaWorld.M21)
                        + (num2 * body2.invInertiaWorld.M31);
                    var num4 =
                        (num0 * body2.invInertiaWorld.M12)
                        + (num1 * body2.invInertiaWorld.M22)
                        + (num2 * body2.invInertiaWorld.M32);
                    var num5 =
                        (num0 * body2.invInertiaWorld.M13)
                        + (num1 * body2.invInertiaWorld.M23)
                        + (num2 * body2.invInertiaWorld.M33);

                    body2.angularVelocity = new JVector(
                        body2.angularVelocity.X + num3,
                        body2.angularVelocity.Y + num4,
                        body2.angularVelocity.Z + num5);
                }
            }
        }

        public float AppliedNormalImpulse => accumulatedNormalImpulse;
        public float AppliedTangentImpulse => accumulatedTangentImpulse;

        public void UpdatePosition()
        {
            if (body1IsMassPoint)
            {
                JVector.Add(ref realRelPos1, ref body1.position, out p1);
            }
            else
            {
                JVector.Transform(ref realRelPos1, ref body1.orientation, out p1);
                JVector.Add(ref p1, ref body1.position, out p1);
            }

            if (body2IsMassPoint)
            {
                JVector.Add(ref realRelPos2, ref body2.position, out p2);
            }
            else
            {
                JVector.Transform(ref realRelPos2, ref body2.orientation, out p2);
                JVector.Add(ref p2, ref body2.position, out p2);
            }

            JVector.Subtract(ref p1, ref p2, out var dist);
            penetration = JVector.Dot(ref dist, ref normal);
        }

        public void ApplyImpulse(ref JVector impulse)
        {
            if (!treatBody1AsStatic)
            {
                body1.linearVelocity = new JVector(
                    body1.linearVelocity.X - (impulse.X * body1.inverseMass),
                    body1.linearVelocity.Y - (impulse.Y * body1.inverseMass),
                    body1.linearVelocity.Z - (impulse.Z * body1.inverseMass));

                float num0, num1, num2;
                num0 = (relativePos1.Y * impulse.Z) - (relativePos1.Z * impulse.Y);
                num1 = (relativePos1.Z * impulse.X) - (relativePos1.X * impulse.Z);
                num2 = (relativePos1.X * impulse.Y) - (relativePos1.Y * impulse.X);

                var num3 =
                    (num0 * body1.invInertiaWorld.M11)
                    + (num1 * body1.invInertiaWorld.M21)
                    + (num2 * body1.invInertiaWorld.M31);
                var num4 =
                    (num0 * body1.invInertiaWorld.M12)
                    + (num1 * body1.invInertiaWorld.M22)
                    + (num2 * body1.invInertiaWorld.M32);
                var num5 =
                    (num0 * body1.invInertiaWorld.M13)
                    + (num1 * body1.invInertiaWorld.M23)
                    + (num2 * body1.invInertiaWorld.M33);

                body1.angularVelocity = new JVector(
                    body1.angularVelocity.X - num3,
                    body1.angularVelocity.Y - num4,
                    body1.angularVelocity.Z - num5);
            }

            if (!treatBody2AsStatic)
            {
                body2.LinearVelocity = new JVector(
                    body2.linearVelocity.X + (impulse.X * body2.inverseMass),
                    body2.linearVelocity.Y + (impulse.Y * body2.inverseMass),
                    body2.linearVelocity.Z + (impulse.Z * body2.inverseMass));

                float num0, num1, num2;
                num0 = (relativePos2.Y * impulse.Z) - (relativePos2.Z * impulse.Y);
                num1 = (relativePos2.Z * impulse.X) - (relativePos2.X * impulse.Z);
                num2 = (relativePos2.X * impulse.Y) - (relativePos2.Y * impulse.X);

                var num3 =
                    (num0 * body2.invInertiaWorld.M11)
                    + (num1 * body2.invInertiaWorld.M21)
                    + (num2 * body2.invInertiaWorld.M31);
                var num4 =
                    (num0 * body2.invInertiaWorld.M12)
                    + (num1 * body2.invInertiaWorld.M22)
                    + (num2 * body2.invInertiaWorld.M32);
                var num5 =
                    (num0 * body2.invInertiaWorld.M13)
                    + (num1 * body2.invInertiaWorld.M23)
                    + (num2 * body2.invInertiaWorld.M33);

                body2.angularVelocity = new JVector(
                    body2.angularVelocity.X + num3,
                    body2.angularVelocity.Y + num4,
                    body2.angularVelocity.Z + num5);
            }
        }

        public void ApplyImpulse(JVector impulse)
        {
            if (!treatBody1AsStatic)
            {
                body1.linearVelocity = new JVector(
                    body1.linearVelocity.X - (impulse.X * body1.inverseMass),
                    body1.linearVelocity.Y - (impulse.Y * body1.inverseMass),
                    body1.linearVelocity.Z - (impulse.Z * body1.inverseMass));

                float num0, num1, num2;
                num0 = (relativePos1.Y * impulse.Z) - (relativePos1.Z * impulse.Y);
                num1 = (relativePos1.Z * impulse.X) - (relativePos1.X * impulse.Z);
                num2 = (relativePos1.X * impulse.Y) - (relativePos1.Y * impulse.X);

                var num3 =
                    (num0 * body1.invInertiaWorld.M11)
                    + (num1 * body1.invInertiaWorld.M21)
                    + (num2 * body1.invInertiaWorld.M31);
                var num4 =
                    (num0 * body1.invInertiaWorld.M12)
                    + (num1 * body1.invInertiaWorld.M22)
                    + (num2 * body1.invInertiaWorld.M32);
                var num5 =
                    (num0 * body1.invInertiaWorld.M13)
                    + (num1 * body1.invInertiaWorld.M23)
                    + (num2 * body1.invInertiaWorld.M33);

                body1.angularVelocity = new JVector(
                    body1.angularVelocity.X + num3,
                    body1.angularVelocity.Y + num4,
                    body1.angularVelocity.Z + num5);
            }

            if (!treatBody2AsStatic)
            {
                body2.linearVelocity = new JVector(
                    body2.linearVelocity.X + (impulse.X * body2.inverseMass),
                    body2.linearVelocity.Y + (impulse.Y * body2.inverseMass),
                    body2.linearVelocity.Z + (impulse.Z * body2.inverseMass));

                float num0, num1, num2;
                num0 = (relativePos2.Y * impulse.Z) - (relativePos2.Z * impulse.Y);
                num1 = (relativePos2.Z * impulse.X) - (relativePos2.X * impulse.Z);
                num2 = (relativePos2.X * impulse.Y) - (relativePos2.Y * impulse.X);

                var num3 =
                    (num0 * body2.invInertiaWorld.M11)
                    + (num1 * body2.invInertiaWorld.M21)
                    + (num2 * body2.invInertiaWorld.M31);
                var num4 =
                    (num0 * body2.invInertiaWorld.M12)
                    + (num1 * body2.invInertiaWorld.M22)
                    + (num2 * body2.invInertiaWorld.M32);
                var num5 =
                    (num0 * body2.invInertiaWorld.M13)
                    + (num1 * body2.invInertiaWorld.M23)
                    + (num2 * body2.invInertiaWorld.M33);

                body2.angularVelocity = new JVector(
                    body2.angularVelocity.X + num3,
                    body2.angularVelocity.Y + num4,
                    body2.angularVelocity.Z + num5);
            }
        }

        public void PrepareForIteration(float timestep)
        {
            float dvx, dvy, dvz;

            dvx = (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y) + body2.linearVelocity.X;
            dvy = (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z) + body2.linearVelocity.Y;
            dvz = (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X) + body2.linearVelocity.Z;

            dvx = dvx - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y) - body1.linearVelocity.X;
            dvy = dvy - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z) - body1.linearVelocity.Y;
            dvz = dvz - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X) - body1.linearVelocity.Z;

            var kNormal = 0.0f;

            var rantra = JVector.Zero;
            if (!treatBody1AsStatic)
            {
                kNormal += body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    rantra = new JVector(
                         (relativePos1.Y * normal.Z) - (relativePos1.Z * normal.Y),
                         (relativePos1.Z * normal.X) - (relativePos1.X * normal.Z),
                         (relativePos1.X * normal.Y) - (relativePos1.Y * normal.X));

                    var num0 = (rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21) + (rantra.Z * body1.invInertiaWorld.M31);
                    var num1 = (rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22) + (rantra.Z * body1.invInertiaWorld.M32);
                    var num2 = (rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23) + (rantra.Z * body1.invInertiaWorld.M33);

                    rantra = new JVector(num0, num1, num2);

                    num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    rantra = new JVector(num0, num1, num2);
                }
            }

            var rbntrb = JVector.Zero;
            if (!treatBody2AsStatic)
            {
                kNormal += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    rbntrb = new JVector(
                        (relativePos2.Y * normal.Z) - (relativePos2.Z * normal.Y),
                        (relativePos2.Z * normal.X) - (relativePos2.X * normal.Z),
                        (relativePos2.X * normal.Y) - (relativePos2.Y * normal.X));

                    var num0 = (rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    var num1 = (rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    var num2 = (rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    rbntrb = new JVector(num0, num1, num2);

                    num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    rbntrb = new JVector(num0, num1, num2);
                }
            }

            if (!treatBody1AsStatic)
            {
                kNormal += (rantra.X * normal.X) + (rantra.Y * normal.Y) + (rantra.Z * normal.Z);
            }

            if (!treatBody2AsStatic)
            {
                kNormal += (rbntrb.X * normal.X) + (rbntrb.Y * normal.Y) + (rbntrb.Z * normal.Z);
            }

            massNormal = 1.0f / kNormal;

            var num = (dvx * normal.X) + (dvy * normal.Y) + (dvz * normal.Z);

            tangent = new JVector(
                dvx - (normal.X * num),
                dvy - (normal.Y * num),
                dvz - (normal.Z * num));

            num = (tangent.X * tangent.X) + (tangent.Y * tangent.Y) + (tangent.Z * tangent.Z);

            if (num != 0.0f)
            {
                num = (float)Math.Sqrt(num);
                tangent = tangent *= (1 / num);
            }

            var kTangent = 0.0f;

            if (treatBody1AsStatic)
            {
                rantra = JVector.Zero;
            }
            else
            {
                kTangent += body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    rantra = new JVector(
                         (relativePos1.Y * tangent.Z) - (relativePos1.Z * tangent.Y),
                         (relativePos1.Z * tangent.X) - (relativePos1.X * tangent.Z),
                         (relativePos1.X * tangent.Y) - (relativePos1.Y * tangent.X));

                    var num0 = (rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21) + (rantra.Z * body1.invInertiaWorld.M31);
                    var num1 = (rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22) + (rantra.Z * body1.invInertiaWorld.M32);
                    var num2 = (rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23) + (rantra.Z * body1.invInertiaWorld.M33);

                    rantra = new JVector(num0, num1, num2);

                    num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    rantra = new JVector(num0, num1, num2);
                }
            }

            if (treatBody2AsStatic)
            {
                rbntrb = JVector.Zero;
            }
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    rbntrb = new JVector(
                        (relativePos2.Y * tangent.Z) - (relativePos2.Z * tangent.Y),
                        (relativePos2.Z * tangent.X) - (relativePos2.X * tangent.Z),
                        (relativePos2.X * tangent.Y) - (relativePos2.Y * tangent.X));

                    var num0 = (rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    var num1 = (rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    var num2 = (rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    rbntrb = new JVector(num0, num1, num2);

                    num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    rbntrb = new JVector(num0, num1, num2);
                }
            }

            if (!treatBody1AsStatic)
            {
                kTangent += JVector.Dot(ref rantra, ref tangent);
            }

            if (!treatBody2AsStatic)
            {
                kTangent += JVector.Dot(ref rbntrb, ref tangent);
            }

            massTangent = 1.0f / kTangent;

            restitutionBias = lostSpeculativeBounce;

            speculativeVelocity = 0.0f;

            var relNormalVel = (normal.X * dvx) + (normal.Y * dvy) + (normal.Z * dvz);

            if (Penetration > settings.allowedPenetration)
            {
                restitutionBias = settings.bias * (1.0f / timestep) * JMath.Max(0.0f, Penetration - settings.allowedPenetration);
                restitutionBias = JMath.Clamp(restitutionBias, 0.0f, settings.maximumBias);
            }

            var timeStepRatio = timestep / lastTimeStep;
            accumulatedNormalImpulse *= timeStepRatio;
            accumulatedTangentImpulse *= timeStepRatio;

            {
                var relTangentVel = -((tangent.X * dvx) + (tangent.Y * dvy) + (tangent.Z * dvz));
                var tangentImpulse = massTangent * relTangentVel;
                var maxTangentImpulse = -StaticFriction * accumulatedNormalImpulse;

                if (tangentImpulse < maxTangentImpulse)
                {
                    friction = DynamicFriction;
                }
                else
                {
                    friction = StaticFriction;
                }
            }

            JVector impulse;

            if (relNormalVel < -1.0f && newContact)
            {
                restitutionBias = Math.Max(-Restitution * relNormalVel, restitutionBias);
            }

            if (penetration < -settings.allowedPenetration)
            {
                speculativeVelocity = penetration / timestep;

                lostSpeculativeBounce = restitutionBias;
                restitutionBias = 0.0f;
            }
            else
            {
                lostSpeculativeBounce = 0.0f;
            }

            impulse = new JVector(
                (normal.X * accumulatedNormalImpulse) + (tangent.X * accumulatedTangentImpulse),
                (normal.Y * accumulatedNormalImpulse) + (tangent.Y * accumulatedTangentImpulse),
                (normal.Z * accumulatedNormalImpulse) + (tangent.Z * accumulatedTangentImpulse));

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity = new JVector(
                    body1.linearVelocity.X - (impulse.X * body1.inverseMass),
                    body1.linearVelocity.Y - (impulse.Y * body1.inverseMass),
                    body1.linearVelocity.Z - (impulse.Z * body1.inverseMass));

                if (!body1IsMassPoint)
                {
                    float num0, num1, num2;
                    num0 = (relativePos1.Y * impulse.Z) - (relativePos1.Z * impulse.Y);
                    num1 = (relativePos1.Z * impulse.X) - (relativePos1.X * impulse.Z);
                    num2 = (relativePos1.X * impulse.Y) - (relativePos1.Y * impulse.X);

                    var num3 =
                        (num0 * body1.invInertiaWorld.M11)
                        + (num1 * body1.invInertiaWorld.M21)
                        + (num2 * body1.invInertiaWorld.M31);
                    var num4 =
                        (num0 * body1.invInertiaWorld.M12)
                        + (num1 * body1.invInertiaWorld.M22)
                        + (num2 * body1.invInertiaWorld.M32);
                    var num5 =
                        (num0 * body1.invInertiaWorld.M13)
                        + (num1 * body1.invInertiaWorld.M23)
                        + (num2 * body1.invInertiaWorld.M33);

                    body1.angularVelocity = new JVector(
                        body1.angularVelocity.X + num3,
                        body1.angularVelocity.Y + num4,
                        body1.angularVelocity.Z + num5);
                }
            }

            if (!treatBody2AsStatic)
            {
                body2.linearVelocity = new JVector(
                    body2.linearVelocity.X + (impulse.X * body2.inverseMass),
                    body2.linearVelocity.Y + (impulse.Y * body2.inverseMass),
                    body2.linearVelocity.Z + (impulse.Z * body2.inverseMass));

                if (!body2IsMassPoint)
                {
                    float num0, num1, num2;
                    num0 = (relativePos2.Y * impulse.Z) - (relativePos2.Z * impulse.Y);
                    num1 = (relativePos2.Z * impulse.X) - (relativePos2.X * impulse.Z);
                    num2 = (relativePos2.X * impulse.Y) - (relativePos2.Y * impulse.X);

                    var num3 =
                        (num0 * body2.invInertiaWorld.M11)
                        + (num1 * body2.invInertiaWorld.M21)
                        + (num2 * body2.invInertiaWorld.M31);
                    var num4 =
                        (num0 * body2.invInertiaWorld.M12)
                        + (num1 * body2.invInertiaWorld.M22)
                        + (num2 * body2.invInertiaWorld.M32);
                    var num5 =
                        (num0 * body2.invInertiaWorld.M13)
                        + (num1 * body2.invInertiaWorld.M23)
                        + (num2 * body2.invInertiaWorld.M33);

                    body2.angularVelocity = new JVector(
                        body2.angularVelocity.X + num3,
                        body2.angularVelocity.Y + num4,
                        body2.angularVelocity.Z + num5);
                }
            }

            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1)
            {
                treatBody1AsStatic = true;
            }
            else
            {
                treatBody2AsStatic = true;
            }
        }

        public void Initialize(
            RigidBody body1,
            RigidBody body2,
            ref JVector point1,
            ref JVector point2,
            ref JVector n,
            float penetration,
            bool newContact,
            ContactSettings settings)
        {
            this.body1 = body1; this.body2 = body2;
            normal = JVector.Normalize(n);
            p1 = point1; p2 = point2;

            this.newContact = newContact;

            JVector.Subtract(ref p1, ref body1.position, out relativePos1);
            JVector.Subtract(ref p2, ref body2.position, out relativePos2);
            JVector.Transform(ref relativePos1, ref body1.invOrientation, out realRelPos1);
            JVector.Transform(ref relativePos2, ref body2.invOrientation, out realRelPos2);

            initialPen = penetration;
            this.penetration = penetration;

            body1IsMassPoint = body1.isParticle;
            body2IsMassPoint = body2.isParticle;

            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = 0.0f;
                accumulatedTangentImpulse = 0.0f;

                lostSpeculativeBounce = 0.0f;

                switch (settings.MaterialCoefficientMixing)
                {
                    case ContactSettings.MaterialCoefficientMixingType.TakeMaximum:
                        StaticFriction = JMath.Max(body1.material.staticFriction, body2.material.staticFriction);
                        DynamicFriction = JMath.Max(body1.material.kineticFriction, body2.material.kineticFriction);
                        Restitution = JMath.Max(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.TakeMinimum:
                        StaticFriction = JMath.Min(body1.material.staticFriction, body2.material.staticFriction);
                        DynamicFriction = JMath.Min(body1.material.kineticFriction, body2.material.kineticFriction);
                        Restitution = JMath.Min(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.UseAverage:
                        StaticFriction = (body1.material.staticFriction + body2.material.staticFriction) / 2.0f;
                        DynamicFriction = (body1.material.kineticFriction + body2.material.kineticFriction) / 2.0f;
                        Restitution = (body1.material.restitution + body2.material.restitution) / 2.0f;
                        break;
                }
            }

            this.settings = settings;
        }
    }
}
