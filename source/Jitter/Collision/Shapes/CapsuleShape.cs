using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class CapsuleShape : Shape
    {
        private float length, radius;

        public float Length
        {
            get => length;
            set
            {
                length = value;
                UpdateShape();
            }
        }

        public float Radius
        {
            get => radius;
            set
            {
                radius = value;
                UpdateShape();
            }
        }

        public CapsuleShape(float length, float radius)
        {
            this.length = length;
            this.radius = radius;
            UpdateShape();
        }

        public override void CalculateMassInertia()
        {
            float massSphere = 3.0f / 4.0f * JMath.Pi * radius * radius * radius;
            float massCylinder = JMath.Pi * radius * radius * length;

            mass = massCylinder + massSphere;

            inertia.M11 = (1.0f / 4.0f * massCylinder * radius * radius) + (1.0f / 12.0f * massCylinder * length * length) + (2.0f / 5.0f * massSphere * radius * radius) + (1.0f / 4.0f * length * length * massSphere);
            inertia.M22 = (1.0f / 2.0f * massCylinder * radius * radius) + (2.0f / 5.0f * massSphere * radius * radius);
            inertia.M33 = (1.0f / 4.0f * massCylinder * radius * radius) + (1.0f / 12.0f * massCylinder * length * length) + (2.0f / 5.0f * massSphere * radius * radius) + (1.0f / 4.0f * length * length * massSphere);
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            float r = JMath.Sqrt((direction.X * direction.X) + (direction.Z * direction.Z));

            if (Math.Abs(direction.Y) > 0.0f)
            {
                JVector.Normalize(ref direction, out var dir);
                JVector.Multiply(ref dir, radius, out result);
                result.Y += Math.Sign(direction.Y) * 0.5f * length;
            }
            else if (r > 0.0f)
            {
                result.X = direction.X / r * radius;
                result.Y = 0.0f;
                result.Z = direction.Z / r * radius;
            }
            else
            {
                result.X = 0.0f;
                result.Y = 0.0f;
                result.Z = 0.0f;
            }
        }
    }
}
