using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class CapsuleShape : Shape
    {
        private float length;
        private float radius;

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

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            float r = JMath.Sqrt((direction.X * direction.X) + (direction.Z * direction.Z));

            if (Math.Abs(direction.Y) > 0.0f)
            {
                JVector.Normalize(in direction, out var dir);
                JVector.Multiply(in dir, radius, out result);

                result = new JVector(
                    result.X,
                    result.Y + Math.Sign(direction.Y) * 0.5f * length,
                    result.Z);
            }
            else if (r > 0.0f)
            {
                result = new JVector(
                    direction.X / r * radius,
                    0.0f,
                    direction.Z / r * radius);
            }
            else
            {
                result = new JVector();
            }
        }
    }
}
