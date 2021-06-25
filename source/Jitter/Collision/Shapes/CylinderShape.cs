using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class CylinderShape : Shape
    {
        private float height, radius;

        public float Height
        {
            get => height; set
            {
                height = value;
                UpdateShape();
            }
        }

        public float Radius
        {
            get => radius; set
            {
                radius = value;
                UpdateShape();
            }
        }

        public CylinderShape(float height, float radius)
        {
            this.height = height;
            this.radius = radius;
            UpdateShape();
        }

        public override void CalculateMassInertia()
        {
            mass = JMath.Pi * radius * radius * height;
            inertia.M11 = (1.0f / 4.0f * mass * radius * radius) + (1.0f / 12.0f * mass * height * height);
            inertia.M22 = 1.0f / 2.0f * mass * radius * radius;
            inertia.M33 = (1.0f / 4.0f * mass * radius * radius) + (1.0f / 12.0f * mass * height * height);
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            var sigma = JMath.Sqrt((direction.X * direction.X) + (direction.Z * direction.Z));

            if (sigma > 0.0f)
            {
                result = new JVector(
                    direction.X / sigma * radius,
                    Math.Sign(direction.Y) * height * 0.5f,
                    direction.Z / sigma * radius);
            }
            else
            {
                result = new JVector(
                    0.0f,
                    Math.Sign(direction.Y) * height * 0.5f,
                    0.0f);
            }
        }
    }
}
