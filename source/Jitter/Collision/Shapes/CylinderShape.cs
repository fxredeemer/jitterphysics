using System;
using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{
    public class CylinderShape : Shape
    {
        private float height, radius;

        public float Height { get { return height; } set { height = value; UpdateShape(); } }

        public float Radius { get { return radius; } set { radius = value; UpdateShape(); } }

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
            float sigma = (float)Math.Sqrt((float)((direction.X * direction.X) + (direction.Z * direction.Z)));

            if (sigma > 0.0f)
            {
                result.X = direction.X / sigma * radius;
                result.Y = (float)Math.Sign(direction.Y) * height * 0.5f;
                result.Z = direction.Z / sigma * radius;
            }
            else
            {
                result.X = 0.0f;
                result.Y = (float)Math.Sign(direction.Y) * height * 0.5f;
                result.Z = 0.0f;
            }
        }
    }
}
