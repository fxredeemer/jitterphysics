using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class ConeShape : Shape
    {
        private float height, radius;

        public float Height
        {
            get => height;
            set
            {
                height = value;
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

        public ConeShape(float height, float radius)
        {
            this.height = height;
            this.radius = radius;

            UpdateShape();
        }

        public override void UpdateShape()
        {
            sina = radius / (float)Math.Sqrt((radius * radius) + (height * height));
            base.UpdateShape();
        }

        private float sina;

        public override void CalculateMassInertia()
        {
            mass = 1.0f / 3.0f * JMath.Pi * radius * radius * height;

            inertia = JMatrix.Identity;
            inertia.M11 = 3.0f / 80.0f * mass * ((radius * radius) + (4 * height * height));
            inertia.M22 = 3.0f / 10.0f * mass * radius * radius;
            inertia.M33 = 3.0f / 80.0f * mass * ((radius * radius) + (4 * height * height));

            geomCen = JVector.Zero;
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            float sigma = (float)Math.Sqrt((direction.X * direction.X) + (direction.Z * direction.Z));

            if (direction.Y > direction.Length() * sina)
            {
                result.X = 0.0f;
                result.Y = 2.0f / 3.0f * height;
                result.Z = 0.0f;
            }
            else if (sigma > 0.0f)
            {
                result.X = radius * direction.X / sigma;
                result.Y = -(1.0f / 3.0f) * height;
                result.Z = radius * direction.Z / sigma;
            }
            else
            {
                result.X = 0.0f;
                result.Y = -(1.0f / 3.0f) * height;
                result.Z = 0.0f;
            }
        }
    }
}
