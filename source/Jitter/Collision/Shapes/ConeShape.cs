using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class ConeShape : Shape
    {
        private float height;
        private float radius;
        private float sina;

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
            sina = radius / JMath.Sqrt((radius * radius) + (height * height));
            base.UpdateShape();
        }

        public override void CalculateMassInertia()
        {
            mass = 1.0f / 3.0f * JMath.Pi * radius * radius * height;

            inertia = JMatrix.FromDiagonal(
                m11: 3.0f / 80.0f * mass * ((radius * radius) + (4 * height * height)),
                m22: 3.0f / 10.0f * mass * radius * radius,
                m33: 3.0f / 80.0f * mass * ((radius * radius) + (4 * height * height)));

            geomCen = JVector.Zero;
        }

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            float sigma = JMath.Sqrt((direction.X * direction.X) + (direction.Z * direction.Z));

            if (direction.Y > direction.Length() * sina)
            {
                result = new JVector(
                    0.0f,
                    2.0f / 3.0f * height,
                    0.0f);
            }
            else if (sigma > 0.0f)
            {
                result = new JVector(
                    radius * direction.X / sigma,
                    -(1.0f / 3.0f) * height,
                    radius * direction.Z / sigma);
            }
            else
            {
                result = new JVector(
                    0.0f,
                    -(1.0f / 3.0f) * height,
                    0.0f);
            }
        }
    }
}
