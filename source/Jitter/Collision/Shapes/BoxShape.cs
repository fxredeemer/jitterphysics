using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class BoxShape : Shape
    {
        private JVector size = JVector.Zero;
        private JVector halfSize = JVector.Zero;

        public JVector Size
        {
            get => size;
            set
            {
                size = value;
                UpdateShape();
            }
        }

        public BoxShape(JVector size)
        {
            this.size = size;
            UpdateShape();
        }

        public BoxShape(float length, float height, float width)
        {
            size = new JVector(length, height, width);
            UpdateShape();
        }

        public override void UpdateShape()
        {
            halfSize = size * 0.5f;
            base.UpdateShape();
        }

        public override void GetBoundingBox(in JMatrix orientation, out JBBox box)
        {
            JMath.Absolute(orientation, out var abs);
            var max = JVector.Transform(halfSize, abs);
            var min = JVector.Negate(max);
            box = new JBBox(min, max);
        }

        public override void CalculateMassInertia()
        {
            mass = size.X * size.Y * size.Z;

            inertia = JMatrix.Identity;

            inertia = new JMatrix(
                m11: 1.0f / 12.0f * mass * ((size.Y * size.Y) + (size.Z * size.Z)),
                m22: 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Z * size.Z)),
                m33: 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Y * size.Y)));

            geomCen = JVector.Zero;
        }

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            result = new JVector(
                Math.Sign(direction.X) * halfSize.X,
                Math.Sign(direction.Y) * halfSize.Y,
                Math.Sign(direction.Z) * halfSize.Z);
        }
    }
}
