using Jitter.LinearMath;
using System;

namespace Jitter.Collision.Shapes
{
    public class BoxShape : Shape
    {
        private JVector size = JVector.Zero;

        public JVector Size
        {
            get => size;
            set { size = value; UpdateShape(); }
        }

        public BoxShape(JVector size)
        {
            this.size = size;
            UpdateShape();
        }

        public BoxShape(float length, float height, float width)
        {
            size.X = length;
            size.Y = height;
            size.Z = width;
            UpdateShape();
        }

        private JVector halfSize = JVector.Zero;

        public override void UpdateShape()
        {
            halfSize = size * 0.5f;
            base.UpdateShape();
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            JMath.Absolute(ref orientation, out var abs);
            JVector.Transform(ref halfSize, ref abs, out var temp);

            box.Max = temp;
            JVector.Negate(ref temp, out box.Min);
        }

        public override void CalculateMassInertia()
        {
            mass = size.X * size.Y * size.Z;

            inertia = JMatrix.Identity;
            inertia.M11 = (1.0f / 12.0f) * mass * (size.Y * size.Y + size.Z * size.Z);
            inertia.M22 = (1.0f / 12.0f) * mass * (size.X * size.X + size.Z * size.Z);
            inertia.M33 = (1.0f / 12.0f) * mass * (size.X * size.X + size.Y * size.Y);

            geomCen = JVector.Zero;
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            result.X = Math.Sign(direction.X) * halfSize.X;
            result.Y = Math.Sign(direction.Y) * halfSize.Y;
            result.Z = Math.Sign(direction.Z) * halfSize.Z;
        }
    }
}
