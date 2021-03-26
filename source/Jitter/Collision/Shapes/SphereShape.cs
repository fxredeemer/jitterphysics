using Jitter.LinearMath;

namespace Jitter.Collision.Shapes
{
    public class SphereShape : Shape
    {
        private float radius = 1.0f;

        public float Radius
        {
            get => radius; set
            {
                radius = value;
                UpdateShape();
            }
        }

        public SphereShape(float radius)
        {
            this.radius = radius;
            UpdateShape();
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            result = direction;
            result.Normalize();

            JVector.Multiply(ref result, radius, out result);
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box.Min.X = -radius;
            box.Min.Y = -radius;
            box.Min.Z = -radius;
            box.Max.X = radius;
            box.Max.Y = radius;
            box.Max.Z = radius;
        }

        public override void CalculateMassInertia()
        {
            mass = 4.0f / 3.0f * JMath.Pi * radius * radius * radius;

            inertia = JMatrix.Identity;
            inertia.M11 = 0.4f * mass * radius * radius;
            inertia.M22 = 0.4f * mass * radius * radius;
            inertia.M33 = 0.4f * mass * radius * radius;
        }
    }
}
