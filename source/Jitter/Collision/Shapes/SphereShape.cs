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

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            result = JVector.Normalize(direction);

            JVector.Multiply(result, radius, out result);
        }

        public override void GetBoundingBox(in JMatrix orientation, out JBBox box)
        {
            box = new JBBox(
                new JVector(-radius, -radius, -radius),
                new JVector(radius, radius, radius));
        }

        public override void CalculateMassInertia()
        {
            mass = 4.0f / 3.0f * JMath.Pi * radius * radius * radius;

            inertia = new JMatrix(
                m11: 0.4f * mass * radius * radius,
                m22: 0.4f * mass * radius * radius,
                m33: 0.4f * mass * radius * radius);
        }
    }
}
