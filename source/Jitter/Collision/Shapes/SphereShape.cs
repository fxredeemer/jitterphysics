﻿using Jitter.LinearMath;

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
            result = JVector.Normalize(result);

            JVector.Multiply(result, radius, out result);
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box = new JBBox(
                new JVector(-radius, -radius, -radius),
                new JVector(radius, radius, radius));
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
