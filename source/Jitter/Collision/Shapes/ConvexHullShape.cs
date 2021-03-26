using Jitter.LinearMath;
using System.Collections.Generic;

namespace Jitter.Collision.Shapes
{
    public class ConvexHullShape : Shape
    {
        private readonly List<JVector> vertices;
        private JVector shifted;

        public ConvexHullShape(List<JVector> vertices)
        {
            this.vertices = vertices;
            UpdateShape();
        }

        public JVector Shift => -1 * shifted;

        public override void CalculateMassInertia()
        {
            mass = CalculateMassInertia(this, out shifted, out inertia);
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            float maxDotProduct = float.MinValue;
            int maxIndex = 0;
            float dotProduct;

            for (int i = 0; i < vertices.Count; i++)
            {
                dotProduct = JVector.Dot(vertices[i], direction);
                if (dotProduct > maxDotProduct)
                {
                    maxDotProduct = dotProduct;
                    maxIndex = i;
                }
            }

            result = vertices[maxIndex] - shifted;
        }
    }
}
