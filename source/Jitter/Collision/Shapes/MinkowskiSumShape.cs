using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision.Shapes
{
    public class MinkowskiSumShape : Shape
    {
        private JVector shifted;
        private readonly List<Shape> shapes = new List<Shape>();

        public MinkowskiSumShape(IEnumerable<Shape> shapes)
        {
            AddShapes(shapes);
        }

        public void AddShapes(IEnumerable<Shape> shapes)
        {
            foreach (var shape in shapes)
            {
                if (shape is Multishape)
                {
                    throw new Exception("Multishapes not supported by MinkowskiSumShape.");
                }

                this.shapes.Add(shape);
            }

            UpdateShape();
        }

        public void AddShape(Shape shape)
        {
            if (shape is Multishape)
            {
                throw new Exception("Multishapes not supported by MinkowskiSumShape.");
            }

            shapes.Add(shape);

            UpdateShape();
        }

        public bool Remove(Shape shape)
        {
            if (shapes.Count == 1)
            {
                throw new Exception("There must be at least one shape.");
            }

            bool result = shapes.Remove(shape);
            UpdateShape();
            return result;
        }

        public JVector Shift()
        {
            return -1 * shifted;
        }

        public override void CalculateMassInertia()
        {
            mass = CalculateMassInertia(this, out shifted, out inertia);
        }

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            var temp2 = JVector.Zero;

            for (int i = 0; i < shapes.Count; i++)
            {
                shapes[i].SupportMapping(in direction, out var temp1);
                JVector.Add( temp1,  temp2, out temp2);
            }

            JVector.Subtract(temp2, shifted, out result);
        }
    }
}
