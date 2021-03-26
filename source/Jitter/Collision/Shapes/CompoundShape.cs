using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision.Shapes
{
    public class CompoundShape : Multishape
    {
        public struct TransformedShape
        {
            internal JVector position;
            internal JMatrix orientation;
            internal JMatrix invOrientation;
            internal JBBox boundingBox;

            public Shape Shape { get; set; }

            public JVector Position
            {
                get => position;
                set
                {
                    position = value;
                    UpdateBoundingBox();
                }
            }

            public JBBox BoundingBox => boundingBox;

            public JMatrix InverseOrientation => invOrientation;

            public JMatrix Orientation
            {
                get => orientation;
                set { orientation = value; JMatrix.Transpose(ref orientation, out invOrientation); UpdateBoundingBox(); }
            }

            public void UpdateBoundingBox()
            {
                Shape.GetBoundingBox(ref orientation, out boundingBox);

                boundingBox.Min += position;
                boundingBox.Max += position;
            }

            public TransformedShape(Shape shape, JMatrix orientation, JVector position)
            {
                this.position = position;
                this.orientation = orientation;
                JMatrix.Transpose(ref orientation, out invOrientation);
                Shape = shape;
                boundingBox = new JBBox();
                UpdateBoundingBox();
            }
        }

        public TransformedShape[] Shapes { get; private set; }

        private JVector shifted;
        public JVector Shift => -1.0f * shifted;

        private JBBox mInternalBBox;

        public CompoundShape(List<TransformedShape> shapes)
        {
            Shapes = new TransformedShape[shapes.Count];
            shapes.CopyTo(Shapes);

            if (!TestValidity())
            {
                throw new ArgumentException("Multishapes are not supported!");
            }

            UpdateShape();
        }

        public CompoundShape(TransformedShape[] shapes)
        {
            Shapes = new TransformedShape[shapes.Length];
            Array.Copy(shapes, Shapes, shapes.Length);

            if (!TestValidity())
            {
                throw new ArgumentException("Multishapes are not supported!");
            }

            UpdateShape();
        }

        private bool TestValidity()
        {
            for (int i = 0; i < Shapes.Length; i++)
            {
                if (Shapes[i].Shape is Multishape)
                {
                    return false;
                }
            }

            return true;
        }

        public override void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
            var triangles = new List<JVector>();

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].Shape.MakeHull(ref triangles, 4);
                for (int e = 0; e < triangles.Count; e++)
                {
                    var pos = triangles[e];
                    JVector.Transform(ref pos, ref Shapes[i].orientation, out pos);
                    JVector.Add(ref pos, ref Shapes[i].position, out pos);
                    triangleList.Add(pos);
                }
                triangles.Clear();
            }
        }

        private void DoShifting()
        {
            for (int i = 0; i < Shapes.Length; i++)
            {
                shifted += Shapes[i].position;
            }

            shifted *= 1.0f / Shapes.Length;

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].position -= shifted;
            }
        }

        public override void CalculateMassInertia()
        {
            inertia = JMatrix.Zero;
            mass = 0.0f;

            for (int i = 0; i < Shapes.Length; i++)
            {
                var currentInertia = Shapes[i].InverseOrientation * Shapes[i].Shape.Inertia * Shapes[i].Orientation;
                var p = Shapes[i].Position * -1.0f;
                float m = Shapes[i].Shape.Mass;

                currentInertia.M11 += m * ((p.Y * p.Y) + (p.Z * p.Z));
                currentInertia.M22 += m * ((p.X * p.X) + (p.Z * p.Z));
                currentInertia.M33 += m * ((p.X * p.X) + (p.Y * p.Y));

                currentInertia.M12 += -p.X * p.Y * m;
                currentInertia.M21 += -p.X * p.Y * m;

                currentInertia.M31 += -p.X * p.Z * m;
                currentInertia.M13 += -p.X * p.Z * m;

                currentInertia.M32 += -p.Y * p.Z * m;
                currentInertia.M23 += -p.Y * p.Z * m;

                inertia += currentInertia;
                mass += m;
            }
        }

        internal CompoundShape()
        {
        }

        protected override Multishape CreateWorkingClone()
        {
            return new CompoundShape
            {
                Shapes = Shapes
            };
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            JVector.Transform(ref direction, ref Shapes[currentShape].invOrientation, out result);
            Shapes[currentShape].Shape.SupportMapping(ref direction, out result);
            JVector.Transform(ref result, ref Shapes[currentShape].orientation, out result);
            JVector.Add(ref result, ref Shapes[currentShape].position, out result);
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box.Min = mInternalBBox.Min;
            box.Max = mInternalBBox.Max;

            var localHalfExtents = 0.5f * (box.Max - box.Min);
            var localCenter = 0.5f * (box.Max + box.Min);

            JVector.Transform(ref localCenter, ref orientation, out var center);

            JMath.Absolute(ref orientation, out var abs);
            JVector.Transform(ref localHalfExtents, ref abs, out var temp);

            box.Max = center + temp;
            box.Min = center - temp;
        }

        private int currentShape;
        private readonly List<int> currentSubShapes = new List<int>();

        public override void SetCurrentShape(int index)
        {
            currentShape = currentSubShapes[index];
            Shapes[currentShape].Shape.SupportCenter(out geomCen);
            geomCen += Shapes[currentShape].Position;
        }

        public override int Prepare(ref JBBox box)
        {
            currentSubShapes.Clear();

            for (int i = 0; i < Shapes.Length; i++)
            {
                if (Shapes[i].boundingBox.Contains(ref box) != JBBox.ContainmentType.Disjoint)
                {
                    currentSubShapes.Add(i);
                }
            }

            return currentSubShapes.Count;
        }

        public override int Prepare(ref JVector rayOrigin, ref JVector rayEnd)
        {
            var box = JBBox.SmallBox;

            box.AddPoint(ref rayOrigin);
            box.AddPoint(ref rayEnd);

            return Prepare(ref box);
        }

        public override void UpdateShape()
        {
            DoShifting();
            UpdateInternalBoundingBox();
            base.UpdateShape();
        }

        protected void UpdateInternalBoundingBox()
        {
            mInternalBBox.Min = new JVector(float.MaxValue);
            mInternalBBox.Max = new JVector(float.MinValue);

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].UpdateBoundingBox();

                JBBox.CreateMerged(ref mInternalBBox, ref Shapes[i].boundingBox, out mInternalBBox);
            }
        }
    }
}
