﻿using Jitter.LinearMath;
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
                set
                {
                    orientation = value;
                    JMatrix.Transpose(orientation, out invOrientation);
                    UpdateBoundingBox();
                }
            }

            public void UpdateBoundingBox()
            {
                Shape.GetBoundingBox(orientation, out boundingBox);

                boundingBox = new JBBox(
                        boundingBox.Min + position,
                        boundingBox.Max + position);
            }

            public TransformedShape(Shape shape, JMatrix orientation, JVector position)
            {
                this.position = position;
                this.orientation = orientation;
                JMatrix.Transpose(orientation, out invOrientation);
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

        public override void MakeHull(List<JVector> triangleList, int generationThreshold)
        {
            var triangles = new List<JVector>();

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].Shape.MakeHull(triangles, 4);
                for (int e = 0; e < triangles.Count; e++)
                {
                    var pos = triangles[e];
                    JVector.Transform(pos, Shapes[i].orientation, out pos);
                    JVector.Add(pos, Shapes[i].position, out pos);
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


                currentInertia = new JMatrix(
                    m11: currentInertia.M11 + m * ((p.Y * p.Y) + (p.Z * p.Z)),
                    m12: currentInertia.M12 + -p.X * p.Y * m,
                    m13: currentInertia.M13 + -p.X * p.Z * m,
                    m21: currentInertia.M21 + -p.X * p.Y * m,
                    m22: currentInertia.M22 + m * ((p.X * p.X) + (p.Z * p.Z)),
                    m23: currentInertia.M23 + -p.Y * p.Z * m,
                    m31: currentInertia.M31 + -p.X * p.Z * m,
                    m32: currentInertia.M32 + -p.Y * p.Z * m,
                    m33: currentInertia.M33 + m * ((p.X * p.X) + (p.Y * p.Y)));

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

        public override void SupportMapping(in JVector direction, out JVector result)
        {
            JVector.Transform(direction, Shapes[currentShape].invOrientation, out result);
            Shapes[currentShape].Shape.SupportMapping(direction, out result);
            JVector.Transform(result, Shapes[currentShape].orientation, out result);
            JVector.Add(result, Shapes[currentShape].position, out result);
        }

        public override void GetBoundingBox(in JMatrix orientation, out JBBox box)
        {
            box = new JBBox(
                mInternalBBox.Min,
                mInternalBBox.Max);

            var localHalfExtents = 0.5f * (box.Max - box.Min);
            var localCenter = 0.5f * (box.Max + box.Min);

            JVector.Transform(localCenter, orientation, out var center);

            JMath.Absolute(orientation, out var abs);
            JVector.Transform(localHalfExtents, abs, out var temp);

            box = new JBBox(
                center - temp,
                center + temp);
        }

        private int currentShape;
        private readonly List<int> currentSubShapes = new List<int>();

        public override void SetCurrentShape(int index)
        {
            currentShape = currentSubShapes[index];
            Shapes[currentShape].Shape.SupportCenter(out geomCen);
            geomCen += Shapes[currentShape].Position;
        }

        public override int Prepare(in JBBox box)
        {
            currentSubShapes.Clear();

            for (int i = 0; i < Shapes.Length; i++)
            {
                if (Shapes[i].boundingBox.Contains(box) != JBBox.ContainmentType.Disjoint)
                {
                    currentSubShapes.Add(i);
                }
            }

            return currentSubShapes.Count;
        }

        public override int Prepare(in JVector rayOrigin, in JVector rayEnd)
        {
            var box = JBBox.SmallBox;

            box = box.AddPoint(rayOrigin);
            box = box.AddPoint(rayEnd);

            return Prepare(box);
        }

        public override void UpdateShape()
        {
            DoShifting();
            UpdateInternalBoundingBox();
            base.UpdateShape();
        }

        protected void UpdateInternalBoundingBox()
        {
            mInternalBBox = new JBBox(
                new JVector(float.MaxValue),
                new JVector(float.MinValue));

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].UpdateBoundingBox();

                JBBox.CreateMerged(mInternalBBox, Shapes[i].boundingBox, out mInternalBBox);
            }
        }
    }
}
