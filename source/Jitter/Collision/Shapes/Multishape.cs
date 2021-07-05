using Jitter.LinearMath;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Collision.Shapes
{
    public abstract class Multishape : Shape
    {
        public abstract void SetCurrentShape(int index);

        public abstract int Prepare(in JBBox box);

        public abstract int Prepare(in JVector rayOrigin, in JVector rayDelta);

        protected abstract Multishape CreateWorkingClone();

        internal bool isClone;

        public bool IsClone => isClone;

        private Stack<Multishape> workingCloneStack = new Stack<Multishape>();

        public Multishape RequestWorkingClone()
        {
            Debug.Assert(workingCloneStack.Count < 10, "Unusual size of the workingCloneStack. Forgot to call ReturnWorkingClone?");
            Debug.Assert(!isClone, "Can't clone clones! Something wrong here!");

            Multishape multiShape;

            lock (workingCloneStack)
            {
                if (workingCloneStack.Count == 0)
                {
                    multiShape = CreateWorkingClone();
                    multiShape.workingCloneStack = workingCloneStack;
                    workingCloneStack.Push(multiShape);
                }
                multiShape = workingCloneStack.Pop();
                multiShape.isClone = true;
            }

            return multiShape;
        }

        public override void UpdateShape()
        {
            lock (workingCloneStack)
            {
                workingCloneStack.Clear();
            }

            base.UpdateShape();
        }

        public void ReturnWorkingClone()
        {
            Debug.Assert(isClone, "Only clones can be returned!");
            lock (workingCloneStack) { workingCloneStack.Push(this); }
        }

        public override void GetBoundingBox(in JMatrix orientation, out JBBox box)
        {
            var helpBox = JBBox.LargeBox;
            var length = Prepare(helpBox);

            box = JBBox.SmallBox;

            for (var i = 0; i < length; i++)
            {
                SetCurrentShape(i);
                base.GetBoundingBox(orientation, out helpBox);
                JBBox.CreateMerged(box, helpBox, out box);
            }
        }

        public override void MakeHull(List<JVector> triangleList, int generationThreshold)
        {
        }

        public override void CalculateMassInertia()
        {
            geomCen = JVector.Zero;

            inertia = JMatrix.Identity;

            JVector.Subtract(boundingBox.Max, boundingBox.Min, out var size);

            mass = size.X * size.Y * size.Z;

            inertia = new JMatrix(
                m11: 1.0f / 12.0f * mass * ((size.Y * size.Y) + (size.Z * size.Z)),
                m22: 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Z * size.Z)),
                m33: 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Y * size.Y)));
        }
    }
}
