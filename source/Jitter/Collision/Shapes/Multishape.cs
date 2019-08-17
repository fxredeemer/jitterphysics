using Jitter.LinearMath;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Collision.Shapes
{
    public abstract class Multishape : Shape
    {
        public abstract void SetCurrentShape(int index);

        public abstract int Prepare(ref JBBox box);

        public abstract int Prepare(ref JVector rayOrigin, ref JVector rayDelta);

        protected abstract Multishape CreateWorkingClone();

        internal bool isClone = false;

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

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            var helpBox = JBBox.LargeBox;
            int length = Prepare(ref helpBox);

            box = JBBox.SmallBox;

            for (int i = 0; i < length; i++)
            {
                SetCurrentShape(i);
                base.GetBoundingBox(ref orientation, out helpBox);
                JBBox.CreateMerged(ref box, ref helpBox, out box);
            }
        }

        public override void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
        }

        public override void CalculateMassInertia()
        {
            geomCen = JVector.Zero;

            inertia = JMatrix.Identity;

            JVector.Subtract(ref boundingBox.Max, ref boundingBox.Min, out var size);

            mass = size.X * size.Y * size.Z;

            inertia.M11 = 1.0f / 12.0f * mass * ((size.Y * size.Y) + (size.Z * size.Z));
            inertia.M22 = 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Z * size.Z));
            inertia.M33 = 1.0f / 12.0f * mass * ((size.X * size.X) + (size.Y * size.Y));
        }
    }
}
