using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision
{
    public class CollisionSystemSAP : CollisionSystem
    {
        private readonly List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();
        private readonly List<IBroadphaseEntity> active = new List<IBroadphaseEntity>();

        private class IBroadphaseEntityXCompare : IComparer<IBroadphaseEntity>
        {
            public int Compare(IBroadphaseEntity body1, IBroadphaseEntity body2)
            {
                float f = body1.BoundingBox.Min.X - body2.BoundingBox.Min.X;
                return (f < 0) ? -1 : (f > 0) ? 1 : 0;
            }
        }

        private readonly IBroadphaseEntityXCompare xComparer;

        private bool swapOrder = false;

        public CollisionSystemSAP()
        {
            xComparer = new IBroadphaseEntityXCompare();
            detectCallback = new Action<object>(DetectCallback);
        }

        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            return bodyList.Remove(body);
        }

        public override void AddEntity(IBroadphaseEntity body)
        {
            if (bodyList.Contains(body))
            {
                throw new ArgumentException("The body was already added to the collision system.", "body");
            }

            bodyList.Add(body);
        }

        private readonly Action<object> detectCallback;

        public override void Detect(bool multiThreaded)
        {
            bodyList.Sort(xComparer);

            active.Clear();

            if (multiThreaded)
            {
                for (int i = 0; i < bodyList.Count; i++)
                {
                    AddToActiveMultithreaded(bodyList[i], false);
                }

                threadManager.Execute();
            }
            else
            {
                for (int i = 0; i < bodyList.Count; i++)
                {
                    AddToActive(bodyList[i], false);
                }
            }
        }

        private void AddToActive(IBroadphaseEntity body, bool addToList)
        {
            float xmin = body.BoundingBox.Min.X;
            int n = active.Count;

            bool thisInactive = body.IsStaticOrInactive;

            JBBox acBox, bodyBox;

            for (int i = 0; i != n;)
            {
                var ac = active[i];
                acBox = ac.BoundingBox;

                if (acBox.Max.X < xmin)
                {
                    n--;
                    active.RemoveAt(i);
                }
                else
                {
                    bodyBox = body.BoundingBox;

                    if (!(thisInactive && ac.IsStaticOrInactive) &&
                        (((bodyBox.Max.Z >= acBox.Min.Z) && (bodyBox.Min.Z <= acBox.Max.Z)) &&
                        ((bodyBox.Max.Y >= acBox.Min.Y) && (bodyBox.Min.Y <= acBox.Max.Y))))
                    {
                        if (base.RaisePassedBroadphase(ac, body))
                        {
                            if (swapOrder)
                            {
                                Detect(body, ac);
                            }
                            else
                            {
                                Detect(ac, body);
                            }

                            swapOrder = !swapOrder;
                        }
                    }

                    i++;
                }
            }

            active.Add(body);
        }

        private void AddToActiveMultithreaded(IBroadphaseEntity body, bool addToList)
        {
            float xmin = body.BoundingBox.Min.X;
            int n = active.Count;

            bool thisInactive = body.IsStaticOrInactive;

            JBBox acBox, bodyBox;

            for (int i = 0; i != n;)
            {
                var ac = active[i];
                acBox = ac.BoundingBox;

                if (acBox.Max.X < xmin)
                {
                    n--;
                    active.RemoveAt(i);
                }
                else
                {
                    bodyBox = body.BoundingBox;

                    if (!(thisInactive && ac.IsStaticOrInactive) &&
                        (((bodyBox.Max.Z >= acBox.Min.Z) && (bodyBox.Min.Z <= acBox.Max.Z)) &&
                        ((bodyBox.Max.Y >= acBox.Min.Y) && (bodyBox.Min.Y <= acBox.Max.Y))))
                    {
                        if (base.RaisePassedBroadphase(ac, body))
                        {
                            var pair = BroadphasePair.Pool.GetNew();

                            if (swapOrder) { pair.Entity1 = body; pair.Entity2 = ac; }
                            else { pair.Entity2 = body; pair.Entity1 = ac; }
                            swapOrder = !swapOrder;

                            threadManager.AddTask(detectCallback, pair);
                        }
                    }

                    i++;
                }
            }

            active.Add(body);
        }

        private void DetectCallback(object obj)
        {
            var pair = obj as BroadphasePair;
            base.Detect(pair.Entity1, pair.Entity2);
            BroadphasePair.Pool.GiveBack(pair);
        }

        private int Compare(IBroadphaseEntity body1, IBroadphaseEntity body2)
        {
            float f = body1.BoundingBox.Min.X - body2.BoundingBox.Min.X;
            return (f < 0) ? -1 : (f > 0) ? 1 : 0;
        }

        public override bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction)
        {
            body = null; normal = JVector.Zero; fraction = float.MaxValue;

            JVector tempNormal; float tempFraction;
            bool result = false;

            foreach (var e in bodyList)
            {
                if (e is SoftBody)
                {
                    var softBody = e as SoftBody;
                    foreach (RigidBody b in softBody.VertexBodies)
                    {
                        if (Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                        {
                            if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                            {
                                body = b;
                                normal = tempNormal;
                                fraction = tempFraction;
                                result = true;
                            }
                        }
                    }
                }
                else
                {
                    var b = e as RigidBody;

                    if (Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
            }

            return result;
        }

        public override bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction)
        {
            fraction = float.MaxValue; normal = JVector.Zero;

            if (!body.BoundingBox.RayIntersect(ref rayOrigin, ref rayDirection))
            {
                return false;
            }

            if (body.Shape is Multishape)
            {
                var ms = (body.Shape as Multishape).RequestWorkingClone();

                bool multiShapeCollides = false;

                JVector.Subtract(ref rayOrigin, ref body.position, out var transformedOrigin);
                JVector.Transform(ref transformedOrigin, ref body.invOrientation, out transformedOrigin);
                JVector.Transform(ref rayDirection, ref body.invOrientation, out var transformedDirection);

                int msLength = ms.Prepare(ref transformedOrigin, ref transformedDirection);

                for (int i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (GJKCollide.Raycast(ms, ref body.orientation, ref body.invOrientation, ref body.position,
                        ref rayOrigin, ref rayDirection, out float tempFraction, out var tempNormal))
                    {
                        if (tempFraction < fraction)
                        {
                            if (useTerrainNormal && ms is TerrainShape)
                            {
                                (ms as TerrainShape).CollisionNormal(out tempNormal);
                                JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                                tempNormal.Negate();
                            }
                            else if (useTriangleMeshNormal && ms is TriangleMeshShape)
                            {
                                (ms as TriangleMeshShape).CollisionNormal(out tempNormal);
                                JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                                tempNormal.Negate();
                            }

                            normal = tempNormal;
                            fraction = tempFraction;
                            multiShapeCollides = true;
                        }
                    }
                }

                ms.ReturnWorkingClone();
                return multiShapeCollides;
            }
            else
            {
                return (GJKCollide.Raycast(body.Shape, ref body.orientation, ref body.invOrientation, ref body.position,
                    ref rayOrigin, ref rayDirection, out fraction, out normal));
            }
        }
    }
}
