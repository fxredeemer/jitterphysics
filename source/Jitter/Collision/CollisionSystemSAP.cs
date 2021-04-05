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

        private bool swapOrder;

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
                throw new ArgumentException("The body was already added to the collision system.", nameof(body));
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
                    AddToActiveMultithreaded(bodyList[i]);
                }

                threadManager.Execute();
            }
            else
            {
                for (int i = 0; i < bodyList.Count; i++)
                {
                    AddToActive(bodyList[i]);
                }
            }
        }

        private void AddToActive(IBroadphaseEntity body)
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

                    if (!(thisInactive && ac.IsStaticOrInactive)
                        && (bodyBox.Max.Z >= acBox.Min.Z)
                        && (bodyBox.Min.Z <= acBox.Max.Z)
                        && (bodyBox.Max.Y >= acBox.Min.Y)
                        && (bodyBox.Min.Y <= acBox.Max.Y)
                        && RaisePassedBroadphase(ac, body))
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

                    i++;
                }
            }

            active.Add(body);
        }

        private void AddToActiveMultithreaded(IBroadphaseEntity body)
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

                    if (!(thisInactive && ac.IsStaticOrInactive)
                        && (bodyBox.Max.Z >= acBox.Min.Z)
                        && (bodyBox.Min.Z <= acBox.Max.Z)
                        && (bodyBox.Max.Y >= acBox.Min.Y)
                        && (bodyBox.Min.Y <= acBox.Max.Y)
                        && RaisePassedBroadphase(ac, body))
                    {
                        var pair = BroadphasePair.Pool.GetNew();

                        if (swapOrder)
                        {
                            pair.Entity1 = body;
                            pair.Entity2 = ac;
                        }
                        else
                        {
                            pair.Entity2 = body;
                            pair.Entity1 = ac;
                        }
                        swapOrder = !swapOrder;

                        threadManager.AddTask(detectCallback, pair);
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

        public override bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction)
        {
            body = null; normal = JVector.Zero; fraction = float.MaxValue;

            JVector tempNormal; float tempFraction;
            bool result = false;

            foreach (var e in bodyList)
            {
                if (e is SoftBody softBody)
                {
                    foreach (RigidBody b in softBody.VertexBodies)
                    {
                        if (Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction)
                            && tempFraction < fraction
                            && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
                else
                {
                    var b = e as RigidBody;

                    if (Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction)
                        && tempFraction < fraction
                        && (raycast == null || raycast(b, tempNormal, tempFraction)))
                    {
                        body = b;
                        normal = tempNormal;
                        fraction = tempFraction;
                        result = true;
                    }
                }
            }

            return result;
        }

        public override bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction)
        {
            fraction = float.MaxValue; normal = JVector.Zero;

            if (!body.BoundingBox.RayIntersect(rayOrigin, rayDirection))
            {
                return false;
            }

            if (body.Shape is Multishape)
            {
                var ms = (body.Shape as Multishape).RequestWorkingClone();

                bool multiShapeCollides = false;

                JVector.Subtract(rayOrigin, body.position, out var transformedOrigin);
                JVector.Transform(transformedOrigin, body.invOrientation, out transformedOrigin);
                JVector.Transform(rayDirection, body.invOrientation, out var transformedDirection);

                int msLength = ms.Prepare(transformedOrigin, transformedDirection);

                for (int i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (GJKCollide.Raycast(
                        ms,
                        body.orientation,
                        body.position,
                        rayOrigin,
                        rayDirection,
                        out float tempFraction,
                        out var tempNormal) && tempFraction < fraction)
                    {
                        if (useTerrainNormal && ms is TerrainShape terrainShape)
                        {
                            terrainShape.CollisionNormal(out tempNormal);
                            JVector.Transform(tempNormal, body.orientation, out tempNormal);
                            tempNormal = JVector.Negate(tempNormal);
                        }
                        else if (useTriangleMeshNormal && ms is TriangleMeshShape triangleMeshShape)
                        {
                            triangleMeshShape.CollisionNormal(out tempNormal);
                            JVector.Transform(tempNormal, body.orientation, out tempNormal);
                            tempNormal = JVector.Negate(tempNormal);
                        }

                        normal = tempNormal;
                        fraction = tempFraction;
                        multiShapeCollides = true;
                    }
                }

                ms.ReturnWorkingClone();
                return multiShapeCollides;
            }
            else
            {
                return GJKCollide.Raycast(
                    body.Shape,
                    body.orientation,
                    body.position,
                    rayOrigin,
                    rayDirection,
                    out fraction,
                    out normal);
            }
        }
    }
}
