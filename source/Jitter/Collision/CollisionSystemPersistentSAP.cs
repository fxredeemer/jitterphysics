using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision
{
    public class CollisionSystemPersistentSAP : CollisionSystem
    {
        private const int AddedObjectsBruteForceIsUsed = 250;

        private class SweepPoint
        {
            public IBroadphaseEntity Body;
            public bool Begin;
            public int Axis;

            public SweepPoint(IBroadphaseEntity body, bool begin, int axis)
            {
                Body = body;
                Begin = begin;
                Axis = axis;
            }

            public float Value
            {
                get
                {
                    if (Begin)
                    {
                        if (Axis == 0)
                        {
                            return Body.BoundingBox.Min.X;
                        }
                        else if (Axis == 1)
                        {
                            return Body.BoundingBox.Min.Y;
                        }
                        else
                        {
                            return Body.BoundingBox.Min.Z;
                        }
                    }
                    else
                    {
                        if (Axis == 0)
                        {
                            return Body.BoundingBox.Max.X;
                        }
                        else if (Axis == 1)
                        {
                            return Body.BoundingBox.Max.Y;
                        }
                        else
                        {
                            return Body.BoundingBox.Max.Z;
                        }
                    }
                }
            }
        }

        private struct OverlapPair
        {
            public IBroadphaseEntity Entity1, Entity2;

            public OverlapPair(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
            {
                Entity1 = entity1;
                Entity2 = entity2;
            }

            internal void SetBodies(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
            {
                Entity1 = entity1;
                Entity2 = entity2;
            }

            public override bool Equals(object obj)
            {
                var other = (OverlapPair)obj;
                return (other.Entity1.Equals(Entity1) && other.Entity2.Equals(Entity2))
                    || (other.Entity1.Equals(Entity2) && other.Entity2.Equals(Entity1));
            }

            public override int GetHashCode()
            {
                return Entity1.GetHashCode() + Entity2.GetHashCode();
            }
        }

        private readonly List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();

        private readonly List<SweepPoint> axis1 = new List<SweepPoint>();
        private readonly List<SweepPoint> axis2 = new List<SweepPoint>();
        private readonly List<SweepPoint> axis3 = new List<SweepPoint>();

        private readonly HashSet<OverlapPair> fullOverlaps = new HashSet<OverlapPair>();
        private readonly Action<object> detectCallback, sortCallback;

        public CollisionSystemPersistentSAP()
        {
            detectCallback = new Action<object>(DetectCallback);
            sortCallback = new Action<object>(SortCallback);
        }

        private int QuickSort(SweepPoint sweepPoint1, SweepPoint sweepPoint2)
        {
            float val1 = sweepPoint1.Value;
            float val2 = sweepPoint2.Value;

            if (val1 > val2)
            {
                return 1;
            }
            else if (val2 > val1)
            {
                return -1;
            }
            else
            {
                return 0;
            }
        }

        private readonly List<IBroadphaseEntity> activeList = new List<IBroadphaseEntity>();

        private void DirtySortAxis(List<SweepPoint> axis)
        {
            axis.Sort(QuickSort);
            activeList.Clear();

            for (int i = 0; i < axis.Count; i++)
            {
                var keyelement = axis[i];

                if (keyelement.Begin)
                {
                    foreach (var body in activeList)
                    {
                        if (CheckBoundingBoxes(body, keyelement.Body))
                        {
                            fullOverlaps.Add(new OverlapPair(body, keyelement.Body));
                        }
                    }

                    activeList.Add(keyelement.Body);
                }
                else
                {
                    activeList.Remove(keyelement.Body);
                }
            }
        }

        private void SortAxis(List<SweepPoint> axis)
        {
            for (int j = 1; j < axis.Count; j++)
            {
                var keyelement = axis[j];
                float key = keyelement.Value;

                int i = j - 1;

                while (i >= 0 && axis[i].Value > key)
                {
                    var swapper = axis[i];

                    if (keyelement.Begin
                        && !swapper.Begin
                        && CheckBoundingBoxes(swapper.Body, keyelement.Body))
                    {
                        lock (fullOverlaps)
                        {
                            fullOverlaps.Add(new OverlapPair(swapper.Body, keyelement.Body));
                        }
                    }

                    if (!keyelement.Begin && swapper.Begin)
                    {
                        lock (fullOverlaps)
                        {
                            fullOverlaps.Remove(new OverlapPair(swapper.Body, keyelement.Body));
                        }
                    }

                    axis[i + 1] = swapper;
                    i--;
                }
                axis[i + 1] = keyelement;
            }
        }

        private int addCounter;
        public override void AddEntity(IBroadphaseEntity body)
        {
            bodyList.Add(body);

            axis1.Add(new SweepPoint(body, true, 0));
            axis1.Add(new SweepPoint(body, false, 0));

            axis2.Add(new SweepPoint(body, true, 1));
            axis2.Add(new SweepPoint(body, false, 1));

            axis3.Add(new SweepPoint(body, true, 2));
            axis3.Add(new SweepPoint(body, false, 2));

            addCounter++;
        }

        private readonly Stack<OverlapPair> depricated = new Stack<OverlapPair>();
        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            int count = 0;
            for (int i = 0; i < axis1.Count; i++)
            {
                if (axis1[i].Body == body)
                {
                    count++; axis1.RemoveAt(i); if (count == 2)
                    {
                        break;
                    }
                    i--;
                }
            }

            count = 0;
            for (int i = 0; i < axis2.Count; i++)
            {
                if (axis2[i].Body == body)
                {
                    count++;
                    axis2.RemoveAt(i);
                    if (count == 2)
                    {
                        break;
                    }
                    i--;
                }
            }

            count = 0;
            for (int i = 0; i < axis3.Count; i++)
            {
                if (axis3[i].Body == body)
                {
                    count++;
                    axis3.RemoveAt(i);
                    if (count == 2)
                    {
                        break;
                    }
                    i--;
                }
            }

            foreach (var pair in fullOverlaps)
            {
                if (pair.Entity1 == body || pair.Entity2 == body)
                {
                    depricated.Push(pair);
                }
            }

            while (depricated.Count > 0)
            {
                fullOverlaps.Remove(depricated.Pop());
            }

            bodyList.Remove(body);

            return true;
        }

        private bool swapOrder;

        public override void Detect(bool multiThreaded)
        {
            if (addCounter > AddedObjectsBruteForceIsUsed)
            {
                fullOverlaps.Clear();

                DirtySortAxis(axis1);
                DirtySortAxis(axis2);
                DirtySortAxis(axis3);
            }
            else
            {
                if (multiThreaded)
                {
                    threadManager.AddTask(sortCallback, axis1);
                    threadManager.AddTask(sortCallback, axis2);
                    threadManager.AddTask(sortCallback, axis3);

                    threadManager.Execute();
                }
                else
                {
                    sortCallback(axis1);
                    sortCallback(axis2);
                    sortCallback(axis3);
                }
            }

            addCounter = 0;

            foreach (var key in fullOverlaps)
            {
                if (CheckBothStaticOrInactive(key.Entity1, key.Entity2))
                {
                    continue;
                }

                if (RaisePassedBroadphase(key.Entity1, key.Entity2))
                {
                    if (multiThreaded)
                    {
                        var pair = BroadphasePair.Pool.GetNew();
                        if (swapOrder)
                        {
                            pair.Entity1 = key.Entity1;
                            pair.Entity2 = key.Entity2;
                        }
                        else
                        {
                            pair.Entity2 = key.Entity2;
                            pair.Entity1 = key.Entity1;
                        }
                        threadManager.AddTask(detectCallback, pair);
                    }
                    else
                    {
                        if (swapOrder)
                        {
                            Detect(key.Entity1, key.Entity2);
                        }
                        else
                        {
                            Detect(key.Entity2, key.Entity1);
                        }
                    }

                    swapOrder = !swapOrder;
                }
            }

            threadManager.Execute();
        }

        private void SortCallback(object obj)
        {
            SortAxis(obj as List<SweepPoint>);
        }

        private void DetectCallback(object obj)
        {
            var pair = obj as BroadphasePair;
            base.Detect(pair.Entity1, pair.Entity2);
            BroadphasePair.Pool.GiveBack(pair);
        }

        public override bool Raycast(
            JVector rayOrigin,
            JVector rayDirection,
            RaycastCallback raycast,
            out RigidBody body,
            out JVector normal,
            out float fraction)
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
                        if (Raycast(
                            b,
                            rayOrigin,
                            rayDirection,
                            out tempNormal,
                            out tempFraction) && tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
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

                    if (Raycast(
                        b,
                        rayOrigin,
                        rayDirection,
                        out tempNormal,
                        out tempFraction) && tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
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
            fraction = float.MaxValue;
            normal = JVector.Zero;

            if (!body.BoundingBox.RayIntersect(rayOrigin, rayDirection))
            {
                return false;
            }

            if (body.Shape is Multishape multishape)
            {
                multishape = multishape.RequestWorkingClone();

                bool multiShapeCollides = false;

                JVector.Subtract(in rayOrigin, in body.position, out var transformedOrigin);
                JVector.Transform(in transformedOrigin, in body.invOrientation, out transformedOrigin);
                JVector.Transform(in rayDirection, in body.invOrientation, out var transformedDirection);

                int msLength = multishape.Prepare(ref transformedOrigin, ref transformedDirection);

                for (int i = 0; i < msLength; i++)
                {
                    multishape.SetCurrentShape(i);

                    if (GJKCollide.Raycast(
                        multishape,
                        ref body.orientation,
                        ref body.position,
                        ref rayOrigin,
                        ref rayDirection,
                        out float tempFraction,
                        out var tempNormal) && tempFraction < fraction)
                    {
                        if (useTerrainNormal && multishape is TerrainShape terrainShape)
                        {
                            terrainShape.CollisionNormal(out tempNormal);
                            JVector.Transform(in tempNormal, in body.orientation, out tempNormal);
                            tempNormal = JVector.Negate(tempNormal);
                        }
                        else if (useTriangleMeshNormal && multishape is TriangleMeshShape triangleMeshShape)
                        {
                            triangleMeshShape.CollisionNormal(out tempNormal);
                            JVector.Transform(in tempNormal, in body.orientation, out tempNormal);
                            tempNormal = JVector.Negate(tempNormal);
                        }

                        normal = tempNormal;
                        fraction = tempFraction;
                        multiShapeCollides = true;
                    }
                }

                multishape.ReturnWorkingClone();
                return multiShapeCollides;
            }
            else
            {
                return GJKCollide.Raycast(
                    body.Shape,
                    ref body.orientation,
                    ref body.position,
                    ref rayOrigin,
                    ref rayDirection,
                    out fraction,
                    out normal);
            }
        }
    }
}
