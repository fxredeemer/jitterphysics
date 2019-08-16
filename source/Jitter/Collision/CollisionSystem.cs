﻿using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Collision
{
    public interface IBroadphaseEntity
    {
        JBBox BoundingBox { get; }
        bool IsStaticOrInactive { get; }
    }

    public delegate void CollisionDetectedHandler(RigidBody body1, RigidBody body2,
                JVector point1, JVector point2, JVector normal, float penetration);

    public delegate bool PassedBroadphaseHandler(IBroadphaseEntity entity1, IBroadphaseEntity entity2);

    public delegate bool PassedNarrowphaseHandler(RigidBody body1, RigidBody body2,
                ref JVector point, ref JVector normal, float penetration);

    public delegate bool RaycastCallback(RigidBody body, JVector normal, float fraction);

    public abstract class CollisionSystem
    {
        protected class BroadphasePair
        {
            public IBroadphaseEntity Entity1;
            public IBroadphaseEntity Entity2;

            public static ResourcePool<BroadphasePair> Pool = new ResourcePool<BroadphasePair>();
        }

        public abstract bool RemoveEntity(IBroadphaseEntity body);

        public abstract void AddEntity(IBroadphaseEntity body);

        public event PassedBroadphaseHandler PassedBroadphase;

        public event CollisionDetectedHandler CollisionDetected;

        protected ThreadManager threadManager = ThreadManager.Instance;

        public bool EnableSpeculativeContacts { get; set; } = false;

        public CollisionSystem()
        {
        }

        internal bool useTerrainNormal = true;
        internal bool useTriangleMeshNormal = true;

        public bool UseTriangleMeshNormal { get => useTriangleMeshNormal; set => useTriangleMeshNormal = value; }

        public bool UseTerrainNormal { get => useTerrainNormal; set => useTerrainNormal = value; }

        public virtual void Detect(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            Debug.Assert(entity1 != entity2, "CollisionSystem reports selfcollision. Something is wrong.");

            var rigidBody1 = entity1 as RigidBody;
            var rigidBody2 = entity2 as RigidBody;

            if (rigidBody1 != null)
            {
                if (rigidBody2 != null)
                {
                    DetectRigidRigid(rigidBody1, rigidBody2);
                }
                else
                {
                    var softBody2 = entity2 as SoftBody;
                    if (softBody2 != null)
                    {
                        DetectSoftRigid(rigidBody1, softBody2);
                    }
                }
            }
            else
            {
                var softBody1 = entity1 as SoftBody;

                if (rigidBody2 != null)
                {
                    if (softBody1 != null)
                    {
                        DetectSoftRigid(rigidBody2, softBody1);
                    }
                }
                else
                {
                    var softBody2 = entity2 as SoftBody;
                    if (softBody1 != null && softBody2 != null)
                    {
                        DetectSoftSoft(softBody1, softBody2);
                    }
                }
            }
        }

        private readonly ResourcePool<List<int>> potentialTriangleLists = new ResourcePool<List<int>>();

        private void DetectSoftSoft(SoftBody body1, SoftBody body2)
        {
            var my = potentialTriangleLists.GetNew();
            var other = potentialTriangleLists.GetNew();

            body1.dynamicTree.Query(other, my, body2.dynamicTree);

            for (int i = 0; i < other.Count; i++)
            {
                var myTriangle = body1.dynamicTree.GetUserData(my[i]);
                var otherTriangle = body2.dynamicTree.GetUserData(other[i]);

                bool result;

                result = XenoCollide.Detect(myTriangle, otherTriangle, ref JMatrix.InternalIdentity, ref JMatrix.InternalIdentity,
                    ref JVector.InternalZero, ref JVector.InternalZero, out var point, out var normal, out float penetration);

                if (result)
                {
                    int minIndexMy = FindNearestTrianglePoint(body1, my[i], ref point);
                    int minIndexOther = FindNearestTrianglePoint(body2, other[i], ref point);

                    RaiseCollisionDetected(body1.VertexBodies[minIndexMy],
                        body2.VertexBodies[minIndexOther], ref point, ref point, ref normal, penetration);
                }
            }

            my.Clear(); other.Clear();
            potentialTriangleLists.GiveBack(my);
            potentialTriangleLists.GiveBack(other);
        }

        private void DetectRigidRigid(RigidBody body1, RigidBody body2)
        {
            bool b1IsMulti = (body1.Shape is Multishape);
            bool b2IsMulti = (body2.Shape is Multishape);

            bool speculative = EnableSpeculativeContacts ||
                (body1.EnableSpeculativeContacts || body2.EnableSpeculativeContacts);

            JVector point, normal;
            float penetration;

            if (!b1IsMulti && !b2IsMulti)
            {
                if (XenoCollide.Detect(body1.Shape, body2.Shape, ref body1.orientation,
                    ref body2.orientation, ref body1.position, ref body2.position,
                    out point, out normal, out penetration))
                {
                    FindSupportPoints(body1, body2, body1.Shape, body2.Shape, ref point, ref normal, out var point1, out var point2);
                    RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                }
                else if (speculative)
                {
                    if (GJKCollide.ClosestPoints(body1.Shape, body2.Shape, ref body1.orientation, ref body2.orientation,
                        ref body1.position, ref body2.position, out var hit1, out var hit2, out normal))
                    {
                        var delta = hit2 - hit1;

                        if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                        {
                            penetration = delta * normal;

                            if (penetration < 0.0f)
                            {
                                RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                            }
                        }
                    }
                }
            }
            else if (b1IsMulti && b2IsMulti)
            {
                var ms1 = (body1.Shape as Multishape);
                var ms2 = (body2.Shape as Multishape);

                ms1 = ms1.RequestWorkingClone();
                ms2 = ms2.RequestWorkingClone();

                var transformedBoundingBox = body2.boundingBox;
                transformedBoundingBox.InverseTransform(ref body1.position, ref body1.orientation);

                int ms1Length = ms1.Prepare(ref transformedBoundingBox);

                transformedBoundingBox = body1.boundingBox;
                transformedBoundingBox.InverseTransform(ref body2.position, ref body2.orientation);

                int ms2Length = ms2.Prepare(ref transformedBoundingBox);

                if (ms1Length == 0 || ms2Length == 0)
                {
                    ms1.ReturnWorkingClone();
                    ms2.ReturnWorkingClone();
                    return;
                }

                for (int i = 0; i < ms1Length; i++)
                {
                    ms1.SetCurrentShape(i);

                    for (int e = 0; e < ms2Length; e++)
                    {
                        ms2.SetCurrentShape(e);

                        if (XenoCollide.Detect(ms1, ms2, ref body1.orientation,
                            ref body2.orientation, ref body1.position, ref body2.position,
                            out point, out normal, out penetration))
                        {
                            FindSupportPoints(body1, body2, ms1, ms2, ref point, ref normal, out var point1, out var point2);
                            RaiseCollisionDetected(body1, body2, ref point1, ref point2, ref normal, penetration);
                        }
                        else if (speculative)
                        {
                            if (GJKCollide.ClosestPoints(ms1, ms2, ref body1.orientation, ref body2.orientation,
                                ref body1.position, ref body2.position, out var hit1, out var hit2, out normal))
                            {
                                var delta = hit2 - hit1;

                                if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                                {
                                    penetration = delta * normal;

                                    if (penetration < 0.0f)
                                    {
                                        RaiseCollisionDetected(body1, body2, ref hit1, ref hit2, ref normal, penetration);
                                    }
                                }
                            }

                        }
                    }
                }

                ms1.ReturnWorkingClone();
                ms2.ReturnWorkingClone();
            }
            else
            {
                RigidBody b1, b2;

                if (body2.Shape is Multishape) { b1 = body2; b2 = body1; }
                else { b2 = body2; b1 = body1; }

                var ms = (b1.Shape as Multishape);

                ms = ms.RequestWorkingClone();

                var transformedBoundingBox = b2.boundingBox;
                transformedBoundingBox.InverseTransform(ref b1.position, ref b1.orientation);

                int msLength = ms.Prepare(ref transformedBoundingBox);

                if (msLength == 0)
                {
                    ms.ReturnWorkingClone();
                    return;
                }

                for (int i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (XenoCollide.Detect(ms, b2.Shape, ref b1.orientation,
                        ref b2.orientation, ref b1.position, ref b2.position,
                        out point, out normal, out penetration))
                    {
                        FindSupportPoints(b1, b2, ms, b2.Shape, ref point, ref normal, out var point1, out var point2);

                        if (useTerrainNormal && ms is TerrainShape)
                        {
                            (ms as TerrainShape).CollisionNormal(out normal);
                            JVector.Transform(ref normal, ref b1.orientation, out normal);
                        }
                        else if (useTriangleMeshNormal && ms is TriangleMeshShape)
                        {
                            (ms as TriangleMeshShape).CollisionNormal(out normal);
                            JVector.Transform(ref normal, ref b1.orientation, out normal);
                        }

                        RaiseCollisionDetected(b1, b2, ref point1, ref point2, ref normal, penetration);
                    }
                    else if (speculative)
                    {
                        if (GJKCollide.ClosestPoints(ms, b2.Shape, ref b1.orientation, ref b2.orientation,
                            ref b1.position, ref b2.position, out var hit1, out var hit2, out normal))
                        {
                            var delta = hit2 - hit1;

                            if (delta.LengthSquared() < (body1.sweptDirection - body2.sweptDirection).LengthSquared())
                            {
                                penetration = delta * normal;

                                if (penetration < 0.0f)
                                {
                                    RaiseCollisionDetected(b1, b2, ref hit1, ref hit2, ref normal, penetration);
                                }
                            }
                        }
                    }
                }

                ms.ReturnWorkingClone();
            }
        }

        private void DetectSoftRigid(RigidBody rigidBody, SoftBody softBody)
        {
            if (rigidBody.Shape is Multishape)
            {
                var ms = (rigidBody.Shape as Multishape);
                ms = ms.RequestWorkingClone();

                var transformedBoundingBox = softBody.BoundingBox;
                transformedBoundingBox.InverseTransform(ref rigidBody.position, ref rigidBody.orientation);

                int msLength = ms.Prepare(ref transformedBoundingBox);

                var detected = potentialTriangleLists.GetNew();
                softBody.dynamicTree.Query(detected, ref rigidBody.boundingBox);

                foreach (int i in detected)
                {
                    var t = softBody.dynamicTree.GetUserData(i);

                    bool result;

                    for (int e = 0; e < msLength; e++)
                    {
                        ms.SetCurrentShape(e);

                        result = XenoCollide.Detect(ms, t, ref rigidBody.orientation, ref JMatrix.InternalIdentity,
                            ref rigidBody.position, ref JVector.InternalZero, out var point, out var normal, out float penetration);

                        if (result)
                        {
                            int minIndex = FindNearestTrianglePoint(softBody, i, ref point);

                            RaiseCollisionDetected(rigidBody,
                                softBody.VertexBodies[minIndex], ref point, ref point, ref normal, penetration);
                        }
                    }
                }

                detected.Clear(); potentialTriangleLists.GiveBack(detected);
                ms.ReturnWorkingClone();
            }
            else
            {
                var detected = potentialTriangleLists.GetNew();
                softBody.dynamicTree.Query(detected, ref rigidBody.boundingBox);

                foreach (int i in detected)
                {
                    var t = softBody.dynamicTree.GetUserData(i);

                    bool result;

                    result = XenoCollide.Detect(rigidBody.Shape, t, ref rigidBody.orientation, ref JMatrix.InternalIdentity,
                        ref rigidBody.position, ref JVector.InternalZero, out var point, out var normal, out float penetration);

                    if (result)
                    {
                        int minIndex = FindNearestTrianglePoint(softBody, i, ref point);

                        RaiseCollisionDetected(rigidBody,
                            softBody.VertexBodies[minIndex], ref point, ref point, ref normal, penetration);
                    }
                }

                detected.Clear();
                potentialTriangleLists.GiveBack(detected);
            }
        }

        public static int FindNearestTrianglePoint(SoftBody sb, int id, ref JVector point)
        {
            var triangle = sb.dynamicTree.GetUserData(id);
            JVector p;

            p = sb.VertexBodies[triangle.indices.I0].position;
            JVector.Subtract(ref p, ref point, out p);

            float length0 = p.LengthSquared();

            p = sb.VertexBodies[triangle.indices.I1].position;
            JVector.Subtract(ref p, ref point, out p);

            float length1 = p.LengthSquared();

            p = sb.VertexBodies[triangle.indices.I2].position;
            JVector.Subtract(ref p, ref point, out p);

            float length2 = p.LengthSquared();

            if (length0 < length1)
            {
                if (length0 < length2)
                {
                    return triangle.indices.I0;
                }
                else
                {
                    return triangle.indices.I2;
                }
            }
            else
            {
                if (length1 < length2)
                {
                    return triangle.indices.I1;
                }
                else
                {
                    return triangle.indices.I2;
                }
            }
        }

        private void FindSupportPoints(RigidBody body1, RigidBody body2,
            Shape shape1, Shape shape2, ref JVector point, ref JVector normal,
            out JVector point1, out JVector point2)
        {
            JVector.Negate(ref normal, out var mn);

            SupportMapping(body1, shape1, ref mn, out var sA);
            SupportMapping(body2, shape2, ref normal, out var sB);

            JVector.Subtract(ref sA, ref point, out sA);
            JVector.Subtract(ref sB, ref point, out sB);

            float dot1 = JVector.Dot(ref sA, ref normal);
            float dot2 = JVector.Dot(ref sB, ref normal);

            JVector.Multiply(ref normal, dot1, out sA);
            JVector.Multiply(ref normal, dot2, out sB);

            JVector.Add(ref point, ref sA, out point1);
            JVector.Add(ref point, ref sB, out point2);
        }

        private void SupportMapping(RigidBody body, Shape workingShape, ref JVector direction, out JVector result)
        {
            JVector.Transform(ref direction, ref body.invOrientation, out result);
            workingShape.SupportMapping(ref result, out result);
            JVector.Transform(ref result, ref body.orientation, out result);
            JVector.Add(ref result, ref body.position, out result);
        }

        public abstract bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction);

        public abstract bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction);

        public bool CheckBothStaticOrInactive(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            return (entity1.IsStaticOrInactive && entity2.IsStaticOrInactive);
        }

        public bool CheckBoundingBoxes(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            var box1 = entity1.BoundingBox;
            var box2 = entity2.BoundingBox;

            return ((((box1.Max.Z >= box2.Min.Z) && (box1.Min.Z <= box2.Max.Z)) &&
                ((box1.Max.Y >= box2.Min.Y) && (box1.Min.Y <= box2.Max.Y))) &&
                ((box1.Max.X >= box2.Min.X) && (box1.Min.X <= box2.Max.X)));
        }

        public bool RaisePassedBroadphase(IBroadphaseEntity entity1, IBroadphaseEntity entity2)
        {
            if (PassedBroadphase != null)
            {
                return PassedBroadphase(entity1, entity2);
            }

            return true;
        }

        protected void RaiseCollisionDetected(RigidBody body1, RigidBody body2,
                                    ref JVector point1, ref JVector point2,
                                    ref JVector normal, float penetration)
        {
            CollisionDetected?.Invoke(body1, body2, point1, point2, normal, penetration);
        }

        public abstract void Detect(bool multiThreaded);
    }
}
