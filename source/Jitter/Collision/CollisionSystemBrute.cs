﻿using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision
{
    public class CollisionSystemBrute : CollisionSystem
    {
        private readonly List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();
        private readonly Action<object> detectCallback;

        public CollisionSystemBrute()
        {
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

        public override void Detect(bool multiThreaded)
        {
            int count = bodyList.Count;

            if (multiThreaded)
            {
                for (int i = 0; i < count; i++)
                {
                    for (int e = i + 1; e < count; e++)
                    {
                        if (!CheckBothStaticOrInactive(bodyList[i], bodyList[e])
                            && CheckBoundingBoxes(bodyList[i], bodyList[e])
                            && RaisePassedBroadphase(bodyList[i], bodyList[e]))
                        {
                            var pair = BroadphasePair.Pool.GetNew();

                            if (swapOrder)
                            {
                                pair.Entity1 = bodyList[i];
                                pair.Entity2 = bodyList[e];
                            }
                            else
                            {
                                pair.Entity2 = bodyList[e];
                                pair.Entity1 = bodyList[i];
                            }
                            swapOrder = !swapOrder;

                            threadManager.AddTask(detectCallback, pair);
                        }
                    }
                }

                threadManager.Execute();
            }
            else
            {
                for (int i = 0; i < count; i++)
                {
                    for (int e = i + 1; e < count; e++)
                    {
                        if (!CheckBothStaticOrInactive(bodyList[i], bodyList[e])
                            && CheckBoundingBoxes(bodyList[i], bodyList[e])
                            && RaisePassedBroadphase(bodyList[i], bodyList[e]))
                        {
                            if (swapOrder)
                            {
                                Detect(bodyList[i], bodyList[e]);
                            }
                            else
                            {
                                Detect(bodyList[e], bodyList[i]);
                            }

                            swapOrder = !swapOrder;
                        }
                    }
                }
            }
        }

        private bool swapOrder;

        private void DetectCallback(object obj)
        {
            var pair = obj as BroadphasePair;
            base.Detect(pair.Entity1, pair.Entity2);
            BroadphasePair.Pool.GiveBack(pair);
        }

        public override bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction)
        {
            body = null;
            normal = JVector.Zero;
            fraction = float.MaxValue;

            JVector tempNormal;
            float tempFraction;
            bool result = false;

            foreach (var e in bodyList)
            {
                if (e is SoftBody softBody)
                {
                    foreach (RigidBody rigidBody in softBody.VertexBodies)
                    {
                        if (Raycast(rigidBody, rayOrigin, rayDirection, out tempNormal, out tempFraction)
                            && tempFraction < fraction
                            && (raycast == null || raycast(rigidBody, tempNormal, tempFraction)))
                        {
                            body = rigidBody;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
                else
                {
                    var rigidBody = e as RigidBody;

                    if (Raycast(rigidBody, rayOrigin, rayDirection, out tempNormal, out tempFraction)
                        && tempFraction < fraction
                        && (raycast == null || raycast(rigidBody, tempNormal, tempFraction)))
                    {
                        body = rigidBody;
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

            if (!body.BoundingBox.RayIntersect(ref rayOrigin, ref rayDirection))
            {
                return false;
            }

            if (body.Shape is Multishape multishape)
            {
                multishape = multishape.RequestWorkingClone();

                bool multiShapeCollides = false;

                JVector.Subtract(ref rayOrigin, ref body.position, out var transformedOrigin);
                JVector.Transform(ref transformedOrigin, ref body.invOrientation, out transformedOrigin);
                JVector.Transform(ref rayDirection, ref body.invOrientation, out var transformedDirection);

                int msLength = multishape.Prepare(ref transformedOrigin, ref transformedDirection);

                for (int i = 0; i < msLength; i++)
                {
                    multishape.SetCurrentShape(i);

                    if (GJKCollide.Raycast(
                        multishape,
                        ref body.orientation,
                        ref body.invOrientation,
                        ref body.position,
                        ref rayOrigin,
                        ref rayDirection,
                        out float tempFraction,
                        out var tempNormal) && tempFraction < fraction)
                    {
                        if (useTerrainNormal && multishape is TerrainShape terrainShape)
                        {
                            terrainShape.CollisionNormal(out tempNormal);
                            JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                            tempNormal.Negate();
                        }
                        else if (useTriangleMeshNormal && multishape is TriangleMeshShape triangleMeshShape)
                        {
                            triangleMeshShape.CollisionNormal(out tempNormal);
                            JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                            tempNormal.Negate();
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
                    ref body.invOrientation,
                    ref body.position,
                    ref rayOrigin,
                    ref rayDirection,
                    out fraction,
                    out normal);
            }
        }
    }
}
