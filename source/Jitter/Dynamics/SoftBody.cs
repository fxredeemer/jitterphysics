using Jitter.Collision;
using Jitter.Collision.Shapes;
using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;

namespace Jitter.Dynamics
{
    public partial class SoftBody : IBroadphaseEntity
    {
        [Flags]
        public enum SpringType
        {
            EdgeSpring = 0x02, ShearSpring = 0x04, BendSpring = 0x08
        }

        public class Spring : Constraint
        {
            public enum DistanceBehavior
            {
                LimitDistance,
                LimitMaximumDistance,
                LimitMinimumDistance,
            }

            public SpringType SpringType { get; set; }

            public Spring(RigidBody body1, RigidBody body2) : base(body1, body2)
            {
                Distance = (body1.position - body2.position).Length();
            }

            public float AppliedImpulse { get; private set; }

            public float Distance { get; set; }

            public DistanceBehavior Behavior { get; set; }

            public float Softness { get; set; } = 0.01f;

            public float BiasFactor { get; set; } = 0.1f;

            private float effectiveMass;
            private float bias;
            private float softnessOverDt;
            private readonly JVector[] jacobian = new JVector[2];

            private bool skipConstraint;

            public override void PrepareForIteration(float timestep)
            {
                JVector.Subtract(ref body2.position, ref body1.position, out var dp);

                float deltaLength = dp.Length() - Distance;

                if (Behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= 0.0f)
                {
                    skipConstraint = true;
                }
                else if (Behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= 0.0f)
                {
                    skipConstraint = true;
                }
                else
                {
                    skipConstraint = false;

                    var n = dp;
                    if (n.LengthSquared() != 0.0f)
                    {
                        n.Normalize();
                    }

                    jacobian[0] = -1.0f * n;
                    jacobian[1] = 1.0f * n;

                    effectiveMass = body1.inverseMass + body2.inverseMass;

                    softnessOverDt = Softness / timestep;
                    effectiveMass += softnessOverDt;

                    effectiveMass = 1.0f / effectiveMass;

                    bias = deltaLength * BiasFactor * (1.0f / timestep);

                    if (!body1.isStatic)
                    {
                        body1.linearVelocity += body1.inverseMass * AppliedImpulse * jacobian[0];
                    }

                    if (!body2.isStatic)
                    {
                        body2.linearVelocity += body2.inverseMass * AppliedImpulse * jacobian[1];
                    }
                }
            }

            public override void Iterate()
            {
                if (skipConstraint)
                {
                    return;
                }

                float jv = JVector.Dot(ref body1.linearVelocity, ref jacobian[0]);
                jv += JVector.Dot(ref body2.linearVelocity, ref jacobian[1]);

                float softnessScalar = AppliedImpulse * softnessOverDt;

                float lambda = -effectiveMass * (jv + bias + softnessScalar);

                if (Behavior == DistanceBehavior.LimitMinimumDistance)
                {
                    float previousAccumulatedImpulse = AppliedImpulse;
                    AppliedImpulse = JMath.Max(AppliedImpulse + lambda, 0);
                    lambda = AppliedImpulse - previousAccumulatedImpulse;
                }
                else if (Behavior == DistanceBehavior.LimitMaximumDistance)
                {
                    float previousAccumulatedImpulse = AppliedImpulse;
                    AppliedImpulse = JMath.Min(AppliedImpulse + lambda, 0);
                    lambda = AppliedImpulse - previousAccumulatedImpulse;
                }
                else
                {
                    AppliedImpulse += lambda;
                }

                JVector temp;

                if (!body1.isStatic)
                {
                    JVector.Multiply(ref jacobian[0], lambda * body1.inverseMass, out temp);
                    JVector.Add(ref temp, ref body1.linearVelocity, out body1.linearVelocity);
                }

                if (!body2.isStatic)
                {
                    JVector.Multiply(ref jacobian[1], lambda * body2.inverseMass, out temp);
                    JVector.Add(ref temp, ref body2.linearVelocity, out body2.linearVelocity);
                }
            }

            public override void DebugDraw(IDebugDrawer drawer)
            {
                drawer.DrawLine(body1.position, body2.position);
            }
        }

        public class MassPoint : RigidBody
        {
            public SoftBody SoftBody { get; }

            public MassPoint(Shape shape, SoftBody owner, Material material)
                : base(shape, material, true)
            {
                SoftBody = owner;
            }
        }

        public class Triangle : ISupportMappable
        {
            public SoftBody Owner { get; }

            internal JBBox boundingBox;
            internal int dynamicTreeID;
            internal TriangleVertexIndices indices;

            public JBBox BoundingBox => boundingBox;
            public int DynamicTreeID => dynamicTreeID;

            public TriangleVertexIndices Indices => indices;

            public MassPoint VertexBody1 => Owner.points[indices.I0];
            public MassPoint VertexBody2 => Owner.points[indices.I1];
            public MassPoint VertexBody3 => Owner.points[indices.I2];

            public Triangle(SoftBody owner)
            {
                Owner = owner;
            }

            public void GetNormal(out JVector normal)
            {
                JVector.Subtract(ref Owner.points[indices.I1].position, ref Owner.points[indices.I0].position, out var sum);
                JVector.Subtract(ref Owner.points[indices.I2].position, ref Owner.points[indices.I0].position, out normal);
                JVector.Cross(ref sum, ref normal, out normal);
            }

            public void UpdateBoundingBox()
            {
                boundingBox = JBBox.SmallBox;
                boundingBox.AddPoint(ref Owner.points[indices.I0].position);
                boundingBox.AddPoint(ref Owner.points[indices.I1].position);
                boundingBox.AddPoint(ref Owner.points[indices.I2].position);

                boundingBox.Min -= new JVector(Owner.triangleExpansion);
                boundingBox.Max += new JVector(Owner.triangleExpansion);
            }

            public float CalculateArea()
            {
                return ((Owner.points[indices.I1].position - Owner.points[indices.I0].position)
                    % (Owner.points[indices.I2].position - Owner.points[indices.I0].position)).Length();
            }

            public void SupportMapping(ref JVector direction, out JVector result)
            {
                float min = JVector.Dot(ref Owner.points[indices.I0].position, ref direction);
                float dot = JVector.Dot(ref Owner.points[indices.I1].position, ref direction);

                var minVertex = Owner.points[indices.I0].position;

                if (dot > min)
                {
                    min = dot;
                    minVertex = Owner.points[indices.I1].position;
                }
                dot = JVector.Dot(ref Owner.points[indices.I2].position, ref direction);
                if (dot > min)
                {
                    min = dot;
                    minVertex = Owner.points[indices.I2].position;
                }

                JVector.Normalize(ref direction, out var exp);
                exp *= Owner.triangleExpansion;
                result = minVertex + exp;
            }

            public void SupportCenter(out JVector center)
            {
                center = Owner.points[indices.I0].position;
                JVector.Add(ref center, ref Owner.points[indices.I1].position, out center);
                JVector.Add(ref center, ref Owner.points[indices.I2].position, out center);
                JVector.Multiply(ref center, 1.0f / 3.0f, out center);
            }
        }

        private readonly SphereShape sphere = new SphereShape(0.1f);

        protected List<Spring> springs = new List<Spring>();
        protected List<MassPoint> points = new List<MassPoint>();
        protected List<Triangle> triangles = new List<Triangle>();

        public ReadOnlyCollection<Spring> EdgeSprings { get; private set; }
        public ReadOnlyCollection<MassPoint> VertexBodies { get; private set; }
        public ReadOnlyCollection<Triangle> Triangles { private set; get; }

        protected float triangleExpansion = 0.1f;

        public bool SelfCollision { get; set; }

        public float TriangleExpansion
        {
            get => triangleExpansion;
            set => triangleExpansion = value;
        }

        public float VertexExpansion { get => sphere.Radius; set => sphere.Radius = value; }

        private float mass = 1.0f;

        internal DynamicTree<Triangle> dynamicTree = new DynamicTree<Triangle>();
        public DynamicTree<Triangle> DynamicTree => dynamicTree;

        public Material Material { get; } = new Material();

        private JBBox box = new JBBox();

        private bool active = true;

        public SoftBody()
        {
        }

        public SoftBody(int sizeX, int sizeY, float scale)
        {
            var indices = new List<TriangleVertexIndices>();
            var vertices = new List<JVector>();

            for (int i = 0; i < sizeY; i++)
            {
                for (int e = 0; e < sizeX; e++)
                {
                    vertices.Add(new JVector(i, 0, e) * scale);
                }
            }

            for (int i = 0; i < sizeX - 1; i++)
            {
                for (int e = 0; e < sizeY - 1; e++)
                {
                    var index = new TriangleVertexIndices();
                    {
                        index.I0 = ((e + 0) * sizeX) + i + 0;
                        index.I1 = ((e + 0) * sizeX) + i + 1;
                        index.I2 = ((e + 1) * sizeX) + i + 1;

                        indices.Add(index);

                        index.I0 = ((e + 0) * sizeX) + i + 0;
                        index.I1 = ((e + 1) * sizeX) + i + 1;
                        index.I2 = ((e + 1) * sizeX) + i + 0;

                        indices.Add(index);
                    }
                }
            }

            EdgeSprings = new ReadOnlyCollection<Spring>(springs);
            VertexBodies = new ReadOnlyCollection<MassPoint>(points);
            Triangles = new ReadOnlyCollection<Triangle>(triangles);

            AddPointsAndSprings(indices, vertices);

            for (int i = 0; i < sizeX - 1; i++)
            {
                for (int e = 0; e < sizeY - 1; e++)
                {
                    var spring = new Spring(points[((e + 0) * sizeX) + i + 1], points[((e + 1) * sizeX) + i + 0])
                    {
                        Softness = 0.01f,
                        BiasFactor = 0.1f
                    };
                    springs.Add(spring);
                }
            }

            foreach (var spring in springs)
            {
                var delta = spring.body1.position - spring.body2.position;

                if (delta.Z != 0.0f && delta.X != 0.0f)
                {
                    spring.SpringType = SpringType.ShearSpring;
                }
                else
                {
                    spring.SpringType = SpringType.EdgeSpring;
                }
            }

            for (int i = 0; i < sizeX - 2; i++)
            {
                for (int e = 0; e < sizeY - 2; e++)
                {
                    var spring1 = new Spring(points[((e + 0) * sizeX) + i + 0], points[((e + 0) * sizeX) + i + 2])
                    {
                        Softness = 0.01f,
                        BiasFactor = 0.1f
                    };

                    var spring2 = new Spring(points[((e + 0) * sizeX) + i + 0], points[((e + 2) * sizeX) + i + 0])
                    {
                        Softness = 0.01f,
                        BiasFactor = 0.1f
                    };

                    spring1.SpringType = SpringType.BendSpring;
                    spring2.SpringType = SpringType.BendSpring;

                    springs.Add(spring1);
                    springs.Add(spring2);
                }
            }
        }

        public SoftBody(List<TriangleVertexIndices> indices, List<JVector> vertices)
        {
            EdgeSprings = new ReadOnlyCollection<Spring>(springs);
            VertexBodies = new ReadOnlyCollection<MassPoint>(points);

            AddPointsAndSprings(indices, vertices);
            Triangles = new ReadOnlyCollection<Triangle>(triangles);
        }

        public float Pressure { get; set; }

        private struct Edge
        {
            public int Index1;
            public int Index2;

            public Edge(int index1, int index2)
            {
                Index1 = index1;
                Index2 = index2;
            }

            public override int GetHashCode()
            {
                return Index1.GetHashCode() + Index2.GetHashCode();
            }

            public override bool Equals(object obj)
            {
                var e = (Edge)obj;
                return (e.Index1 == Index1 && e.Index2 == Index2) || (e.Index1 == Index2 && e.Index2 == Index1);
            }
        }

        private void AddPressureForces(float timeStep)
        {
            if (Pressure == 0.0f || Volume == 0.0f)
            {
                return;
            }

            float invVolume = 1.0f / Volume;

            foreach (var t in triangles)
            {
                var v1 = points[t.indices.I0].position;
                var v2 = points[t.indices.I1].position;
                var v3 = points[t.indices.I2].position;

                var cross = (v3 - v1) % (v2 - v1);
                var center = (v1 + v2 + v3) * (1.0f / 3.0f);

                points[t.indices.I0].AddForce(invVolume * cross * Pressure);
                points[t.indices.I1].AddForce(invVolume * cross * Pressure);
                points[t.indices.I2].AddForce(invVolume * cross * Pressure);
            }
        }

        public void Translate(JVector position)
        {
            foreach (var point in points)
            {
                point.Position += position;
            }

            Update(float.Epsilon);
        }

        public void AddForce(JVector force)
        {
            throw new NotImplementedException();
        }

        public void Rotate(JMatrix orientation, JVector center)
        {
            for (int i = 0; i < points.Count; i++)
            {
                points[i].position = JVector.Transform(points[i].position - center, orientation);
            }
        }

        public JVector CalculateCenter()
        {
            throw new NotImplementedException();
        }

        private HashSet<Edge> GetEdges(List<TriangleVertexIndices> indices)
        {
            var edges = new HashSet<Edge>();

            for (int i = 0; i < indices.Count; i++)
            {
                Edge edge = new Edge(indices[i].I0, indices[i].I1);
                if (!edges.Contains(edge))
                {
                    edges.Add(edge);
                }

                edge = new Edge(indices[i].I1, indices[i].I2);
                if (!edges.Contains(edge))
                {
                    edges.Add(edge);
                }

                edge = new Edge(indices[i].I2, indices[i].I0);
                if (!edges.Contains(edge))
                {
                    edges.Add(edge);
                }
            }

            return edges;
        }

        private readonly List<int> queryList = new List<int>();

        public virtual void DoSelfCollision(CollisionDetectedHandler collision)
        {
            if (!SelfCollision)
            {
                return;
            }

            for (int i = 0; i < points.Count; i++)
            {
                queryList.Clear();
                dynamicTree.Query(queryList, ref points[i].boundingBox);

                for (int e = 0; e < queryList.Count; e++)
                {
                    var t = dynamicTree.GetUserData(queryList[e]);

                    if (!(t.VertexBody1 == points[i] || t.VertexBody2 == points[i] || t.VertexBody3 == points[i]))
                    {
                        if (XenoCollide.Detect(points[i].Shape, t, ref points[i].orientation,
                            ref JMatrix.InternalIdentity, ref points[i].position, ref JVector.InternalZero,
                            out var point, out var normal, out float penetration))
                        {
                            int nearest = CollisionSystem.FindNearestTrianglePoint(this, queryList[e], ref point);

                            collision(points[i], points[nearest], point, point, normal, penetration);
                        }
                    }
                }
            }
        }

        private void AddPointsAndSprings(List<TriangleVertexIndices> indices, List<JVector> vertices)
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                var point = new MassPoint(sphere, this, Material)
                {
                    Position = vertices[i],

                    Mass = 0.1f
                };

                points.Add(point);
            }

            for (int i = 0; i < indices.Count; i++)
            {
                var index = indices[i];

                var t = new Triangle(this)
                {
                    indices = index
                };
                triangles.Add(t);

                t.boundingBox = JBBox.SmallBox;
                t.boundingBox.AddPoint(points[t.indices.I0].position);
                t.boundingBox.AddPoint(points[t.indices.I1].position);
                t.boundingBox.AddPoint(points[t.indices.I2].position);

                t.dynamicTreeID = dynamicTree.AddProxy(ref t.boundingBox, t);
            }

            var edges = GetEdges(indices);

            int count = 0;

            foreach (var edge in edges)
            {
                var spring = new Spring(points[edge.Index1], points[edge.Index2])
                {
                    Softness = 0.01f,
                    BiasFactor = 0.1f,
                    SpringType = SpringType.EdgeSpring
                };

                springs.Add(spring);
                count++;
            }
        }

        public void SetSpringValues(float bias, float softness)
        {
            SetSpringValues(SpringType.EdgeSpring | SpringType.ShearSpring | SpringType.BendSpring,
                bias, softness);
        }

        public void SetSpringValues(SpringType type, float bias, float softness)
        {
            for (int i = 0; i < springs.Count; i++)
            {
                if ((springs[i].SpringType & type) != 0)
                {
                    springs[i].Softness = softness; springs[i].BiasFactor = bias;
                }
            }
        }

        public virtual void Update(float timestep)
        {
            active = false;

            foreach (var point in points)
            {
                if (point.isActive && !point.isStatic) { active = true; break; }
            }

            if (!active)
            {
                return;
            }

            box = JBBox.SmallBox;
            Volume = 0.0f;
            mass = 0.0f;

            foreach (var point in points)
            {
                mass += point.Mass;
                box.AddPoint(point.position);
            }

            box.Min -= new JVector(TriangleExpansion);
            box.Max += new JVector(TriangleExpansion);

            foreach (var t in triangles)
            {
                var prevCenter = t.boundingBox.Center;
                t.UpdateBoundingBox();

                var linVel = t.VertexBody1.linearVelocity
                    + t.VertexBody2.linearVelocity
                    + t.VertexBody3.linearVelocity;

                linVel *= 1.0f / 3.0f;

                dynamicTree.MoveProxy(t.dynamicTreeID, ref t.boundingBox, linVel * timestep);

                var v1 = points[t.indices.I0].position;
                var v2 = points[t.indices.I1].position;
                var v3 = points[t.indices.I2].position;

                Volume -= (((v2.Y - v1.Y) * (v3.Z - v1.Z))
                    - ((v2.Z - v1.Z) * (v3.Y - v1.Y))) * (v1.X + v2.X + v3.X);
            }

            Volume /= 6.0f;

            AddPressureForces(timestep);
        }

        public float Mass
        {
            get => mass;
            set
            {
                for (int i = 0; i < points.Count; i++)
                {
                    points[i].Mass = value / points.Count;
                }
            }
        }

        public float Volume { get; private set; } = 1.0f;

        public JBBox BoundingBox => box;

        public int BroadphaseTag { get; set; }

        public object Tag { get; set; }

        public bool IsStaticOrInactive => !active;
    }
}


