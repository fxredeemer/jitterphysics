using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision.Shapes
{
    public delegate void ShapeUpdatedHandler();

    public abstract class Shape : ISupportMappable
    {
        internal JMatrix inertia = JMatrix.Identity;
        internal float mass = 1.0f;

        internal JBBox boundingBox = JBBox.LargeBox;
        internal JVector geomCen = JVector.Zero;

        public event ShapeUpdatedHandler ShapeUpdated;

        public JMatrix Inertia { get => inertia; protected set => inertia = value; }

        public float Mass { get => mass; protected set => mass = value; }

        protected void RaiseShapeUpdated()
        {
            ShapeUpdated?.Invoke();
        }

        public JBBox BoundingBox => boundingBox;

        public object Tag { get; set; }

        private struct ClipTriangle
        {
            public JVector n1;
            public JVector n2;
            public JVector n3;
            public int generation;
        };

        public virtual void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
            float distanceThreshold = 0.0f;

            if (generationThreshold < 0)
            {
                generationThreshold = 4;
            }

            var activeTriList = new Stack<ClipTriangle>();

            var v = new JVector[]
            {
                new JVector( -1,  0,  0 ),
                new JVector(  1,  0,  0 ),
                new JVector(  0, -1,  0 ),
                new JVector(  0,  1,  0 ),
                new JVector(  0,  0, -1 ),
                new JVector(  0,  0,  1 ),
            };

            int[,] kTriangleVerts = new int[8, 3]
            {
                { 5, 1, 3 },
                { 4, 3, 1 },
                { 3, 4, 0 },
                { 0, 5, 3 },
                { 5, 2, 1 },
                { 4, 1, 2 },
                { 2, 0, 4 },
                { 0, 2, 5 }
            };

            for (int i = 0; i < 8; i++)
            {
                var tri = new ClipTriangle
                {
                    n1 = v[kTriangleVerts[i, 0]],
                    n2 = v[kTriangleVerts[i, 1]],
                    n3 = v[kTriangleVerts[i, 2]],
                    generation = 0
                };
                activeTriList.Push(tri);
            }

            while (activeTriList.Count > 0)
            {
                var tri = activeTriList.Pop();

                SupportMapping(ref tri.n1, out var p1);
                SupportMapping(ref tri.n2, out var p2);
                SupportMapping(ref tri.n3, out var p3);

                float d1 = (p2 - p1).LengthSquared();
                float d2 = (p3 - p2).LengthSquared();
                float d3 = (p1 - p3).LengthSquared();

                if (Math.Max(Math.Max(d1, d2), d3) > distanceThreshold && tri.generation < generationThreshold)
                {
                    var tri1 = new ClipTriangle();
                    var tri2 = new ClipTriangle();
                    var tri3 = new ClipTriangle();
                    var tri4 = new ClipTriangle();

                    tri1.generation = tri.generation + 1;
                    tri2.generation = tri.generation + 1;
                    tri3.generation = tri.generation + 1;
                    tri4.generation = tri.generation + 1;

                    tri1.n1 = tri.n1;
                    tri2.n2 = tri.n2;
                    tri3.n3 = tri.n3;

                    var n = 0.5f * (tri.n1 + tri.n2);
                    n.Normalize();

                    tri1.n2 = n;
                    tri2.n1 = n;
                    tri4.n3 = n;

                    n = 0.5f * (tri.n2 + tri.n3);
                    n.Normalize();

                    tri2.n3 = n;
                    tri3.n2 = n;
                    tri4.n1 = n;

                    n = 0.5f * (tri.n3 + tri.n1);
                    n.Normalize();

                    tri1.n3 = n;
                    tri3.n1 = n;
                    tri4.n2 = n;

                    activeTriList.Push(tri1);
                    activeTriList.Push(tri2);
                    activeTriList.Push(tri3);
                    activeTriList.Push(tri4);
                }
                else
                {
                    if (((p3 - p1) % (p2 - p1)).LengthSquared() > JMath.Epsilon)
                    {
                        triangleList.Add(p1);
                        triangleList.Add(p2);
                        triangleList.Add(p3);
                    }
                }
            }
        }

        public virtual void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            var vec = JVector.Zero;

            vec.Set(orientation.M11, orientation.M21, orientation.M31);
            SupportMapping(ref vec, out vec);
            box.Max.X = (orientation.M11 * vec.X) + (orientation.M21 * vec.Y) + (orientation.M31 * vec.Z);

            vec.Set(orientation.M12, orientation.M22, orientation.M32);
            SupportMapping(ref vec, out vec);
            box.Max.Y = (orientation.M12 * vec.X) + (orientation.M22 * vec.Y) + (orientation.M32 * vec.Z);

            vec.Set(orientation.M13, orientation.M23, orientation.M33);
            SupportMapping(ref vec, out vec);
            box.Max.Z = (orientation.M13 * vec.X) + (orientation.M23 * vec.Y) + (orientation.M33 * vec.Z);

            vec.Set(-orientation.M11, -orientation.M21, -orientation.M31);
            SupportMapping(ref vec, out vec);
            box.Min.X = (orientation.M11 * vec.X) + (orientation.M21 * vec.Y) + (orientation.M31 * vec.Z);

            vec.Set(-orientation.M12, -orientation.M22, -orientation.M32);
            SupportMapping(ref vec, out vec);
            box.Min.Y = (orientation.M12 * vec.X) + (orientation.M22 * vec.Y) + (orientation.M32 * vec.Z);

            vec.Set(-orientation.M13, -orientation.M23, -orientation.M33);
            SupportMapping(ref vec, out vec);
            box.Min.Z = (orientation.M13 * vec.X) + (orientation.M23 * vec.Y) + (orientation.M33 * vec.Z);
        }

        public virtual void UpdateShape()
        {
            GetBoundingBox(ref JMatrix.InternalIdentity, out boundingBox);

            CalculateMassInertia();
            RaiseShapeUpdated();
        }

        public static float CalculateMassInertia(Shape shape, out JVector centerOfMass,
            out JMatrix inertia)
        {
            float mass = 0.0f;
            centerOfMass = JVector.Zero; inertia = JMatrix.Zero;

            if (shape is Multishape)
            {
                throw new ArgumentException("Can't calculate inertia of multishapes.", nameof(shape));
            }

            var hullTriangles = new List<JVector>();
            shape.MakeHull(ref hullTriangles, 3);

            float a = 1.0f / 60.0f, b = 1.0f / 120.0f;
            var C = new JMatrix(a, b, b, b, a, b, b, b, a);

            for (int i = 0; i < hullTriangles.Count; i += 3)
            {
                var column0 = hullTriangles[i + 0];
                var column1 = hullTriangles[i + 1];
                var column2 = hullTriangles[i + 2];

                var A = new JMatrix(column0.X, column1.X, column2.X,
                    column0.Y, column1.Y, column2.Y,
                    column0.Z, column1.Z, column2.Z);

                float detA = A.Determinant();

                var tetrahedronInertia = JMatrix.Multiply(A * C * JMatrix.Transpose(A), detA);

                var tetrahedronCOM = (1.0f / 4.0f) * (hullTriangles[i + 0] + hullTriangles[i + 1] + hullTriangles[i + 2]);
                float tetrahedronMass = (1.0f / 6.0f) * detA;

                inertia += tetrahedronInertia;
                centerOfMass += tetrahedronMass * tetrahedronCOM;
                mass += tetrahedronMass;
            }

            inertia = JMatrix.Multiply(JMatrix.Identity, inertia.Trace()) - inertia;
            centerOfMass = centerOfMass * (1.0f / mass);

            float x = centerOfMass.X;
            float y = centerOfMass.Y;
            float z = centerOfMass.Z;

            var t = new JMatrix(
    -mass * ((y * y) + (z * z)), mass * x * y, mass * x * z,
    mass * y * x, -mass * ((z * z) + (x * x)), mass * y * z,
    mass * z * x, mass * z * y, -mass * ((x * x) + (y * y)));

            JMatrix.Add(ref inertia, ref t, out inertia);

            return mass;
        }

        public virtual void CalculateMassInertia()
        {
            mass = Shape.CalculateMassInertia(this, out geomCen, out inertia);
        }

        public abstract void SupportMapping(ref JVector direction, out JVector result);

        public void SupportCenter(out JVector geomCenter)
        {
            geomCenter = geomCen;
        }
    }
}
