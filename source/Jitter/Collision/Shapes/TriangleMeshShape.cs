using Jitter.LinearMath;
using System.Collections.Generic;

namespace Jitter.Collision.Shapes
{
    public class TriangleMeshShape : Multishape
    {
        private readonly List<int> potentialTriangles = new List<int>();
        private readonly Octree octree;

        public float SphericalExpansion { get; set; } = 0.05f;

        public TriangleMeshShape(Octree octree)
        {
            this.octree = octree;
            UpdateShape();
        }

        internal TriangleMeshShape() { }

        protected override Multishape CreateWorkingClone()
        {
            var clone = new TriangleMeshShape(octree)
            {
                SphericalExpansion = SphericalExpansion
            };
            return clone;
        }

        public override int Prepare(ref JBBox box)
        {
            potentialTriangles.Clear();

            var exp = new JBBox(
                new JVector(
                    box.Min.X - SphericalExpansion,
                    box.Min.Y - SphericalExpansion,
                    box.Min.Z - SphericalExpansion),
                new JVector(
                    box.Max.X + SphericalExpansion,
                    box.Max.Y + SphericalExpansion,
                    box.Max.Z + SphericalExpansion));

            octree.GetTrianglesIntersectingtAABox(potentialTriangles, ref exp);

            return potentialTriangles.Count;
        }

        public override void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
            var large = JBBox.LargeBox;

            var indices = new List<int>();
            octree.GetTrianglesIntersectingtAABox(indices, ref large);

            for (var i = 0; i < indices.Count; i++)
            {
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I0));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I1));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I2));
            }
        }

        public override int Prepare(ref JVector rayOrigin, ref JVector rayDelta)
        {
            potentialTriangles.Clear();

            JVector.Normalize(ref rayDelta, out var expDelta);
            expDelta = rayDelta + (expDelta * SphericalExpansion);

            octree.GetTrianglesIntersectingRay(potentialTriangles, rayOrigin, expDelta);

            return potentialTriangles.Count;
        }

        private readonly JVector[] vecs = new JVector[3];

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            JVector.Normalize(ref direction, out var exp);
            exp *= SphericalExpansion;

            var min = JVector.Dot(ref vecs[0], ref direction);
            var minIndex = 0;
            var dot = JVector.Dot(ref vecs[1], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = JVector.Dot(ref vecs[2], ref direction);
            if (dot > min)
            {
                minIndex = 2;
            }

            result = vecs[minIndex] + exp;
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box = octree.rootNodeBox;

            box = new JBBox(
                new JVector(
                    box.Min.X - SphericalExpansion,
                    box.Min.Y - SphericalExpansion,
                    box.Min.Z - SphericalExpansion),
                new JVector(
                    box.Max.X + SphericalExpansion,
                    box.Max.Y + SphericalExpansion,
                    box.Max.Z + SphericalExpansion));

            box.Transform(ref orientation);
        }

        public bool FlipNormals { get; set; }

        public override void SetCurrentShape(int index)
        {
            vecs[0] = octree.GetVertex(octree.tris[potentialTriangles[index]].I0);
            vecs[1] = octree.GetVertex(octree.tris[potentialTriangles[index]].I1);
            vecs[2] = octree.GetVertex(octree.tris[potentialTriangles[index]].I2);

            var sum = vecs[0];
            JVector.Add(ref sum, ref vecs[1], out sum);
            JVector.Add(ref sum, ref vecs[2], out sum);
            JVector.Multiply(ref sum, 1.0f / 3.0f, out sum);

            geomCen = sum;

            JVector.Subtract(ref vecs[1], ref vecs[0], out sum);
            JVector.Subtract(ref vecs[2], ref vecs[0], out normal);
            JVector.Cross(ref sum, ref normal, out normal);

            if (FlipNormals)
            {
                normal = JVector.Negate(normal);
            }
        }

        private JVector normal = JVector.Up;

        public void CollisionNormal(out JVector normal)
        {
            normal = this.normal;
        }
    }
}
