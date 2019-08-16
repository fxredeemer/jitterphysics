
#region Using Statements
using System;
using System.Collections.Generic;
using Jitter.LinearMath;
#endregion

namespace Jitter.Collision.Shapes
{
    public class TerrainShape : Multishape
    {
        private float[,] heights;
        private float scaleX, scaleZ;
        private int heightsLength0, heightsLength1;

        private int minX, maxX;
        private int minZ, maxZ;
        private int numX, numZ;

        private JBBox boundings;

        public float SphericalExpansion { get; set; } = 0.05f;

        public TerrainShape(float[,] heights, float scaleX, float scaleZ)
        {
            heightsLength0 = heights.GetLength(0);
            heightsLength1 = heights.GetLength(1);

            #region Bounding Box
            boundings = JBBox.SmallBox;

            for (int i = 0; i < heightsLength0; i++)
            {
                for (int e = 0; e < heightsLength1; e++)
                {
                    if (heights[i, e] > boundings.Max.Y)
                        boundings.Max.Y = heights[i, e];
                    else if (heights[i, e] < boundings.Min.Y)
                        boundings.Min.Y = heights[i, e];
                }
            }

            boundings.Min.X = 0.0f;
            boundings.Min.Z = 0.0f;

            boundings.Max.X = checked(heightsLength0 * scaleX);
            boundings.Max.Z = checked(heightsLength1 * scaleZ);

            #endregion

            this.heights = heights;
            this.scaleX = scaleX;
            this.scaleZ = scaleZ;

            UpdateShape();
        }

        internal TerrainShape() { }

        protected override Multishape CreateWorkingClone()
        {
            var clone = new TerrainShape
            {
                heights = heights,
                scaleX = scaleX,
                scaleZ = scaleZ,
                boundings = boundings,
                heightsLength0 = heightsLength0,
                heightsLength1 = heightsLength1,
                SphericalExpansion = SphericalExpansion
            };
            return clone;
        }

        private readonly JVector[] points = new JVector[3];
        private JVector normal = JVector.Up;

        public override void SetCurrentShape(int index)
        {
            bool leftTriangle = false;

            if (index >= numX * numZ)
            {
                leftTriangle = true;
                index -= numX * numZ;
            }

            int quadIndexX = index % numX;
            int quadIndexZ = index / numX;

            if (leftTriangle)
            {
                points[0].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[2].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }
            else
            {
                points[0].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1].Set((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
                points[2].Set((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }

            var sum = points[0];
            JVector.Add(ref sum, ref points[1], out sum);
            JVector.Add(ref sum, ref points[2], out sum);
            JVector.Multiply(ref sum, 1.0f / 3.0f, out sum);
            geomCen = sum;

            JVector.Subtract(ref points[1], ref points[0], out sum);
            JVector.Subtract(ref points[2], ref points[0], out normal);
            JVector.Cross(ref sum, ref normal, out normal);
        }

        public void CollisionNormal(out JVector normal)
        {
            normal = this.normal;
        }

        public override int Prepare(ref JBBox box)
        {

            if (box.Min.X < boundings.Min.X) minX = 0;
            else
            {
                minX = (int)Math.Floor((float)((box.Min.X - SphericalExpansion) / scaleX));
                minX = Math.Max(minX, 0);
            }

            if (box.Max.X > boundings.Max.X) maxX = heightsLength0 - 1;
            else
            {
                maxX = (int)Math.Ceiling((float)((box.Max.X + SphericalExpansion) / scaleX));
                maxX = Math.Min(maxX, heightsLength0 - 1);
            }

            if (box.Min.Z < boundings.Min.Z) minZ = 0;
            else
            {
                minZ = (int)Math.Floor((float)((box.Min.Z - SphericalExpansion) / scaleZ));
                minZ = Math.Max(minZ, 0);
            }

            if (box.Max.Z > boundings.Max.Z) maxZ = heightsLength1 - 1;
            else
            {
                maxZ = (int)Math.Ceiling((float)((box.Max.Z + SphericalExpansion) / scaleZ));
                maxZ = Math.Min(maxZ, heightsLength1 - 1);
            }

            numX = maxX - minX;
            numZ = maxZ - minZ;

            return numX * numZ * 2;
        }

        public override void CalculateMassInertia()
        {
            inertia = JMatrix.Identity;
            Mass = 1.0f;
        }

        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box = boundings;

            #region Expand Spherical
            box.Min.X -= SphericalExpansion;
            box.Min.Y -= SphericalExpansion;
            box.Min.Z -= SphericalExpansion;
            box.Max.X += SphericalExpansion;
            box.Max.Y += SphericalExpansion;
            box.Max.Z += SphericalExpansion;
            #endregion

            box.Transform(ref orientation);
        }

        public override void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
            for (int index = 0; index < (heightsLength0 - 1) * (heightsLength1 - 1); index++)
            {
                int quadIndexX = index % (heightsLength0 - 1);
                int quadIndexZ = index / (heightsLength0 - 1);

                triangleList.Add(new JVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new JVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new JVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));

                triangleList.Add(new JVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 0], (0 + quadIndexZ + 0) * scaleZ));
                triangleList.Add(new JVector((0 + quadIndexX + 1) * scaleX, heights[0 + quadIndexX + 1, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));
                triangleList.Add(new JVector((0 + quadIndexX + 0) * scaleX, heights[0 + quadIndexX + 0, 0 + quadIndexZ + 1], (0 + quadIndexZ + 1) * scaleZ));
            }
        }

        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            JVector.Normalize(ref direction, out var expandVector);
            JVector.Multiply(ref expandVector, SphericalExpansion, out expandVector);

            int minIndex = 0;
            float min = JVector.Dot(ref points[0], ref direction);
            float dot = JVector.Dot(ref points[1], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = JVector.Dot(ref points[2], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 2;
            }

            JVector.Add(ref points[minIndex], ref expandVector, out result);
        }

        public override int Prepare(ref JVector rayOrigin, ref JVector rayDelta)
        {
            var box = JBBox.SmallBox;

            #region RayEnd + Expand Spherical
            JVector.Normalize(ref rayDelta, out var rayEnd);
            rayEnd = rayOrigin + rayDelta + rayEnd * SphericalExpansion;
            #endregion

            box.AddPoint(ref rayOrigin);
            box.AddPoint(ref rayEnd);

            return Prepare(ref box);
        }
    }
}
