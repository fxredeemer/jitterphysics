using Jitter.LinearMath;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Jitter.Collision
{
    public class Octree
    {
        [Flags]
        private enum EChild
        {
            XP = 0x1,
            YP = 0x2,
            ZP = 0x4,
            PPP = XP | YP | ZP,
            PPM = XP | YP,
            PMP = XP | ZP,
            PMM = XP,
            MPP = YP | ZP,
            MPM = YP,
            MMP = ZP,
            MMM = 0x0,
        }

        private struct Node
        {
            public ushort[] nodeIndices;
            public int[] triIndices;
            public JBBox box;
        }

        private class BuildNode
        {
            public int childType;
            public List<int> nodeIndices = new List<int>();
            public List<int> triIndices = new List<int>();
            public JBBox box;
        }

        private JVector[] positions;
        private JBBox[] triBoxes;
        private Node[] nodes;
        internal TriangleVertexIndices[] tris;
        internal JBBox rootNodeBox;

        public JBBox RootNodeBox => rootNodeBox;

        public void Clear()
        {
            positions = null;
            triBoxes = null;
            tris = null;
            nodes = null;
            nodeStackPool.ResetResourcePool();
        }

        public void SetTriangles(List<JVector> positions, List<TriangleVertexIndices> tris)
        {
            this.positions = new JVector[positions.Count];
            positions.CopyTo(this.positions);

            this.tris = new TriangleVertexIndices[tris.Count];
            tris.CopyTo(this.tris);
        }

        public void BuildOctree()
        {
            triBoxes = new JBBox[tris.Length];

            rootNodeBox = new JBBox(new JVector(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity),
                               new JVector(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity));

            for (var i = 0; i < tris.Length; i++)
            {
                var i0 = positions[tris[i].I0];
                var i1 = positions[tris[i].I1];
                var i2 = positions[tris[i].I2];

                JVector.Min(i1, i2, out var min);
                JVector.Min(i0, min, out min);

                JVector.Max(i1, i2, out var max);
                JVector.Max(i0, max, out max);

                triBoxes[i] = new JBBox(min, max);

                JVector.Min(rootNodeBox.Min, min, out var rootNodeBoxMin);
                JVector.Max(rootNodeBox.Max, max, out var rootNodeBoxMax);

                rootNodeBox = new JBBox(rootNodeBoxMin, rootNodeBoxMax);
            }

            var buildNodes = new List<BuildNode>
            {
                new BuildNode()
            };
            buildNodes[0].box = rootNodeBox;

            var children = new JBBox[8];
            for (var triNum = 0; triNum < tris.Length; triNum++)
            {
                var nodeIndex = 0;
                var box = rootNodeBox;

                while (box.Contains(triBoxes[triNum]) == JBBox.ContainmentType.Contains)
                {
                    var childCon = -1;
                    for (var i = 0; i < 8; ++i)
                    {
                        CreateAABox(ref box, (EChild)i, out children[i]);
                        if (children[i].Contains(triBoxes[triNum]) == JBBox.ContainmentType.Contains)
                        {
                            childCon = i;
                            break;
                        }
                    }

                    if (childCon == -1)
                    {
                        buildNodes[nodeIndex].triIndices.Add(triNum);
                        break;
                    }
                    else
                    {
                        var childIndex = -1;
                        for (var index = 0; index < buildNodes[nodeIndex].nodeIndices.Count; ++index)
                        {
                            if (buildNodes[buildNodes[nodeIndex].nodeIndices[index]].childType == childCon)
                            {
                                childIndex = index;
                                break;
                            }
                        }
                        if (childIndex == -1)
                        {
                            var parentNode = buildNodes[nodeIndex];
                            var newNode = new BuildNode
                            {
                                childType = childCon,
                                box = children[childCon]
                            };
                            buildNodes.Add(newNode);

                            nodeIndex = buildNodes.Count - 1;
                            box = children[childCon];
                            parentNode.nodeIndices.Add(nodeIndex);
                        }
                        else
                        {
                            nodeIndex = buildNodes[nodeIndex].nodeIndices[childIndex];
                            box = children[childCon];
                        }
                    }
                }
            }

            nodes = new Node[buildNodes.Count];
            nodeStackPool = new ArrayResourcePool<ushort>(buildNodes.Count);
            for (var i = 0; i < nodes.Length; i++)
            {
                nodes[i].nodeIndices = new ushort[buildNodes[i].nodeIndices.Count];
                for (var index = 0; index < nodes[i].nodeIndices.Length; ++index)
                {
                    nodes[i].nodeIndices[index] = (ushort)buildNodes[i].nodeIndices[index];
                }

                nodes[i].triIndices = new int[buildNodes[i].triIndices.Count];
                buildNodes[i].triIndices.CopyTo(nodes[i].triIndices);
                nodes[i].box = buildNodes[i].box;
            }
            buildNodes.Clear();
        }

        public Octree(List<JVector> positions, List<TriangleVertexIndices> tris)
        {
            SetTriangles(positions, tris);
            BuildOctree();
        }

        private void CreateAABox(ref JBBox aabb, EChild child, out JBBox result)
        {
            JVector.Subtract(aabb.Max, aabb.Min, out var dims);
            JVector.Multiply(dims, 0.5f, out dims);

            var offset = JVector.Zero;

            switch (child)
            {
                case EChild.PPP: offset = new JVector(1, 1, 1); break;
                case EChild.PPM: offset = new JVector(1, 1, 0); break;
                case EChild.PMP: offset = new JVector(1, 0, 1); break;
                case EChild.PMM: offset = new JVector(1, 0, 0); break;
                case EChild.MPP: offset = new JVector(0, 1, 1); break;
                case EChild.MPM: offset = new JVector(0, 1, 0); break;
                case EChild.MMP: offset = new JVector(0, 0, 1); break;
                case EChild.MMM: offset = new JVector(0, 0, 0); break;

                default:
                    System.Diagnostics.Debug.WriteLine("Octree.CreateAABox  got impossible child");
                    break;
            }

            var min = new JVector(offset.X * dims.X, offset.Y * dims.Y, offset.Z * dims.Z);

            JVector.Add(min, aabb.Min, out min);
            JVector.Add(min, dims, out var max);

            const float extra = 0.00001f;

            JVector.Multiply(dims, extra, out var temp);
            JVector.Subtract(min, temp, out min);
            JVector.Add(max, temp, out max);

            result = new JBBox(min, max);
        }

        private void GatherTriangles(int nodeIndex, ref List<int> tris)
        {
            tris.AddRange(nodes[nodeIndex].triIndices);

            var numChildren = nodes[nodeIndex].nodeIndices.Length;
            for (var i = 0; i < numChildren; ++i)
            {
                int childNodeIndex = nodes[nodeIndex].nodeIndices[i];
                GatherTriangles(childNodeIndex, ref tris);
            }
        }

        public int GetTrianglesIntersectingtAABox(List<int> triangles, in JBBox testBox)
        {
            if (nodes.Length == 0)
            {
                return 0;
            }

            var curStackIndex = 0;
            var endStackIndex = 1;

            var nodeStack = nodeStackPool.GetNew();

            nodeStack[0] = 0;

            var triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                var nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.Contains(testBox) != JBBox.ContainmentType.Disjoint)
                {
                    for (var i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].Contains(testBox) != JBBox.ContainmentType.Disjoint)
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    var numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (var i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);

            return triCount;
        }

        private ArrayResourcePool<ushort> nodeStackPool;

        public int GetTrianglesIntersectingRay(List<int> triangles, JVector rayOrigin, JVector rayDelta)
        {
            if (nodes.Length == 0)
            {
                return 0;
            }

            var curStackIndex = 0;
            var endStackIndex = 1;

            var nodeStack = nodeStackPool.GetNew();
            nodeStack[0] = 0;

            var triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                var nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.SegmentIntersect(rayOrigin, rayDelta))
                {
                    for (var i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                    {
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].SegmentIntersect(rayOrigin, rayDelta))
                        {
                            triangles.Add(nodes[nodeIndex].triIndices[i]);
                            triCount++;
                        }
                    }

                    var numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (var i = 0; i < numChildren; ++i)
                    {
                        nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                    }
                }
            }

            nodeStackPool.GiveBack(nodeStack);
            return triCount;
        }

        public TriangleVertexIndices GetTriangleVertexIndex(int index)
        {
            return tris[index];
        }

        public JVector GetVertex(int vertex)
        {
            return positions[vertex];
        }

        public void GetVertex(int vertex, out JVector result)
        {
            result = positions[vertex];
        }

        public int NumTriangles => tris.Length;
    }
}
