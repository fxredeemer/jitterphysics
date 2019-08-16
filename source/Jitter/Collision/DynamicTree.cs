using Jitter.LinearMath;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Collision
{
    public struct DynamicTreeNode<T>
    {
        public JBBox AABB;

        public float MinorRandomExtension;

        public int Child1;
        public int Child2;

        public int LeafCount;
        public int ParentOrNext;
        public T UserData;

        public bool IsLeaf()
        {
            return Child1 == DynamicTree<T>.NullNode;
        }
    }

    public class DynamicTree<T>
    {
        internal const int NullNode = -1;
        private int _freeList;
        private int _insertionCount;
        private int _nodeCapacity;
        private int _nodeCount;
        private const float SettingsAABBMultiplier = 2.0f;

        private readonly float settingsRndExtension = 0.1f;

        public int Root { get; private set; }
        public DynamicTreeNode<T>[] Nodes { get; private set; }

        public DynamicTree()
            : this(0.1f)
        {
        }

        public DynamicTree(float rndExtension)
        {
            settingsRndExtension = rndExtension;
            Root = NullNode;

            _nodeCapacity = 16;
            Nodes = new DynamicTreeNode<T>[_nodeCapacity];

            for (int i = 0; i < _nodeCapacity - 1; ++i)
            {
                Nodes[i].ParentOrNext = i + 1;
            }
            Nodes[_nodeCapacity - 1].ParentOrNext = NullNode;
        }

        private readonly Random rnd = new Random();

        public int AddProxy(ref JBBox aabb, T userData)
        {
            int proxyId = AllocateNode();

            Nodes[proxyId].MinorRandomExtension = (float)rnd.NextDouble() * settingsRndExtension;

            var r = new JVector(Nodes[proxyId].MinorRandomExtension);
            Nodes[proxyId].AABB.Min = aabb.Min - r;
            Nodes[proxyId].AABB.Max = aabb.Max + r;
            Nodes[proxyId].UserData = userData;
            Nodes[proxyId].LeafCount = 1;

            InsertLeaf(proxyId);

            return proxyId;
        }

        public void RemoveProxy(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            Debug.Assert(Nodes[proxyId].IsLeaf());

            RemoveLeaf(proxyId);
            FreeNode(proxyId);
        }

        public bool MoveProxy(int proxyId, ref JBBox aabb, JVector displacement)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);

            Debug.Assert(Nodes[proxyId].IsLeaf());

            if (Nodes[proxyId].AABB.Contains(ref aabb) != JBBox.ContainmentType.Disjoint)
            {
                return false;
            }

            RemoveLeaf(proxyId);

            var b = aabb;
            var r = new JVector(Nodes[proxyId].MinorRandomExtension);
            b.Min = b.Min - r;
            b.Max = b.Max + r;

            var d = SettingsAABBMultiplier * displacement;

            if (d.X < 0.0f)
            {
                b.Min.X += d.X;
            }
            else
            {
                b.Max.X += d.X;
            }

            if (d.Y < 0.0f)
            {
                b.Min.Y += d.Y;
            }
            else
            {
                b.Max.Y += d.Y;
            }

            if (d.Z < 0.0f)
            {
                b.Min.Z += d.Z;
            }
            else
            {
                b.Max.Z += d.Z;
            }

            Nodes[proxyId].AABB = b;

            InsertLeaf(proxyId);
            return true;
        }

        public T GetUserData(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            return Nodes[proxyId].UserData;
        }

        public void GetFatAABB(int proxyId, out JBBox fatAABB)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            fatAABB = Nodes[proxyId].AABB;
        }

        public int ComputeHeight()
        {
            return ComputeHeight(Root);
        }

        public void Query(JVector origin, JVector direction, List<int> collisions)
        {
            var stack = stackPool.GetNew();

            stack.Push(Root);

            while (stack.Count > 0)
            {
                int nodeId = stack.Pop();
                var node = Nodes[nodeId];

                if (node.AABB.RayIntersect(ref origin, ref direction))
                {
                    if (node.IsLeaf())
                    {
                        collisions.Add(nodeId);
                    }
                    else
                    {
                        if (Nodes[node.Child1].AABB.RayIntersect(ref origin, ref direction))
                        {
                            stack.Push(node.Child1);
                        }

                        if (Nodes[node.Child2].AABB.RayIntersect(ref origin, ref direction))
                        {
                            stack.Push(node.Child2);
                        }
                    }
                }
            }

            stackPool.GiveBack(stack);
        }

        public void Query(List<int> other, List<int> my, DynamicTree<T> tree)
        {
            var stack1 = stackPool.GetNew();
            var stack2 = stackPool.GetNew();

            stack1.Push(Root);
            stack2.Push(tree.Root);

            while (stack1.Count > 0)
            {
                int nodeId1 = stack1.Pop();
                int nodeId2 = stack2.Pop();

                if (nodeId1 == NullNode)
                {
                    continue;
                }

                if (nodeId2 == NullNode)
                {
                    continue;
                }

                if (tree.Nodes[nodeId2].AABB.Contains(ref Nodes[nodeId1].AABB) != JBBox.ContainmentType.Disjoint)
                {
                    if (Nodes[nodeId1].IsLeaf() && tree.Nodes[nodeId2].IsLeaf())
                    {
                        my.Add(nodeId1);
                        other.Add(nodeId2);
                    }
                    else if (tree.Nodes[nodeId2].IsLeaf())
                    {
                        stack1.Push(Nodes[nodeId1].Child1);
                        stack2.Push(nodeId2);

                        stack1.Push(Nodes[nodeId1].Child2);
                        stack2.Push(nodeId2);
                    }
                    else if (Nodes[nodeId1].IsLeaf())
                    {
                        stack1.Push(nodeId1);
                        stack2.Push(tree.Nodes[nodeId2].Child1);

                        stack1.Push(nodeId1);
                        stack2.Push(tree.Nodes[nodeId2].Child2);
                    }
                    else
                    {
                        stack1.Push(Nodes[nodeId1].Child1);
                        stack2.Push(tree.Nodes[nodeId2].Child1);

                        stack1.Push(Nodes[nodeId1].Child1);
                        stack2.Push(tree.Nodes[nodeId2].Child2);

                        stack1.Push(Nodes[nodeId1].Child2);
                        stack2.Push(tree.Nodes[nodeId2].Child1);

                        stack1.Push(Nodes[nodeId1].Child2);
                        stack2.Push(tree.Nodes[nodeId2].Child2);
                    }
                }
            }

            stackPool.GiveBack(stack1);
            stackPool.GiveBack(stack2);
        }

        private readonly ResourcePool<Stack<int>> stackPool = new ResourcePool<Stack<int>>();

        public void Query(List<int> my, ref JBBox aabb)
        {
            var _stack = stackPool.GetNew();

            _stack.Push(Root);

            while (_stack.Count > 0)
            {
                int nodeId = _stack.Pop();
                if (nodeId == NullNode)
                {
                    continue;
                }

                var node = Nodes[nodeId];

                if (aabb.Contains(ref node.AABB) != JBBox.ContainmentType.Disjoint)
                {
                    if (node.IsLeaf())
                    {
                        my.Add(nodeId);
                    }
                    else
                    {
                        _stack.Push(node.Child1);
                        _stack.Push(node.Child2);
                    }
                }
            }

            stackPool.GiveBack(_stack);
        }

        private int CountLeaves(int nodeId)
        {
            if (nodeId == NullNode)
            {
                return 0;
            }

            Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
            var node = Nodes[nodeId];

            if (node.IsLeaf())
            {
                Debug.Assert(node.LeafCount == 1);
                return 1;
            }

            int count1 = CountLeaves(node.Child1);
            int count2 = CountLeaves(node.Child2);
            int count = count1 + count2;
            Debug.Assert(count == node.LeafCount);
            return count;
        }

        private void Validate()
        {
            CountLeaves(Root);
        }

        private int AllocateNode()
        {
            if (_freeList == NullNode)
            {
                Debug.Assert(_nodeCount == _nodeCapacity);

                var oldNodes = Nodes;
                _nodeCapacity *= 2;
                Nodes = new DynamicTreeNode<T>[_nodeCapacity];
                Array.Copy(oldNodes, Nodes, _nodeCount);

                for (int i = _nodeCount; i < _nodeCapacity - 1; ++i)
                {
                    Nodes[i].ParentOrNext = i + 1;
                }
                Nodes[_nodeCapacity - 1].ParentOrNext = NullNode;
                _freeList = _nodeCount;
            }

            int nodeId = _freeList;
            _freeList = Nodes[nodeId].ParentOrNext;
            Nodes[nodeId].ParentOrNext = NullNode;
            Nodes[nodeId].Child1 = NullNode;
            Nodes[nodeId].Child2 = NullNode;
            Nodes[nodeId].LeafCount = 0;
            ++_nodeCount;
            return nodeId;
        }

        private void FreeNode(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
            Debug.Assert(0 < _nodeCount);
            Nodes[nodeId].ParentOrNext = _freeList;
            _freeList = nodeId;
            --_nodeCount;
        }

        private void InsertLeaf(int leaf)
        {
            ++_insertionCount;

            if (Root == NullNode)
            {
                Root = leaf;
                Nodes[Root].ParentOrNext = NullNode;
                return;
            }

            var leafAABB = Nodes[leaf].AABB;
            int sibling = Root;
            while (Nodes[sibling].IsLeaf() == false)
            {
                int child1 = Nodes[sibling].Child1;
                int child2 = Nodes[sibling].Child2;

                JBBox.CreateMerged(ref Nodes[sibling].AABB, ref leafAABB, out Nodes[sibling].AABB);

                Nodes[sibling].LeafCount += 1;

                float siblingArea = Nodes[sibling].AABB.Perimeter;
                var parentAABB = new JBBox();
                JBBox.CreateMerged(ref Nodes[sibling].AABB, ref leafAABB, out Nodes[sibling].AABB);

                float parentArea = parentAABB.Perimeter;
                float cost1 = 2.0f * parentArea;

                float inheritanceCost = 2.0f * (parentArea - siblingArea);

                float cost2;
                if (Nodes[child1].IsLeaf())
                {
                    var aabb = new JBBox();
                    JBBox.CreateMerged(ref leafAABB, ref Nodes[child1].AABB, out aabb);
                    cost2 = aabb.Perimeter + inheritanceCost;
                }
                else
                {
                    var aabb = new JBBox();
                    JBBox.CreateMerged(ref leafAABB, ref Nodes[child1].AABB, out aabb);

                    float oldArea = Nodes[child1].AABB.Perimeter;
                    float newArea = aabb.Perimeter;
                    cost2 = (newArea - oldArea) + inheritanceCost;
                }

                float cost3;
                if (Nodes[child2].IsLeaf())
                {
                    var aabb = new JBBox();
                    JBBox.CreateMerged(ref leafAABB, ref Nodes[child2].AABB, out aabb);
                    cost3 = aabb.Perimeter + inheritanceCost;
                }
                else
                {
                    var aabb = new JBBox();
                    JBBox.CreateMerged(ref leafAABB, ref Nodes[child2].AABB, out aabb);
                    float oldArea = Nodes[child2].AABB.Perimeter;
                    float newArea = aabb.Perimeter;
                    cost3 = newArea - oldArea + inheritanceCost;
                }

                if (cost1 < cost2 && cost1 < cost3)
                {
                    break;
                }

                JBBox.CreateMerged(ref leafAABB, ref Nodes[sibling].AABB, out Nodes[sibling].AABB);

                if (cost2 < cost3)
                {
                    sibling = child1;
                }
                else
                {
                    sibling = child2;
                }
            }

            int oldParent = Nodes[sibling].ParentOrNext;
            int newParent = AllocateNode();
            Nodes[newParent].ParentOrNext = oldParent;
            Nodes[newParent].UserData = default(T);
            JBBox.CreateMerged(ref leafAABB, ref Nodes[sibling].AABB, out Nodes[newParent].AABB);
            Nodes[newParent].LeafCount = Nodes[sibling].LeafCount + 1;

            if (oldParent != NullNode)
            {
                if (Nodes[oldParent].Child1 == sibling)
                {
                    Nodes[oldParent].Child1 = newParent;
                }
                else
                {
                    Nodes[oldParent].Child2 = newParent;
                }

                Nodes[newParent].Child1 = sibling;
                Nodes[newParent].Child2 = leaf;
                Nodes[sibling].ParentOrNext = newParent;
                Nodes[leaf].ParentOrNext = newParent;
            }
            else
            {
                Nodes[newParent].Child1 = sibling;
                Nodes[newParent].Child2 = leaf;
                Nodes[sibling].ParentOrNext = newParent;
                Nodes[leaf].ParentOrNext = newParent;
                Root = newParent;
            }
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == Root)
            {
                Root = NullNode;
                return;
            }

            int parent = Nodes[leaf].ParentOrNext;
            int grandParent = Nodes[parent].ParentOrNext;
            int sibling;
            if (Nodes[parent].Child1 == leaf)
            {
                sibling = Nodes[parent].Child2;
            }
            else
            {
                sibling = Nodes[parent].Child1;
            }

            if (grandParent != NullNode)
            {
                if (Nodes[grandParent].Child1 == parent)
                {
                    Nodes[grandParent].Child1 = sibling;
                }
                else
                {
                    Nodes[grandParent].Child2 = sibling;
                }
                Nodes[sibling].ParentOrNext = grandParent;
                FreeNode(parent);

                parent = grandParent;
                while (parent != NullNode)
                {
                    JBBox.CreateMerged(ref Nodes[Nodes[parent].Child1].AABB,
                        ref Nodes[Nodes[parent].Child2].AABB, out Nodes[parent].AABB);

                    Debug.Assert(Nodes[parent].LeafCount > 0);
                    Nodes[parent].LeafCount -= 1;

                    parent = Nodes[parent].ParentOrNext;
                }
            }
            else
            {
                Root = sibling;
                Nodes[sibling].ParentOrNext = NullNode;
                FreeNode(parent);
            }
        }

        private int ComputeHeight(int nodeId)
        {
            if (nodeId == NullNode)
            {
                return 0;
            }

            Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
            var node = Nodes[nodeId];
            int height1 = ComputeHeight(node.Child1);
            int height2 = ComputeHeight(node.Child2);
            return 1 + Math.Max(height1, height2);
        }
    }
}