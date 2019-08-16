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
}