using Jitter.LinearMath;
using System;
using System.Collections.Generic;

namespace Jitter.Collision
{
    public struct TriangleVertexIndices
    {
        public int I0;
        public int I1;
        public int I2;

        public TriangleVertexIndices(int i0, int i1, int i2)
        {
            I0 = i0;
            I1 = i1;
            I2 = i2;
        }

        public void Set(int i0, int i1, int i2)
        {
            I0 = i0; I1 = i1; I2 = i2;
        }
    }
}
