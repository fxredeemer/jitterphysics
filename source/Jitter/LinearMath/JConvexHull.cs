using System;
using System.Collections.Generic;

namespace Jitter.LinearMath
{
    public static class JConvexHull
    {
        public enum Approximation
        {
            Level1 = 6,
            Level2 = 7,
            Level3 = 8,
            Level4 = 9,
            Level5 = 10,
            Level6 = 11,
            Level7 = 12,
            Level8 = 13,
            Level9 = 15,
            Level10 = 20,
            Level15 = 25,
            Level20 = 30
        }

        public static int[] Build(List<JVector> pointCloud, Approximation factor)
        {
            var allIndices = new List<int>();

            var steps = (int)factor;

            for (var thetaIndex = 0; thetaIndex < steps; thetaIndex++)
            {
                var theta = JMath.Pi / (steps - 1) * thetaIndex;
                var sinTheta = (float)Math.Sin(theta);
                var cosTheta = (float)Math.Cos(theta);

                for (var phiIndex = 0; phiIndex < steps; phiIndex++)
                {
                    var phi = (2.0f * JMath.Pi / (steps - 0) * phiIndex) - JMath.Pi;
                    var sinPhi = (float)Math.Sin(phi);
                    var cosPhi = (float)Math.Cos(phi);

                    var dir = new JVector(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);

                    var index = FindExtremePoint(pointCloud, ref dir);
                    allIndices.Add(index);
                }
            }

            allIndices.Sort();

            for (var i = 1; i < allIndices.Count; i++)
            {
                if (allIndices[i - 1] == allIndices[i])
                {
                    allIndices.RemoveAt(i - 1); i--;
                }
            }

            return allIndices.ToArray();
        }

        private static int FindExtremePoint(List<JVector> points, ref JVector dir)
        {
            var index = 0;
            var current = float.MinValue;

            JVector point; float value;

            for (var i = 1; i < points.Count; i++)
            {
                point = points[i];

                value = JVector.Dot(ref point, ref dir);
                if (value > current)
                {
                    current = value; index = i;
                }
            }

            return index;
        }
    }
}
