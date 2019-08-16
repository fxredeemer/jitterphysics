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

            int steps = (int)factor;

            for (int thetaIndex = 0; thetaIndex < steps; thetaIndex++)
            {
                float theta = JMath.Pi / (steps - 1) * thetaIndex;
                float sinTheta = (float)Math.Sin(theta);
                float cosTheta = (float)Math.Cos(theta);

                for (int phiIndex = 0; phiIndex < steps; phiIndex++)
                {
                    float phi = ((2.0f * JMath.Pi) / (steps - 0) * phiIndex) - JMath.Pi;
                    float sinPhi = (float)Math.Sin(phi);
                    float cosPhi = (float)Math.Cos(phi);

                    var dir = new JVector(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);

                    int index = FindExtremePoint(pointCloud, ref dir);
                    allIndices.Add(index);
                }
            }

            allIndices.Sort();

            for (int i = 1; i < allIndices.Count; i++)
            {
                if (allIndices[i - 1] == allIndices[i])
                { allIndices.RemoveAt(i - 1); i--; }
            }

            return allIndices.ToArray();
        }

        private static int FindExtremePoint(List<JVector> points, ref JVector dir)
        {
            int index = 0;
            float current = float.MinValue;

            JVector point; float value;

            for (int i = 1; i < points.Count; i++)
            {
                point = points[i];

                value = JVector.Dot(ref point, ref dir);
                if (value > current) { current = value; index = i; }
            }

            return index;
        }
    }
}
