namespace Jitter.LinearMath
{
    public readonly struct JBBox
    {
        public static readonly JBBox LargeBox = new JBBox(JVector.MinValue, JVector.MaxValue);
        public static readonly JBBox SmallBox = new JBBox(JVector.MaxValue, JVector.MinValue);

        public enum ContainmentType
        {
            Disjoint,
            Contains,
            Intersects
        }

        public JVector Min { get; }
        public JVector Max { get; }

        public JBBox(JVector min, JVector max)
        {
            Min = min;
            Max = max;
        }

        internal JBBox InverseTransform(in JVector position, in JMatrix orientation)
        {
            var max = JVector.Subtract(Min, position);
            var min = JVector.Subtract(Max, position);

            var center = JVector.Add(max, min);
            center *= 0.5f;

            var halfExtents = JVector.Subtract(max, min);
            halfExtents *= 0.5f;

            center = JVector.TransposedTransform(center, orientation);

            var abs = JMath.Absolute(orientation);
            halfExtents = JVector.TransposedTransform(halfExtents, abs);

            return new JBBox(
                JVector.Subtract(center, halfExtents),
                JVector.Add(center, halfExtents));
        }

        public JBBox Transform(in JMatrix orientation)
        {
            var halfExtents = 0.5f * (Max - Min);
            var center = 0.5f * (Max + Min);

            JVector.Transform(center, orientation, out center);

            JMath.Absolute(orientation, out var abs);
            JVector.Transform(halfExtents, abs, out halfExtents);

            return new JBBox(
                center - halfExtents,
                center + halfExtents);
        }

        private static bool Intersect1D(
            float start,
            float dir,
            float min,
            float max,
            ref float enter,
            ref float exit)
        {
            if (dir * dir < JMath.Epsilon * JMath.Epsilon)
            {
                return start >= min && start <= max;
            }

            float t0 = (min - start) / dir;
            float t1 = (max - start) / dir;

            if (t0 > t1)
            {
                float tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            if (t0 > exit || t1 < enter)
            {
                return false;
            }

            if (t0 > enter)
            {
                enter = t0;
            }

            if (t1 < exit)
            {
                exit = t1;
            }

            return true;
        }

        public bool SegmentIntersect(in JVector origin, in JVector direction)
        {
            float enter = 0.0f;
            float exit = 1.0f;

            if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            {
                return false;
            }

            if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            {
                return false;
            }

            if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            {
                return false;
            }

            return true;
        }

        public bool RayIntersect(in JVector origin, in JVector direction)
        {
            float enter = 0.0f, exit = float.MaxValue;

            if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            {
                return false;
            }

            if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            {
                return false;
            }

            if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            {
                return false;
            }

            return true;
        }

        public bool SegmentIntersect(JVector origin, JVector direction)
        {
            return SegmentIntersect(origin, direction);
        }

        public bool RayIntersect(JVector origin, JVector direction)
        {
            return RayIntersect(origin, direction);
        }

        public ContainmentType Contains(JVector point)
        {
            return Contains(point);
        }

        public ContainmentType Contains(in JVector point)
        {
            return ((Min.X <= point.X) && (point.X <= Max.X)
                && (Min.Y <= point.Y) && (point.Y <= Max.Y)
                && (Min.Z <= point.Z) && (point.Z <= Max.Z)) ? ContainmentType.Contains : ContainmentType.Disjoint;
        }

        public void GetCorners(JVector[] corners)
        {
            corners[0] = new JVector(Min.X, Max.Y, Max.Z);
            corners[1] = new JVector(Max.X, Max.Y, Max.Z);
            corners[2] = new JVector(Max.X, Min.Y, Max.Z);
            corners[3] = new JVector(Min.X, Min.Y, Max.Z);
            corners[4] = new JVector(Min.X, Max.Y, Min.Z);
            corners[5] = new JVector(Max.X, Max.Y, Min.Z);
            corners[6] = new JVector(Max.X, Min.Y, Min.Z);
            corners[7] = new JVector(Min.X, Min.Y, Min.Z);
        }

        public JBBox AddPoint(in JVector point)
        {
            return new JBBox(
                JVector.Min(Min, point),
                JVector.Max(Max, point));
        }

        public static JBBox CreateFromPoints(JVector[] points)
        {
            var vector3 = JVector.MaxValue;
            var vector2 = JVector.MinValue;

            for (int i = 0; i < points.Length; i++)
            {
                vector3 = JVector.Min(vector3, points[i]);
                vector2 = JVector.Max(vector2, points[i]);
            }

            return new JBBox(vector3, vector2);
        }

        public ContainmentType Contains(JBBox box)
        {
            return Contains(box);
        }

        public ContainmentType Contains(in JBBox box)
        {
            var result = ContainmentType.Disjoint;
            if ((Max.X >= box.Min.X) && (Min.X <= box.Max.X) && (Max.Y >= box.Min.Y) && (Min.Y <= box.Max.Y) && (Max.Z >= box.Min.Z) && (Min.Z <= box.Max.Z))
            {
                result = ((Min.X <= box.Min.X) && (box.Max.X <= Max.X) && (Min.Y <= box.Min.Y) && (box.Max.Y <= Max.Y) && (Min.Z <= box.Min.Z) && (box.Max.Z <= Max.Z)) ? ContainmentType.Contains : ContainmentType.Intersects;
            }

            return result;
        }

        public static JBBox CreateMerged(JBBox original, JBBox additional)
        {
            CreateMerged(original, additional, out var result);
            return result;
        }

        public static void CreateMerged(in JBBox original, in JBBox additional, out JBBox result)
        {
            JVector.Min(original.Min, additional.Min, out var vector2);
            JVector.Max(original.Max, additional.Max, out var vector);

            result = new JBBox(vector2, vector);
        }

        public JVector Center => (Min + Max) * (1.0f / 2.0f);

        internal float Perimeter => 2.0f * (((Max.X - Min.X) * (Max.Y - Min.Y))
                    + ((Max.X - Min.X) * (Max.Z - Min.Z))
                    + ((Max.Z - Min.Z) * (Max.Y - Min.Y)));
    }
}
