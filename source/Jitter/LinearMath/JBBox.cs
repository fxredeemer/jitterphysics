namespace Jitter.LinearMath
{
    public readonly struct JBBox
    {
        public enum ContainmentType
        {
            Disjoint,
            Contains,
            Intersects
        }

        public readonly JVector Min;
        public readonly JVector Max;

        public static readonly JBBox LargeBox = new JBBox(
            new JVector(float.MinValue),
            new JVector(float.MaxValue));

        public static readonly JBBox SmallBox = new JBBox(
            new JVector(float.MaxValue),
            new JVector(float.MinValue));

        public JBBox(JVector min, JVector max)
        {
            Min = min;
            Max = max;
        }

        internal void InverseTransform(ref JVector position, ref JMatrix orientation)
        {
            JVector.Subtract(ref Max, ref position, out Max);
            JVector.Subtract(ref Min, ref position, out Min);

            JVector.Add(ref Max, ref Min, out var center);
            center *= 0.5f;

            JVector.Subtract(ref Max, ref Min, out var halfExtents);
            halfExtents *= 0.5f;

            JVector.TransposedTransform(ref center, ref orientation, out center);

            JMath.Absolute(ref orientation, out var abs);
            JVector.TransposedTransform(ref halfExtents, ref abs, out halfExtents);

            JVector.Add(ref center, ref halfExtents, out Max);
            JVector.Subtract(ref center, ref halfExtents, out Min);
        }

        public void Transform(ref JMatrix orientation)
        {
            var halfExtents = 0.5f * (Max - Min);
            var center = 0.5f * (Max + Min);

            JVector.Transform(ref center, ref orientation, out center);

            JMath.Absolute(ref orientation, out var abs);
            JVector.Transform(ref halfExtents, ref abs, out halfExtents);

            Max = center + halfExtents;
            Min = center - halfExtents;
        }

        private bool Intersect1D(float start, float dir, float min, float max,
            ref float enter, ref float exit)
        {
            if (dir * dir < JMath.Epsilon * JMath.Epsilon)
            {
                return start >= min && start <= max;
            }

            var t0 = (min - start) / dir;
            var t1 = (max - start) / dir;

            if (t0 > t1) { var tmp = t0; t0 = t1; t1 = tmp; }

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

        public bool SegmentIntersect(ref JVector origin, ref JVector direction)
        {
            float enter = 0.0f, exit = 1.0f;

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

        public bool RayIntersect(ref JVector origin, ref JVector direction)
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
            return SegmentIntersect(ref origin, ref direction);
        }

        public bool RayIntersect(JVector origin, JVector direction)
        {
            return RayIntersect(ref origin, ref direction);
        }

        public ContainmentType Contains(JVector point)
        {
            return Contains(ref point);
        }

        public ContainmentType Contains(ref JVector point)
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

        public void AddPoint(JVector point)
        {
            AddPoint(ref point);
        }

        public void AddPoint(ref JVector point)
        {
            JVector.Max(ref Max, ref point, out Max);
            JVector.Min(ref Min, ref point, out Min);
        }

        public static JBBox CreateFromPoints(JVector[] points)
        {
            var vector3 = new JVector(float.MaxValue);
            var vector2 = new JVector(float.MinValue);

            for (var i = 0; i < points.Length; i++)
            {
                JVector.Min(ref vector3, ref points[i], out vector3);
                JVector.Max(ref vector2, ref points[i], out vector2);
            }
            return new JBBox(vector3, vector2);
        }

        public ContainmentType Contains(JBBox box)
        {
            return Contains(ref box);
        }

        public ContainmentType Contains(ref JBBox box)
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
            CreateMerged(ref original, ref additional, out var result);
            return result;
        }

        public static void CreateMerged(ref JBBox original, ref JBBox additional, out JBBox result)
        {
            JVector.Min(ref original.Min, ref additional.Min, out var vector2);
            JVector.Max(ref original.Max, ref additional.Max, out var vector);
            result.Min = vector2;
            result.Max = vector;
        }

        public JVector Center => (Min + Max) * (1.0f / 2.0f);

        internal float Perimeter => 2.0f * (((Max.X - Min.X) * (Max.Y - Min.Y))
                    + ((Max.X - Min.X) * (Max.Z - Min.Z))
                    + ((Max.Z - Min.Z) * (Max.Y - Min.Y)));
    }
}
