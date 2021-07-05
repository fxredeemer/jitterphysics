using Jitter.LinearMath;

namespace Jitter.Collision
{
    public interface ISupportMappable
    {
        void SupportMapping(in JVector direction, out JVector result);

        void SupportCenter(out JVector center);
    }

    public sealed class XenoCollide
    {
        private const float CollideEpsilon = 1e-4f;
        private const int MaximumIterations = 34;

        private static void SupportMapTransformed(
            ISupportMappable support,
            in JMatrix orientation,
            in JVector position,
            in JVector direction,
            out JVector result)
        {
            result = new JVector(
                (direction.X * orientation.M11) + (direction.Y * orientation.M12) + (direction.Z * orientation.M13),
                (direction.X * orientation.M21) + (direction.Y * orientation.M22) + (direction.Z * orientation.M23),
                (direction.X * orientation.M31) + (direction.Y * orientation.M32) + (direction.Z * orientation.M33));

            support.SupportMapping(result, out result);

            var x = (result.X * orientation.M11) + (result.Y * orientation.M21) + (result.Z * orientation.M31);
            var y = (result.X * orientation.M12) + (result.Y * orientation.M22) + (result.Z * orientation.M32);
            var z = (result.X * orientation.M13) + (result.Y * orientation.M23) + (result.Z * orientation.M33);

            result = new JVector(
                position.X + x,
                position.Y + y,
                position.Z + z);
        }

        public static bool Detect(
            ISupportMappable support1,
            ISupportMappable support2,
            JMatrix orientation1,
            JMatrix orientation2,
            JVector position1,
            JVector position2,
            out JVector point,
            out JVector normal,
            out float penetration)
        {
            JVector temp1;
            JVector mn;

            point = JVector.Zero;
            penetration = 0.0f;

            support1.SupportCenter(out var v01);
            JVector.Transform(v01, in orientation1, out v01);
            JVector.Add(position1, v01, out v01);

            support2.SupportCenter(out var v02);
            JVector.Transform(v02, orientation2, out v02);
            JVector.Add(position2, v02, out v02);

            JVector.Subtract(v02, v01, out var v0);

            if (v0.IsNearlyZero())
            {
                v0 = new JVector(0.00001f, 0, 0);
            }

            mn = v0;
            JVector.Negate(v0, out normal);

            SupportMapTransformed(support1, orientation1, position1, mn, out var v11);
            SupportMapTransformed(support2, orientation2, position2, normal, out var v12);
            JVector.Subtract(v12, v11, out var v1);

            if (JVector.Dot(v1, normal) <= 0.0f)
            {
                return false;
            }

            JVector.Cross(v1, v0, out normal);

            if (normal.IsNearlyZero())
            {
                JVector.Subtract(v1, v0, out normal);

                normal = JVector.Normalize(normal);

                point = v11;
                JVector.Add(point, v12, out point);
                JVector.Multiply(point, 0.5f, out point);

                JVector.Subtract(v12, v11, out temp1);
                penetration = JVector.Dot(temp1, normal);

                return true;
            }

            JVector.Negate(normal, out mn);
            SupportMapTransformed(support1, orientation1, position1, mn, out var v21);
            SupportMapTransformed(support2, orientation2, position2, normal, out var v22);
            JVector.Subtract(v22, v21, out var v2);

            if (JVector.Dot(v2, normal) <= 0.0f)
            {
                return false;
            }

            JVector.Subtract(v1, v0, out temp1);
            JVector.Subtract(v2, v0, out var temp2);
            JVector.Cross(temp1, temp2, out normal);

            var dist = JVector.Dot(normal, v0);

            if (dist > 0.0f)
            {
                JVector.Swap(ref v1, ref v2);
                JVector.Swap(ref v11, ref v21);
                JVector.Swap(ref v12, ref v22);
                JVector.Negate(normal, out normal);
            }

            var phase2 = 0;
            var phase1 = 0;
            var hit = false;

            while (true)
            {
                if (phase1 > MaximumIterations)
                {
                    return false;
                }

                phase1++;

                JVector.Negate(normal, out mn);
                SupportMapTransformed(support1, orientation1, position1, mn, out var v31);
                SupportMapTransformed(support2, orientation2, position2, normal, out var v32);
                JVector.Subtract(v32, v31, out var v3);

                if (JVector.Dot(v3, normal) <= 0.0f)
                {
                    return false;
                }

                JVector.Cross(v1, v3, out temp1);
                if (JVector.Dot(temp1, v0) < 0.0f)
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    JVector.Subtract(v1, v0, out temp1);
                    JVector.Subtract(v3, v0, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                JVector.Cross(v3, v2, out temp1);
                if (JVector.Dot(temp1, v0) < 0.0f)
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    JVector.Subtract(v3, v0, out temp1);
                    JVector.Subtract(v2, v0, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                while (true)
                {
                    phase2++;

                    JVector.Subtract(v2, v1, out temp1);
                    JVector.Subtract(v3, v1, out temp2);
                    JVector.Cross(temp1, temp2, out normal);

                    if (normal.IsNearlyZero())
                    {
                        return true;
                    }

                    normal = JVector.Normalize(normal);

                    var d = JVector.Dot(normal, v1);

                    if (d >= 0 && !hit)
                    {
                        hit = true;
                    }

                    JVector.Negate(normal, out mn);
                    SupportMapTransformed(support1, orientation1, position1, mn, out var v41);
                    SupportMapTransformed(support2, orientation2, position2, normal, out var v42);
                    JVector.Subtract(v42, v41, out var v4);

                    JVector.Subtract(v4, v3, out temp1);
                    var delta = JVector.Dot(temp1, normal);
                    penetration = JVector.Dot(v4, normal);

                    if (delta <= CollideEpsilon || penetration <= 0.0f || phase2 > MaximumIterations)
                    {
                        if (hit)
                        {
                            JVector.Cross(v1, v2, out temp1);
                            var b0 = JVector.Dot(temp1, v3);
                            JVector.Cross(v3, v2, out temp1);
                            var b1 = JVector.Dot(temp1, v0);
                            JVector.Cross(v0, v1, out temp1);
                            var b2 = JVector.Dot(temp1, v3);
                            JVector.Cross(v2, v1, out temp1);
                            var b3 = JVector.Dot(temp1, v0);

                            var sum = b0 + b1 + b2 + b3;

                            if (sum <= 0)
                            {
                                b0 = 0;
                                JVector.Cross(v2, v3, out temp1);
                                b1 = JVector.Dot(temp1, normal);
                                JVector.Cross(v3, v1, out temp1);
                                b2 = JVector.Dot(temp1, normal);
                                JVector.Cross(v1, v2, out temp1);
                                b3 = JVector.Dot(temp1, normal);

                                sum = b1 + b2 + b3;
                            }

                            var inv = 1.0f / sum;

                            JVector.Multiply(v01, b0, out point);
                            JVector.Multiply(v11, b1, out temp1);
                            JVector.Add(point, temp1, out point);
                            JVector.Multiply(v21, b2, out temp1);
                            JVector.Add(point, temp1, out point);
                            JVector.Multiply(v31, b3, out temp1);
                            JVector.Add(point, temp1, out point);

                            JVector.Multiply(v02, b0, out temp2);
                            JVector.Add(temp2, point, out point);
                            JVector.Multiply(v12, b1, out temp1);
                            JVector.Add(point, temp1, out point);
                            JVector.Multiply(v22, b2, out temp1);
                            JVector.Add(point, temp1, out point);
                            JVector.Multiply(v32, b3, out temp1);
                            JVector.Add(point, temp1, out point);

                            JVector.Multiply(point, inv * 0.5f, out point);
                        }

                        return hit;
                    }

                    JVector.Cross(v4, v0, out temp1);
                    var dot = JVector.Dot(temp1, v1);

                    if (dot >= 0.0f)
                    {
                        dot = JVector.Dot(temp1, v2);

                        if (dot >= 0.0f)
                        {
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                        else
                        {
                            v3 = v4;
                            v31 = v41;
                            v32 = v42;
                        }
                    }
                    else
                    {
                        dot = JVector.Dot(temp1, v3);

                        if (dot >= 0.0f)
                        {
                            v2 = v4;
                            v21 = v41;
                            v22 = v42;
                        }
                        else
                        {
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                    }
                }
            }
        }
    }
}