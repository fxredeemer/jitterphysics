using Jitter.LinearMath;

namespace Jitter.Collision
{
    public interface ISupportMappable
    {
        void SupportMapping(ref JVector direction, out JVector result);

        void SupportCenter(out JVector center);
    }

    public sealed class XenoCollide
    {
        private const float CollideEpsilon = 1e-4f;
        private const int MaximumIterations = 34;

        private static void SupportMapTransformed(ISupportMappable support,
            ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            result.X = (direction.X * orientation.M11) + (direction.Y * orientation.M12) + (direction.Z * orientation.M13);
            result.Y = (direction.X * orientation.M21) + (direction.Y * orientation.M22) + (direction.Z * orientation.M23);
            result.Z = (direction.X * orientation.M31) + (direction.Y * orientation.M32) + (direction.Z * orientation.M33);

            support.SupportMapping(ref result, out result);

            float x = (result.X * orientation.M11) + (result.Y * orientation.M21) + (result.Z * orientation.M31);
            float y = (result.X * orientation.M12) + (result.Y * orientation.M22) + (result.Z * orientation.M32);
            float z = (result.X * orientation.M13) + (result.Y * orientation.M23) + (result.Z * orientation.M33);

            result.X = position.X + x;
            result.Y = position.Y + y;
            result.Z = position.Z + z;
        }

        public static bool Detect(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
     ref JMatrix orientation2, ref JVector position1, ref JVector position2,
     out JVector point, out JVector normal, out float penetration)
        {
            JVector temp1;
            JVector mn;

            point = normal = JVector.Zero;
            penetration = 0.0f;

            support1.SupportCenter(out var v01);
            JVector.Transform(ref v01, ref orientation1, out v01);
            JVector.Add(ref position1, ref v01, out v01);

            support2.SupportCenter(out var v02);
            JVector.Transform(ref v02, ref orientation2, out v02);
            JVector.Add(ref position2, ref v02, out v02);

            JVector.Subtract(ref v02, ref v01, out var v0);

            if (v0.IsNearlyZero())
            {
                v0 = new JVector(0.00001f, 0, 0);
            }

            mn = v0;
            JVector.Negate(ref v0, out normal);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out var v11);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out var v12);
            JVector.Subtract(ref v12, ref v11, out var v1);

            if (JVector.Dot(ref v1, ref normal) <= 0.0f)
            {
                return false;
            }

            JVector.Cross(ref v1, ref v0, out normal);

            if (normal.IsNearlyZero())
            {
                JVector.Subtract(ref v1, ref v0, out normal);

                normal.Normalize();

                point = v11;
                JVector.Add(ref point, ref v12, out point);
                JVector.Multiply(ref point, 0.5f, out point);

                JVector.Subtract(ref v12, ref v11, out temp1);
                penetration = JVector.Dot(ref temp1, ref normal);

                return true;
            }

            JVector.Negate(ref normal, out mn);
            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out var v21);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out var v22);
            JVector.Subtract(ref v22, ref v21, out var v2);

            if (JVector.Dot(ref v2, ref normal) <= 0.0f)
            {
                return false;
            }

            JVector.Subtract(ref v1, ref v0, out temp1);
            JVector.Subtract(ref v2, ref v0, out var temp2);
            JVector.Cross(ref temp1, ref temp2, out normal);

            float dist = JVector.Dot(ref normal, ref v0);

            if (dist > 0.0f)
            {
                JVector.Swap(ref v1, ref v2);
                JVector.Swap(ref v11, ref v21);
                JVector.Swap(ref v12, ref v22);
                JVector.Negate(ref normal, out normal);
            }

            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            while (true)
            {
                if (phase1 > MaximumIterations)
                {
                    return false;
                }

                phase1++;

                JVector.Negate(ref normal, out mn);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out var v31);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out var v32);
                JVector.Subtract(ref v32, ref v31, out var v3);

                if (JVector.Dot(ref v3, ref normal) <= 0.0f)
                {
                    return false;
                }

                JVector.Cross(ref v1, ref v3, out temp1);
                if (JVector.Dot(ref temp1, ref v0) < 0.0f)
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    JVector.Subtract(ref v1, ref v0, out temp1);
                    JVector.Subtract(ref v3, ref v0, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                JVector.Cross(ref v3, ref v2, out temp1);
                if (JVector.Dot(ref temp1, ref v0) < 0.0f)
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    JVector.Subtract(ref v3, ref v0, out temp1);
                    JVector.Subtract(ref v2, ref v0, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                while (true)
                {
                    phase2++;

                    JVector.Subtract(ref v2, ref v1, out temp1);
                    JVector.Subtract(ref v3, ref v1, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);

                    if (normal.IsNearlyZero())
                    {
                        return true;
                    }

                    normal.Normalize();

                    float d = JVector.Dot(ref normal, ref v1);

                    if (d >= 0 && !hit)
                    {
                        hit = true;
                    }

                    JVector.Negate(ref normal, out mn);
                    SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out var v41);
                    SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out var v42);
                    JVector.Subtract(ref v42, ref v41, out var v4);

                    JVector.Subtract(ref v4, ref v3, out temp1);
                    float delta = JVector.Dot(ref temp1, ref normal);
                    penetration = JVector.Dot(ref v4, ref normal);

                    if (delta <= CollideEpsilon || penetration <= 0.0f || phase2 > MaximumIterations)
                    {
                        if (hit)
                        {
                            JVector.Cross(ref v1, ref v2, out temp1);
                            float b0 = JVector.Dot(ref temp1, ref v3);
                            JVector.Cross(ref v3, ref v2, out temp1);
                            float b1 = JVector.Dot(ref temp1, ref v0);
                            JVector.Cross(ref v0, ref v1, out temp1);
                            float b2 = JVector.Dot(ref temp1, ref v3);
                            JVector.Cross(ref v2, ref v1, out temp1);
                            float b3 = JVector.Dot(ref temp1, ref v0);

                            float sum = b0 + b1 + b2 + b3;

                            if (sum <= 0)
                            {
                                b0 = 0;
                                JVector.Cross(ref v2, ref v3, out temp1);
                                b1 = JVector.Dot(ref temp1, ref normal);
                                JVector.Cross(ref v3, ref v1, out temp1);
                                b2 = JVector.Dot(ref temp1, ref normal);
                                JVector.Cross(ref v1, ref v2, out temp1);
                                b3 = JVector.Dot(ref temp1, ref normal);

                                sum = b1 + b2 + b3;
                            }

                            float inv = 1.0f / sum;

                            JVector.Multiply(ref v01, b0, out point);
                            JVector.Multiply(ref v11, b1, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v21, b2, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v31, b3, out temp1);
                            JVector.Add(ref point, ref temp1, out point);

                            JVector.Multiply(ref v02, b0, out temp2);
                            JVector.Add(ref temp2, ref point, out point);
                            JVector.Multiply(ref v12, b1, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v22, b2, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v32, b3, out temp1);
                            JVector.Add(ref point, ref temp1, out point);

                            JVector.Multiply(ref point, inv * 0.5f, out point);
                        }

                        return hit;
                    }

                    JVector.Cross(ref v4, ref v0, out temp1);
                    float dot = JVector.Dot(ref temp1, ref v1);

                    if (dot >= 0.0f)
                    {
                        dot = JVector.Dot(ref temp1, ref v2);

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
                        dot = JVector.Dot(ref temp1, ref v3);

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