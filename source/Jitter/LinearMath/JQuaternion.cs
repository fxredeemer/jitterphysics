using System;

namespace Jitter.LinearMath
{
    public struct JQuaternion
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public JQuaternion(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static JQuaternion Add(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            Add(ref quaternion1, ref quaternion2, out var result);
            return result;
        }

        public static void CreateFromYawPitchRoll(float yaw, float pitch, float roll, out JQuaternion result)
        {
            var num9 = roll * 0.5f;
            var num6 = (float)Math.Sin(num9);
            var num5 = (float)Math.Cos(num9);
            var num8 = pitch * 0.5f;
            var num4 = (float)Math.Sin(num8);
            var num3 = (float)Math.Cos(num8);
            var num7 = yaw * 0.5f;
            var num2 = (float)Math.Sin(num7);
            var num = (float)Math.Cos(num7);
            result.X = (num * num4 * num5) + (num2 * num3 * num6);
            result.Y = (num2 * num3 * num5) - (num * num4 * num6);
            result.Z = (num * num3 * num6) - (num2 * num4 * num5);
            result.W = (num * num3 * num5) + (num2 * num4 * num6);
        }

        public static void Add(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            result.X = quaternion1.X + quaternion2.X;
            result.Y = quaternion1.Y + quaternion2.Y;
            result.Z = quaternion1.Z + quaternion2.Z;
            result.W = quaternion1.W + quaternion2.W;
        }

        public static JQuaternion Conjugate(JQuaternion value)
        {
            JQuaternion quaternion;
            quaternion.X = -value.X;
            quaternion.Y = -value.Y;
            quaternion.Z = -value.Z;
            quaternion.W = value.W;
            return quaternion;
        }

        public static JQuaternion Subtract(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            Subtract(ref quaternion1, ref quaternion2, out var result);
            return result;
        }

        public static void Subtract(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            result.X = quaternion1.X - quaternion2.X;
            result.Y = quaternion1.Y - quaternion2.Y;
            result.Z = quaternion1.Z - quaternion2.Z;
            result.W = quaternion1.W - quaternion2.W;
        }

        public static JQuaternion Multiply(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            Multiply(ref quaternion1, ref quaternion2, out var result);
            return result;
        }

        public static void Multiply(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            var x = quaternion1.X;
            var y = quaternion1.Y;
            var z = quaternion1.Z;
            var w = quaternion1.W;
            var num4 = quaternion2.X;
            var num3 = quaternion2.Y;
            var num2 = quaternion2.Z;
            var num = quaternion2.W;
            var num12 = (y * num2) - (z * num3);
            var num11 = (z * num4) - (x * num2);
            var num10 = (x * num3) - (y * num4);
            var num9 = (x * num4) + (y * num3) + (z * num2);
            result.X = (x * num) + (num4 * w) + num12;
            result.Y = (y * num) + (num3 * w) + num11;
            result.Z = (z * num) + (num2 * w) + num10;
            result.W = (w * num) - num9;
        }

        public static JQuaternion Multiply(JQuaternion quaternion1, float scaleFactor)
        {
            Multiply(ref quaternion1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(ref JQuaternion quaternion1, float scaleFactor, out JQuaternion result)
        {
            result.X = quaternion1.X * scaleFactor;
            result.Y = quaternion1.Y * scaleFactor;
            result.Z = quaternion1.Z * scaleFactor;
            result.W = quaternion1.W * scaleFactor;
        }

        public void Normalize()
        {
            var num2 = (X * X) + (Y * Y) + (Z * Z) + (W * W);
            var num = 1f / (JMath.Sqrt(num2));
            X *= num;
            Y *= num;
            Z *= num;
            W *= num;
        }

        public static JQuaternion CreateFromMatrix(JMatrix matrix)
        {
            CreateFromMatrix(ref matrix, out var result);
            return result;
        }

        public static void CreateFromMatrix(ref JMatrix matrix, out JQuaternion result)
        {
            var num8 = matrix.M11 + matrix.M22 + matrix.M33;
            if (num8 > 0f)
            {
                var num = JMath.Sqrt(num8 + 1f);
                result.W = num * 0.5f;
                num = 0.5f / num;
                result.X = (matrix.M23 - matrix.M32) * num;
                result.Y = (matrix.M31 - matrix.M13) * num;
                result.Z = (matrix.M12 - matrix.M21) * num;
            }
            else if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            {
                var num7 = JMath.Sqrt(1f + matrix.M11 - matrix.M22 - matrix.M33);
                var num4 = 0.5f / num7;
                result.X = 0.5f * num7;
                result.Y = (matrix.M12 + matrix.M21) * num4;
                result.Z = (matrix.M13 + matrix.M31) * num4;
                result.W = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33)
            {
                var num6 = JMath.Sqrt(1f + matrix.M22 - matrix.M11 - matrix.M33);
                var num3 = 0.5f / num6;
                result.X = (matrix.M21 + matrix.M12) * num3;
                result.Y = 0.5f * num6;
                result.Z = (matrix.M32 + matrix.M23) * num3;
                result.W = (matrix.M31 - matrix.M13) * num3;
            }
            else
            {
                var num5 = JMath.Sqrt(1f + matrix.M33 - matrix.M11 - matrix.M22);
                var num2 = 0.5f / num5;
                result.X = (matrix.M31 + matrix.M13) * num2;
                result.Y = (matrix.M32 + matrix.M23) * num2;
                result.Z = 0.5f * num5;
                result.W = (matrix.M12 - matrix.M21) * num2;
            }
        }

        public static JQuaternion operator *(JQuaternion value1, JQuaternion value2)
        {
            Multiply(ref value1, ref value2, out var result);
            return result;
        }

        public static JQuaternion operator +(JQuaternion value1, JQuaternion value2)
        {
            Add(ref value1, ref value2, out var result);
            return result;
        }

        public static JQuaternion operator -(JQuaternion value1, JQuaternion value2)
        {
            Subtract(ref value1, ref value2, out var result);
            return result;
        }
    }
}
