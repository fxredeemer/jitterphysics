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
            Add(quaternion1, quaternion2, out var result);
            return result;
        }

        public static void CreateFromYawPitchRoll(float yaw, float pitch, float roll, out JQuaternion result)
        {
            float num9 = roll * 0.5f;
            float num6 = (float)Math.Sin(num9);
            float num5 = (float)Math.Cos(num9);
            float num8 = pitch * 0.5f;
            float num4 = (float)Math.Sin(num8);
            float num3 = (float)Math.Cos(num8);
            float num7 = yaw * 0.5f;
            float num2 = (float)Math.Sin(num7);
            float num = (float)Math.Cos(num7);
            result.X = (num * num4 * num5) + (num2 * num3 * num6);
            result.Y = (num2 * num3 * num5) - (num * num4 * num6);
            result.Z = (num * num3 * num6) - (num2 * num4 * num5);
            result.W = (num * num3 * num5) + (num2 * num4 * num6);
        }

        public static void Add(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
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
            Subtract(quaternion1, quaternion2, out var result);
            return result;
        }

        public static void Subtract(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
        {
            result.X = quaternion1.X - quaternion2.X;
            result.Y = quaternion1.Y - quaternion2.Y;
            result.Z = quaternion1.Z - quaternion2.Z;
            result.W = quaternion1.W - quaternion2.W;
        }

        public static JQuaternion Multiply(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            Multiply(quaternion1, quaternion2, out var result);
            return result;
        }

        public static void Multiply(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
        {
            float x = quaternion1.X;
            float y = quaternion1.Y;
            float z = quaternion1.Z;
            float w = quaternion1.W;
            float num4 = quaternion2.X;
            float num3 = quaternion2.Y;
            float num2 = quaternion2.Z;
            float num = quaternion2.W;
            float num12 = (y * num2) - (z * num3);
            float num11 = (z * num4) - (x * num2);
            float num10 = (x * num3) - (y * num4);
            float num9 = (x * num4) + (y * num3) + (z * num2);
            result.X = (x * num) + (num4 * w) + num12;
            result.Y = (y * num) + (num3 * w) + num11;
            result.Z = (z * num) + (num2 * w) + num10;
            result.W = (w * num) - num9;
        }

        public static JQuaternion Multiply(JQuaternion quaternion1, float scaleFactor)
        {
            Multiply(quaternion1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JQuaternion quaternion1, float scaleFactor, out JQuaternion result)
        {
            result.X = quaternion1.X * scaleFactor;
            result.Y = quaternion1.Y * scaleFactor;
            result.Z = quaternion1.Z * scaleFactor;
            result.W = quaternion1.W * scaleFactor;
        }

        public void Normalize()
        {
            float num2 = (X * X) + (Y * Y) + (Z * Z) + (W * W);
            float num = 1f / JMath.Sqrt(num2);
            X *= num;
            Y *= num;
            Z *= num;
            W *= num;
        }

        public static JQuaternion CreateFromMatrix(JMatrix matrix)
        {
            CreateFromMatrix(matrix, out var result);
            return result;
        }

        public static void CreateFromMatrix(in JMatrix matrix, out JQuaternion result)
        {
            float num8 = matrix.M11 + matrix.M22 + matrix.M33;
            if (num8 > 0f)
            {
                float num = JMath.Sqrt(num8 + 1f);
                result.W = num * 0.5f;
                num = 0.5f / num;
                result.X = (matrix.M23 - matrix.M32) * num;
                result.Y = (matrix.M31 - matrix.M13) * num;
                result.Z = (matrix.M12 - matrix.M21) * num;
            }
            else if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            {
                float num7 = JMath.Sqrt(1f + matrix.M11 - matrix.M22 - matrix.M33);
                float num4 = 0.5f / num7;
                result.X = 0.5f * num7;
                result.Y = (matrix.M12 + matrix.M21) * num4;
                result.Z = (matrix.M13 + matrix.M31) * num4;
                result.W = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33)
            {
                float num6 = JMath.Sqrt(1f + matrix.M22 - matrix.M11 - matrix.M33);
                float num3 = 0.5f / num6;
                result.X = (matrix.M21 + matrix.M12) * num3;
                result.Y = 0.5f * num6;
                result.Z = (matrix.M32 + matrix.M23) * num3;
                result.W = (matrix.M31 - matrix.M13) * num3;
            }
            else
            {
                float num5 = JMath.Sqrt(1f + matrix.M33 - matrix.M11 - matrix.M22);
                float num2 = 0.5f / num5;
                result.X = (matrix.M31 + matrix.M13) * num2;
                result.Y = (matrix.M32 + matrix.M23) * num2;
                result.Z = 0.5f * num5;
                result.W = (matrix.M12 - matrix.M21) * num2;
            }
        }

        public static JQuaternion operator *(JQuaternion value1, JQuaternion value2)
        {
            Multiply(value1, value2, out var result);
            return result;
        }

        public static JQuaternion operator +(JQuaternion value1, JQuaternion value2)
        {
            Add(value1, value2, out var result);
            return result;
        }

        public static JQuaternion operator -(JQuaternion value1, JQuaternion value2)
        {
            Subtract(value1, value2, out var result);
            return result;
        }
    }
}
