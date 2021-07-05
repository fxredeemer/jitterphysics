using System;

namespace Jitter.LinearMath
{
    public readonly struct JQuaternion
    {
        public float X { get; }
        public float Y { get; }
        public float Z { get; }
        public float W { get; }

        public JQuaternion(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static JQuaternion Add(in JQuaternion quaternion1, in JQuaternion quaternion2)
        {
            Add(quaternion1, quaternion2, out var result);
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

            result = new JQuaternion(
                x: (num * num4 * num5) + (num2 * num3 * num6),
                y: (num2 * num3 * num5) - (num * num4 * num6),
                z: (num * num3 * num6) - (num2 * num4 * num5),
                w: (num * num3 * num5) + (num2 * num4 * num6));
        }

        public static void Add(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
        {
            result = new JQuaternion(
                x: quaternion1.X + quaternion2.X,
                y: quaternion1.Y + quaternion2.Y,
                z: quaternion1.Z + quaternion2.Z,
                w: quaternion1.W + quaternion2.W);
        }

        public static JQuaternion Conjugate(in JQuaternion value)
        {
            return new JQuaternion(
                x: -value.X,
                y: -value.Y,
                z: -value.Z,
                w: value.W);
        }

        public static JQuaternion Subtract(in JQuaternion quaternion1, in JQuaternion quaternion2)
        {
            Subtract(quaternion1, quaternion2, out var result);
            return result;
        }

        public static void Subtract(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
        {
            result = new JQuaternion(
                x: quaternion1.X - quaternion2.X,
                y: quaternion1.Y - quaternion2.Y,
                z: quaternion1.Z - quaternion2.Z,
                w: quaternion1.W - quaternion2.W);
        }

        public static JQuaternion Multiply(in JQuaternion quaternion1, in JQuaternion quaternion2)
        {
            Multiply(quaternion1, quaternion2, out var result);
            return result;
        }

        public static void Multiply(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
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

            result = new JQuaternion(
                x: (x * num) + (num4 * w) + num12,
                y: (y * num) + (num3 * w) + num11,
                z: (z * num) + (num2 * w) + num10,
                w: (w * num) - num9);
        }

        public static JQuaternion Multiply(in JQuaternion quaternion1, float scaleFactor)
        {
            Multiply(quaternion1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JQuaternion quaternion1, float scaleFactor, out JQuaternion result)
        {
            result = new JQuaternion(
                x: quaternion1.X * scaleFactor,
                y: quaternion1.Y * scaleFactor,
                z: quaternion1.Z * scaleFactor,
                w: quaternion1.W * scaleFactor);
        }

        public JQuaternion Normalize()
        {
            var num2 = (X * X) + (Y * Y) + (Z * Z) + (W * W);
            var num = 1f / (JMath.Sqrt(num2));

            return new JQuaternion(
                x: X * num,
                y: X * num,
                z: X * num,
                w: X * num);
        }

        public static JQuaternion CreateFromMatrix(in JMatrix matrix)
        {
            CreateFromMatrix(matrix, out var result);
            return result;
        }

        public static void CreateFromMatrix(in JMatrix matrix, out JQuaternion result)
        {
            var num8 = matrix.M11 + matrix.M22 + matrix.M33;

            float x, y, z, w;

            if (num8 > 0f)
            {
                var num = JMath.Sqrt(num8 + 1f);
                w = num * 0.5f;
                num = 0.5f / num;
                x = (matrix.M23 - matrix.M32) * num;
                y = (matrix.M31 - matrix.M13) * num;
                z = (matrix.M12 - matrix.M21) * num;
            }
            else if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            {
                var num7 = JMath.Sqrt(1f + matrix.M11 - matrix.M22 - matrix.M33);
                var num4 = 0.5f / num7;
                x = 0.5f * num7;
                y = (matrix.M12 + matrix.M21) * num4;
                z = (matrix.M13 + matrix.M31) * num4;
                w = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33)
            {
                var num6 = JMath.Sqrt(1f + matrix.M22 - matrix.M11 - matrix.M33);
                var num3 = 0.5f / num6;
                x = (matrix.M21 + matrix.M12) * num3;
                y = 0.5f * num6;
                z = (matrix.M32 + matrix.M23) * num3;
                w = (matrix.M31 - matrix.M13) * num3;
            }
            else
            {
                var num5 = JMath.Sqrt(1f + matrix.M33 - matrix.M11 - matrix.M22);
                var num2 = 0.5f / num5;
                x = (matrix.M31 + matrix.M13) * num2;
                y = (matrix.M32 + matrix.M23) * num2;
                z = 0.5f * num5;
                w = (matrix.M12 - matrix.M21) * num2;
            }

            result = new JQuaternion(x, y, z, w);
        }

        public static JQuaternion operator *(in JQuaternion value1, in JQuaternion value2)
        {
            Multiply(value1, value2, out var result);
            return result;
        }

        public static JQuaternion operator +(in JQuaternion value1, in JQuaternion value2)
        {
            Add(value1, value2, out var result);
            return result;
        }

        public static JQuaternion operator -(in JQuaternion value1, in JQuaternion value2)
        {
            Subtract(value1, value2, out var result);
            return result;
        }
    }
}
