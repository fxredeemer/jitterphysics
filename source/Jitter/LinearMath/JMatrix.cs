using System;

namespace Jitter.LinearMath
{
    public readonly struct JMatrix
    {
        public readonly float M11;
        public readonly float M12;
        public readonly float M13;
        public readonly float M21;
        public readonly float M22;
        public readonly float M23;
        public readonly float M31;
        public readonly float M32;
        public readonly float M33;

        internal static JMatrix InternalIdentity;

        public static readonly JMatrix Identity;
        public static readonly JMatrix Zero;

        static JMatrix()
        {
            Zero = new JMatrix();

            Identity = new JMatrix
            {
                M11 = 1.0f,
                M22 = 1.0f,
                M33 = 1.0f
            };

            InternalIdentity = Identity;
        }

        public static JMatrix CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            JQuaternion.CreateFromYawPitchRoll(yaw, pitch, roll, out var quaternion);
            CreateFromQuaternion(ref quaternion, out var matrix);
            return matrix;
        }

        public static JMatrix CreateRotationX(float radians)
        {
            JMatrix matrix;
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            matrix.M11 = 1f;
            matrix.M12 = 0f;
            matrix.M13 = 0f;
            matrix.M21 = 0f;
            matrix.M22 = num2;
            matrix.M23 = num;
            matrix.M31 = 0f;
            matrix.M32 = -num;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationX(float radians, out JMatrix result)
        {
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            result.M11 = 1f;
            result.M12 = 0f;
            result.M13 = 0f;
            result.M21 = 0f;
            result.M22 = num2;
            result.M23 = num;
            result.M31 = 0f;
            result.M32 = -num;
            result.M33 = num2;
        }

        public static JMatrix CreateRotationY(float radians)
        {
            JMatrix matrix;
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            matrix.M11 = num2;
            matrix.M12 = 0f;
            matrix.M13 = -num;
            matrix.M21 = 0f;
            matrix.M22 = 1f;
            matrix.M23 = 0f;
            matrix.M31 = num;
            matrix.M32 = 0f;
            matrix.M33 = num2;
            return matrix;
        }

        public static void CreateRotationY(float radians, out JMatrix result)
        {
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            result.M11 = num2;
            result.M12 = 0f;
            result.M13 = -num;
            result.M21 = 0f;
            result.M22 = 1f;
            result.M23 = 0f;
            result.M31 = num;
            result.M32 = 0f;
            result.M33 = num2;
        }

        public static JMatrix CreateRotationZ(float radians)
        {
            JMatrix matrix;
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            matrix.M11 = num2;
            matrix.M12 = num;
            matrix.M13 = 0f;
            matrix.M21 = -num;
            matrix.M22 = num2;
            matrix.M23 = 0f;
            matrix.M31 = 0f;
            matrix.M32 = 0f;
            matrix.M33 = 1f;
            return matrix;
        }

        public static void CreateRotationZ(float radians, out JMatrix result)
        {
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
            result.M11 = num2;
            result.M12 = num;
            result.M13 = 0f;
            result.M21 = -num;
            result.M22 = num2;
            result.M23 = 0f;
            result.M31 = 0f;
            result.M32 = 0f;
            result.M33 = 1f;
        }

        public JMatrix(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
        {
            M11 = m11;
            M12 = m12;
            M13 = m13;
            M21 = m21;
            M22 = m22;
            M23 = m23;
            M31 = m31;
            M32 = m32;
            M33 = m33;
        }

        public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        {
            Multiply(ref matrix1, ref matrix2, out var result);
            return result;
        }

        public static void Multiply(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            var num0 = (matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21) + (matrix1.M13 * matrix2.M31);
            var num1 = (matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22) + (matrix1.M13 * matrix2.M32);
            var num2 = (matrix1.M11 * matrix2.M13) + (matrix1.M12 * matrix2.M23) + (matrix1.M13 * matrix2.M33);
            var num3 = (matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21) + (matrix1.M23 * matrix2.M31);
            var num4 = (matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22) + (matrix1.M23 * matrix2.M32);
            var num5 = (matrix1.M21 * matrix2.M13) + (matrix1.M22 * matrix2.M23) + (matrix1.M23 * matrix2.M33);
            var num6 = (matrix1.M31 * matrix2.M11) + (matrix1.M32 * matrix2.M21) + (matrix1.M33 * matrix2.M31);
            var num7 = (matrix1.M31 * matrix2.M12) + (matrix1.M32 * matrix2.M22) + (matrix1.M33 * matrix2.M32);
            var num8 = (matrix1.M31 * matrix2.M13) + (matrix1.M32 * matrix2.M23) + (matrix1.M33 * matrix2.M33);

            result.M11 = num0;
            result.M12 = num1;
            result.M13 = num2;
            result.M21 = num3;
            result.M22 = num4;
            result.M23 = num5;
            result.M31 = num6;
            result.M32 = num7;
            result.M33 = num8;
        }

        public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        {
            Add(ref matrix1, ref matrix2, out var result);
            return result;
        }

        public static void Add(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            result.M11 = matrix1.M11 + matrix2.M11;
            result.M12 = matrix1.M12 + matrix2.M12;
            result.M13 = matrix1.M13 + matrix2.M13;
            result.M21 = matrix1.M21 + matrix2.M21;
            result.M22 = matrix1.M22 + matrix2.M22;
            result.M23 = matrix1.M23 + matrix2.M23;
            result.M31 = matrix1.M31 + matrix2.M31;
            result.M32 = matrix1.M32 + matrix2.M32;
            result.M33 = matrix1.M33 + matrix2.M33;
        }

        public static JMatrix Inverse(JMatrix matrix)
        {
            Inverse(ref matrix, out var result);
            return result;
        }

        public float Determinant()
        {
            return (M11 * M22 * M33) + (M12 * M23 * M31) + (M13 * M21 * M32)
                   - (M31 * M22 * M13) - (M32 * M23 * M11) - (M33 * M21 * M12);
        }

        public static void Invert(ref JMatrix matrix, out JMatrix result)
        {
            var determinantInverse = 1 / matrix.Determinant();
            var m11 = ((matrix.M22 * matrix.M33) - (matrix.M23 * matrix.M32)) * determinantInverse;
            var m12 = ((matrix.M13 * matrix.M32) - (matrix.M33 * matrix.M12)) * determinantInverse;
            var m13 = ((matrix.M12 * matrix.M23) - (matrix.M22 * matrix.M13)) * determinantInverse;

            var m21 = ((matrix.M23 * matrix.M31) - (matrix.M21 * matrix.M33)) * determinantInverse;
            var m22 = ((matrix.M11 * matrix.M33) - (matrix.M13 * matrix.M31)) * determinantInverse;
            var m23 = ((matrix.M13 * matrix.M21) - (matrix.M11 * matrix.M23)) * determinantInverse;

            var m31 = ((matrix.M21 * matrix.M32) - (matrix.M22 * matrix.M31)) * determinantInverse;
            var m32 = ((matrix.M12 * matrix.M31) - (matrix.M11 * matrix.M32)) * determinantInverse;
            var m33 = ((matrix.M11 * matrix.M22) - (matrix.M12 * matrix.M21)) * determinantInverse;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;

            result.M31 = m31;
            result.M32 = m32;
            result.M33 = m33;
        }

        public static void Inverse(ref JMatrix matrix, out JMatrix result)
        {
            var det = (matrix.M11 * matrix.M22 * matrix.M33)
                - (matrix.M11 * matrix.M23 * matrix.M32)
                - (matrix.M12 * matrix.M21 * matrix.M33)
                + (matrix.M12 * matrix.M23 * matrix.M31)
                + (matrix.M13 * matrix.M21 * matrix.M32)
                - (matrix.M13 * matrix.M22 * matrix.M31);

            var num11 = (matrix.M22 * matrix.M33) - (matrix.M23 * matrix.M32);
            var num12 = (matrix.M13 * matrix.M32) - (matrix.M12 * matrix.M33);
            var num13 = (matrix.M12 * matrix.M23) - (matrix.M22 * matrix.M13);

            var num21 = (matrix.M23 * matrix.M31) - (matrix.M33 * matrix.M21);
            var num22 = (matrix.M11 * matrix.M33) - (matrix.M31 * matrix.M13);
            var num23 = (matrix.M13 * matrix.M21) - (matrix.M23 * matrix.M11);

            var num31 = (matrix.M21 * matrix.M32) - (matrix.M31 * matrix.M22);
            var num32 = (matrix.M12 * matrix.M31) - (matrix.M32 * matrix.M11);
            var num33 = (matrix.M11 * matrix.M22) - (matrix.M21 * matrix.M12);

            result.M11 = num11 / det;
            result.M12 = num12 / det;
            result.M13 = num13 / det;
            result.M21 = num21 / det;
            result.M22 = num22 / det;
            result.M23 = num23 / det;
            result.M31 = num31 / det;
            result.M32 = num32 / det;
            result.M33 = num33 / det;
        }

        public static JMatrix Multiply(JMatrix matrix1, float scaleFactor)
        {
            Multiply(ref matrix1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(ref JMatrix matrix1, float scaleFactor, out JMatrix result)
        {
            var num = scaleFactor;
            result.M11 = matrix1.M11 * num;
            result.M12 = matrix1.M12 * num;
            result.M13 = matrix1.M13 * num;
            result.M21 = matrix1.M21 * num;
            result.M22 = matrix1.M22 * num;
            result.M23 = matrix1.M23 * num;
            result.M31 = matrix1.M31 * num;
            result.M32 = matrix1.M32 * num;
            result.M33 = matrix1.M33 * num;
        }

        public static JMatrix CreateFromQuaternion(JQuaternion quaternion)
        {
            CreateFromQuaternion(ref quaternion, out var result);
            return result;
        }

        public static void CreateFromQuaternion(ref JQuaternion quaternion, out JMatrix result)
        {
            var num9 = quaternion.X * quaternion.X;
            var num8 = quaternion.Y * quaternion.Y;
            var num7 = quaternion.Z * quaternion.Z;
            var num6 = quaternion.X * quaternion.Y;
            var num5 = quaternion.Z * quaternion.W;
            var num4 = quaternion.Z * quaternion.X;
            var num3 = quaternion.Y * quaternion.W;
            var num2 = quaternion.Y * quaternion.Z;
            var num = quaternion.X * quaternion.W;
            result.M11 = 1f - (2f * (num8 + num7));
            result.M12 = 2f * (num6 + num5);
            result.M13 = 2f * (num4 - num3);
            result.M21 = 2f * (num6 - num5);
            result.M22 = 1f - (2f * (num7 + num9));
            result.M23 = 2f * (num2 + num);
            result.M31 = 2f * (num4 + num3);
            result.M32 = 2f * (num2 - num);
            result.M33 = 1f - (2f * (num8 + num9));
        }

        public static JMatrix Transpose(JMatrix matrix)
        {
            Transpose(ref matrix, out var result);
            return result;
        }

        public static void Transpose(ref JMatrix matrix, out JMatrix result)
        {
            result.M11 = matrix.M11;
            result.M12 = matrix.M21;
            result.M13 = matrix.M31;
            result.M21 = matrix.M12;
            result.M22 = matrix.M22;
            result.M23 = matrix.M32;
            result.M31 = matrix.M13;
            result.M32 = matrix.M23;
            result.M33 = matrix.M33;
        }

        public static JMatrix operator *(JMatrix value1, JMatrix value2)
        {
            Multiply(ref value1, ref value2, out var result);
            return result;
        }

        public float Trace()
        {
            return M11 + M22 + M33;
        }

        public static JMatrix operator +(JMatrix value1, JMatrix value2)
        {
            Add(ref value1, ref value2, out var result);
            return result;
        }

        public static JMatrix operator -(JMatrix value1, JMatrix value2)
        {
            Multiply(ref value2, -1.0f, out value2);
            Add(ref value1, ref value2, out var result);
            return result;
        }

        public static void CreateFromAxisAngle(ref JVector axis, float angle, out JMatrix result)
        {
            var x = axis.X;
            var y = axis.Y;
            var z = axis.Z;
            var num2 = (float)Math.Sin(angle);
            var num = (float)Math.Cos(angle);
            var num11 = x * x;
            var num10 = y * y;
            var num9 = z * z;
            var num8 = x * y;
            var num7 = x * z;
            var num6 = y * z;
            result.M11 = num11 + (num * (1f - num11));
            result.M12 = num8 - (num * num8) + (num2 * z);
            result.M13 = num7 - (num * num7) - (num2 * y);
            result.M21 = num8 - (num * num8) - (num2 * z);
            result.M22 = num10 + (num * (1f - num10));
            result.M23 = num6 - (num * num6) + (num2 * x);
            result.M31 = num7 - (num * num7) + (num2 * y);
            result.M32 = num6 - (num * num6) - (num2 * x);
            result.M33 = num9 + (num * (1f - num9));
        }

        public static JMatrix CreateFromAxisAngle(JVector axis, float angle)
        {
            CreateFromAxisAngle(ref axis, angle, out var result);
            return result;
        }
    }
}
