using System;

namespace Jitter.LinearMath
{
    public struct JMatrix
    {
        public float M11;
        public float M12;
        public float M13;
        public float M21;
        public float M22;
        public float M23;
        public float M31;
        public float M32;
        public float M33;

        public static readonly JMatrix Identity = new JMatrix { M11 = 1.0f, M22 = 1.0f, M33 = 1.0f };

        internal static JMatrix InternalIdentity = Identity;

        public static readonly JMatrix Zero = new JMatrix();

        public static JMatrix CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            JQuaternion.CreateFromYawPitchRoll(yaw, pitch, roll, out var quaternion);
            CreateFromQuaternion(quaternion, out var matrix);
            return matrix;
        }

        public static JMatrix CreateRotationX(float radians)
        {
            JMatrix matrix;
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            float num2 = (float)Math.Cos(radians);
            float num = (float)Math.Sin(radians);
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
            Multiply(matrix1, matrix2, out var result);
            return result;
        }

        public static void Multiply(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
        {
            float num0 = (matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21) + (matrix1.M13 * matrix2.M31);
            float num1 = (matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22) + (matrix1.M13 * matrix2.M32);
            float num2 = (matrix1.M11 * matrix2.M13) + (matrix1.M12 * matrix2.M23) + (matrix1.M13 * matrix2.M33);
            float num3 = (matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21) + (matrix1.M23 * matrix2.M31);
            float num4 = (matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22) + (matrix1.M23 * matrix2.M32);
            float num5 = (matrix1.M21 * matrix2.M13) + (matrix1.M22 * matrix2.M23) + (matrix1.M23 * matrix2.M33);
            float num6 = (matrix1.M31 * matrix2.M11) + (matrix1.M32 * matrix2.M21) + (matrix1.M33 * matrix2.M31);
            float num7 = (matrix1.M31 * matrix2.M12) + (matrix1.M32 * matrix2.M22) + (matrix1.M33 * matrix2.M32);
            float num8 = (matrix1.M31 * matrix2.M13) + (matrix1.M32 * matrix2.M23) + (matrix1.M33 * matrix2.M33);

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
            Add(matrix1, matrix2, out var result);
            return result;
        }

        public static void Add(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
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
            Inverse(matrix, out var result);
            return result;
        }

        public float Determinant()
        {
            return (M11 * M22 * M33) + (M12 * M23 * M31) + (M13 * M21 * M32)
                   - (M31 * M22 * M13) - (M32 * M23 * M11) - (M33 * M21 * M12);
        }

        public static void Invert(in JMatrix matrix, out JMatrix result)
        {
            float determinantInverse = 1 / matrix.Determinant();
            float m11 = ((matrix.M22 * matrix.M33) - (matrix.M23 * matrix.M32)) * determinantInverse;
            float m12 = ((matrix.M13 * matrix.M32) - (matrix.M33 * matrix.M12)) * determinantInverse;
            float m13 = ((matrix.M12 * matrix.M23) - (matrix.M22 * matrix.M13)) * determinantInverse;

            float m21 = ((matrix.M23 * matrix.M31) - (matrix.M21 * matrix.M33)) * determinantInverse;
            float m22 = ((matrix.M11 * matrix.M33) - (matrix.M13 * matrix.M31)) * determinantInverse;
            float m23 = ((matrix.M13 * matrix.M21) - (matrix.M11 * matrix.M23)) * determinantInverse;

            float m31 = ((matrix.M21 * matrix.M32) - (matrix.M22 * matrix.M31)) * determinantInverse;
            float m32 = ((matrix.M12 * matrix.M31) - (matrix.M11 * matrix.M32)) * determinantInverse;
            float m33 = ((matrix.M11 * matrix.M22) - (matrix.M12 * matrix.M21)) * determinantInverse;

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

        public static void Inverse(in JMatrix matrix, out JMatrix result)
        {
            float det = (matrix.M11 * matrix.M22 * matrix.M33)
                - (matrix.M11 * matrix.M23 * matrix.M32)
                - (matrix.M12 * matrix.M21 * matrix.M33)
                + (matrix.M12 * matrix.M23 * matrix.M31)
                + (matrix.M13 * matrix.M21 * matrix.M32)
                - (matrix.M13 * matrix.M22 * matrix.M31);

            float num11 = (matrix.M22 * matrix.M33) - (matrix.M23 * matrix.M32);
            float num12 = (matrix.M13 * matrix.M32) - (matrix.M12 * matrix.M33);
            float num13 = (matrix.M12 * matrix.M23) - (matrix.M22 * matrix.M13);

            float num21 = (matrix.M23 * matrix.M31) - (matrix.M33 * matrix.M21);
            float num22 = (matrix.M11 * matrix.M33) - (matrix.M31 * matrix.M13);
            float num23 = (matrix.M13 * matrix.M21) - (matrix.M23 * matrix.M11);

            float num31 = (matrix.M21 * matrix.M32) - (matrix.M31 * matrix.M22);
            float num32 = (matrix.M12 * matrix.M31) - (matrix.M32 * matrix.M11);
            float num33 = (matrix.M11 * matrix.M22) - (matrix.M21 * matrix.M12);

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
            Multiply(matrix1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JMatrix matrix1, float scaleFactor, out JMatrix result)
        {
            float num = scaleFactor;
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
            CreateFromQuaternion(quaternion, out var result);
            return result;
        }

        public static void CreateFromQuaternion(in JQuaternion quaternion, out JMatrix result)
        {
            float num9 = quaternion.X * quaternion.X;
            float num8 = quaternion.Y * quaternion.Y;
            float num7 = quaternion.Z * quaternion.Z;
            float num6 = quaternion.X * quaternion.Y;
            float num5 = quaternion.Z * quaternion.W;
            float num4 = quaternion.Z * quaternion.X;
            float num3 = quaternion.Y * quaternion.W;
            float num2 = quaternion.Y * quaternion.Z;
            float num = quaternion.X * quaternion.W;

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
            Transpose(matrix, out var result);
            return result;
        }

        public static void Transpose(in JMatrix matrix, out JMatrix result)
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
            Multiply(value1, value2, out var result);
            return result;
        }

        public float Trace()
        {
            return M11 + M22 + M33;
        }

        public static JMatrix operator +(JMatrix value1, JMatrix value2)
        {
            Add(value1, value2, out var result);
            return result;
        }

        public static JMatrix operator -(JMatrix value1, JMatrix value2)
        {
            Multiply(value2, -1.0f, out value2);
            Add(value1, value2, out var result);
            return result;
        }

        public static void CreateFromAxisAngle(in JVector axis, float angle, out JMatrix result)
        {
            float x = axis.X;
            float y = axis.Y;
            float z = axis.Z;
            float num2 = (float)Math.Sin(angle);
            float num = (float)Math.Cos(angle);
            float num11 = x * x;
            float num10 = y * y;
            float num9 = z * z;
            float num8 = x * y;
            float num7 = x * z;
            float num6 = y * z;
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
            CreateFromAxisAngle(axis, angle, out var result);
            return result;
        }
    }
}
