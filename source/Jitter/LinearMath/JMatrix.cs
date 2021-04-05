using System;

namespace Jitter.LinearMath
{
    public readonly struct JMatrix
    {
        public float M11 { get; }
        public float M12 { get; }
        public float M13 { get; }
        public float M21 { get; }
        public float M22 { get; }
        public float M23 { get; }
        public float M31 { get; }
        public float M32 { get; }
        public float M33 { get; }

        public static readonly JMatrix Identity = FromDiagonal(1f, 1f, 1f); 
        public static readonly JMatrix Zero = new JMatrix();

        public static JMatrix CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            JQuaternion.CreateFromYawPitchRoll(yaw, pitch, roll, out var quaternion);
            CreateFromQuaternion(quaternion, out var matrix);
            return matrix;
        }

        public static JMatrix FromDiagonal(float m11, float m22, float m33)
        {
            return new JMatrix(
                m11: m11,
                m12: 0f,
                m13: 0f,
                m21: 0f,
                m22: m22,
                m23: 0f,
                m31: 0f,
                m32: 0f,
                m33: m33);
        }

        public static JMatrix CreateRotationX(float radians)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);

            return new JMatrix(
                m11: 1f,
                m12: 0f,
                m13: 0f,
                m21: 0f,
                m22: num2,
                m23: num,
                m31: 0f,
                m32: -num,
                m33: num2);
        }

        public static void CreateRotationX(float radians, out JMatrix result)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);

            result = new JMatrix(
                m11: 1f,
                m12: 0f,
                m13: 0f,
                m21: 0f,
                m22: num2,
                m23: num,
                m31: 0f,
                m32: -num,
                m33: num2);
        }

        public static JMatrix CreateRotationY(float radians)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);
            return new JMatrix(
                m11: num2,
                m12: 0f,
                m13: -num,
                m21: 0f,
                m22: 1f,
                m23: 0f,
                m31: num,
                m32: 0f,
                m33: num2);
        }

        public static void CreateRotationY(float radians, out JMatrix result)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);
            result = new JMatrix(
                m11: num2,
                m12: 0f,
                m13: -num,
                m21: 0f,
                m22: 1f,
                m23: 0f,
                m31: num,
                m32: 0f,
                m33: num2);
        }

        public static JMatrix CreateRotationZ(float radians)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);

            return new JMatrix(
                m11: num2,
                m12: num,
                m13: 0f,
                m21: -num,
                m22: num2,
                m23: 0f,
                m31: 0f,
                m32: 0f,
                m33: 1f);
        }

        public static void CreateRotationZ(float radians, out JMatrix result)
        {
            float num2 = JMath.Cos(radians);
            float num = JMath.Sin(radians);

            result = new JMatrix(
                m11: num2,
                m12: num,
                m13: 0f,
                m21: -num,
                m22: num2,
                m23: 0f,
                m31: 0f,
                m32: 0f,
                m33: 1f);
        }

        public JMatrix(
            float m11,
            float m12,
            float m13,
            float m21,
            float m22,
            float m23,
            float m31,
            float m32,
            float m33)
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

            result = new JMatrix(
                m11: num0,
                m12: num1,
                m13: num2,
                m21: num3,
                m22: num4,
                m23: num5,
                m31: num6,
                m32: num7,
                m33: num8);
        }

        public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        {
            Add(matrix1, matrix2, out var result);
            return result;
        }

        public static void Add(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
        {
            result = new JMatrix(
                m11: matrix1.M11 + matrix2.M11,
                m12: matrix1.M12 + matrix2.M12,
                m13: matrix1.M13 + matrix2.M13,
                m21: matrix1.M21 + matrix2.M21,
                m22: matrix1.M22 + matrix2.M22,
                m23: matrix1.M23 + matrix2.M23,
                m31: matrix1.M31 + matrix2.M31,
                m32: matrix1.M32 + matrix2.M32,
                m33: matrix1.M33 + matrix2.M33);
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

            result = new JMatrix(
                m11: m11,
                m12: m12,
                m13: m13,
                m21: m21,
                m22: m22,
                m23: m23,
                m31: m31,
                m32: m32,
                m33: m33);
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

            result = new JMatrix(
                m11: num11 / det,
                m12: num12 / det,
                m13: num13 / det,
                m21: num21 / det,
                m22: num22 / det,
                m23: num23 / det,
                m31: num31 / det,
                m32: num32 / det,
                m33: num33 / det);
        }

        public static JMatrix Multiply(JMatrix matrix1, float scaleFactor)
        {
            Multiply(matrix1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JMatrix matrix1, float scaleFactor, out JMatrix result)
        {
            float num = scaleFactor;
            result = new JMatrix(
                m11: matrix1.M11 * num,
                m12: matrix1.M12 * num,
                m13: matrix1.M13 * num,
                m21: matrix1.M21 * num,
                m22: matrix1.M22 * num,
                m23: matrix1.M23 * num,
                m31: matrix1.M31 * num,
                m32: matrix1.M32 * num,
                m33: matrix1.M33 * num);
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
            float num1 = quaternion.X * quaternion.W;

            result = new JMatrix(
                m11: 1f - (2f * (num8 + num7)),
                m12: 2f * (num6 + num5),
                m13: 2f * (num4 - num3),
                m21: 2f * (num6 - num5),
                m22: 1f - (2f * (num7 + num9)),
                m23: 2f * (num2 + num1),
                m31: 2f * (num4 + num3),
                m32: 2f * (num2 - num1),
                m33: 1f - (2f * (num8 + num9)));
        }

        public static JMatrix Transpose(JMatrix matrix)
        {
            Transpose(matrix, out var result);
            return result;
        }

        public static void Transpose(in JMatrix matrix, out JMatrix result)
        {
            result = new JMatrix(
                m11: matrix.M11,
                m12: matrix.M21,
                m13: matrix.M31,
                m21: matrix.M12,
                m22: matrix.M22,
                m23: matrix.M32,
                m31: matrix.M13,
                m32: matrix.M23,
                m33: matrix.M33);
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
            float num2 = JMath.Sin(angle);
            float num = JMath.Cos(angle);
            float num11 = x * x;
            float num10 = y * y;
            float num9 = z * z;
            float num8 = x * y;
            float num7 = x * z;
            float num6 = y * z;

            result = new JMatrix(
                m11: num11 + (num * (1f - num11)),
                m12: num8 - (num * num8) + (num2 * z),
                m13: num7 - (num * num7) - (num2 * y),
                m21: num8 - (num * num8) - (num2 * z),
                m22: num10 + (num * (1f - num10)),
                m23: num6 - (num * num6) + (num2 * x),
                m31: num7 - (num * num7) + (num2 * y),
                m32: num6 - (num * num6) - (num2 * x),
                m33: num9 + (num * (1f - num9)));
        }

        public static JMatrix CreateFromAxisAngle(JVector axis, float angle)
        {
            CreateFromAxisAngle(axis, angle, out var result);
            return result;
        }
    }
}
