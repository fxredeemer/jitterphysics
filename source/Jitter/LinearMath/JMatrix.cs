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

        public static readonly JMatrix Identity = new JMatrix(
                m11: 1.0f,
                m12: 0f,
                m13: 0f,
                m21: 0f,
                m22: 1.0f,
                m23: 0f,
                m31: 0f,
                m32: 0f,
                m33: 1.0f);

        public static readonly JMatrix Zero = new JMatrix();

        public static JMatrix CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            JQuaternion.CreateFromYawPitchRoll(yaw, pitch, roll, out var quaternion);
            CreateFromQuaternion(in quaternion, out var matrix);
            return matrix;
        }

        public static JMatrix CreateRotationX(float radians)
        {
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);

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
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);

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
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
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
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);
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
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);

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
            var num2 = (float)Math.Cos(radians);
            var num = (float)Math.Sin(radians);

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

        public JMatrix(float m11, float m22, float m33)
        {
            M11 = m11;
            M12 = 0f;
            M13 = 0f;
            M22 = m22;
            M21 = 0f;
            M23 = 0f;
            M33 = m33;
            M31 = 0f;
            M32 = 0f;
        }

        public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        {
            Multiply(matrix1, matrix2, out var result);
            return result;
        }

        public static void Multiply(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
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
            Add(in matrix1, in matrix2, out var result);
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
            Inverse(in matrix, out var result);
            return result;
        }

        public float Determinant()
        {
            return (M11 * M22 * M33) + (M12 * M23 * M31) + (M13 * M21 * M32)
                   - (M31 * M22 * M13) - (M32 * M23 * M11) - (M33 * M21 * M12);
        }

        public static void Invert(in JMatrix matrix, out JMatrix result)
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

            result = new JMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
        }

        public static void Inverse(in JMatrix matrix, out JMatrix result)
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


            result = new JMatrix(
                num11 / det,
                num12 / det,
                num13 / det,
                num21 / det,
                num22 / det,
                num23 / det,
                num31 / det,
                num32 / det,
                num33 / det);
        }

        public static JMatrix Multiply(JMatrix matrix1, float scaleFactor)
        {
            Multiply(in matrix1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JMatrix matrix1, float scaleFactor, out JMatrix result)
        {
            result = new JMatrix(
                m11: matrix1.M11 * scaleFactor,
                m12: matrix1.M12 * scaleFactor,
                m13: matrix1.M13 * scaleFactor,
                m21: matrix1.M21 * scaleFactor,
                m22: matrix1.M22 * scaleFactor,
                m23: matrix1.M23 * scaleFactor,
                m31: matrix1.M31 * scaleFactor,
                m32: matrix1.M32 * scaleFactor,
                m33: matrix1.M33 * scaleFactor);
        }

        public static JMatrix CreateFromQuaternion(JQuaternion quaternion)
        {
            CreateFromQuaternion(in quaternion, out var result);
            return result;
        }

        public static void CreateFromQuaternion(in JQuaternion quaternion, out JMatrix result)
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

            result = new JMatrix(
                m11: 1f - (2f * (num8 + num7)),
                m12: 2f * (num6 + num5),
                m13: 2f * (num4 - num3),
                m21: 2f * (num6 - num5),
                m22: 1f - (2f * (num7 + num9)),
                m23: 2f * (num2 + num),
                m31: 2f * (num4 + num3),
                m32: 2f * (num2 - num),
                m33: 1f - (2f * (num8 + num9)));
        }

        public static JMatrix Transpose(JMatrix matrix)
        {
            Transpose(in matrix, out var result);
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
            Multiply(in value1, in value2, out var result);
            return result;
        }

        public float Trace()
        {
            return M11 + M22 + M33;
        }

        public static JMatrix operator +(JMatrix value1, JMatrix value2)
        {
            Add(value1, in value2, out var result);
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
            CreateFromAxisAngle(in axis, angle, out var result);
            return result;
        }
    }
}
