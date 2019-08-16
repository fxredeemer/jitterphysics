using Jitter.LinearMath;
using Microsoft.Xna.Framework;

namespace JitterDemo
{
    public sealed class Conversion
    {
        public static JVector ToJitterVector(Vector3 vector)
        {
            return new JVector(vector.X, vector.Y, vector.Z);
        }

        public static Matrix ToXNAMatrix(JMatrix matrix)
        {
            return new Matrix(
                matrix.M11,
                matrix.M12,
                matrix.M13,
                0.0f,
                matrix.M21,
                matrix.M22,
                matrix.M23,
                0.0f,
                matrix.M31,
                matrix.M32,
                matrix.M33,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                1.0f);
        }

        public static JMatrix ToJitterMatrix(Matrix matrix)
        {
            return new JMatrix
            {
                M11 = matrix.M11,
                M12 = matrix.M12,
                M13 = matrix.M13,
                M21 = matrix.M21,
                M22 = matrix.M22,
                M23 = matrix.M23,
                M31 = matrix.M31,
                M32 = matrix.M32,
                M33 = matrix.M33
            };
        }

        public static Vector3 ToXNAVector(JVector vector)
        {
            return new Vector3(vector.X, vector.Y, vector.Z);
        }
    }
}
