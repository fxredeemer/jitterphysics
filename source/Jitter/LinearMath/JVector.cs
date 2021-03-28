using System;

namespace Jitter.LinearMath
{
    public struct JVector
    {
        public static readonly JVector Zero = new JVector(0, 0, 0);
        public static readonly JVector Left = new JVector(1, 0, 0);
        public static readonly JVector Right = new JVector(-1, 0, 0);
        public static readonly JVector Up = new JVector(0, 1, 0);
        public static readonly JVector Down = new JVector(0, -1, 0);
        public static readonly JVector Backward = new JVector(0, 0, 1);
        public static readonly JVector Forward = new JVector(0, 0, -1);
        public static readonly JVector One = new JVector(1, 1, 1);
        public static readonly JVector MinValue = new JVector(float.MinValue);
        public static readonly JVector MaxValue = new JVector(float.MaxValue);
        internal static JVector InternalZero = new JVector(0, 0, 0);
        internal static JVector Arbitrary = new JVector(1, 1, 1);

        private const float ZeroEpsilonSq = JMath.Epsilon * JMath.Epsilon;

        public float X;
        public float Y;
        public float Z;

        public JVector(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public void Set(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public JVector(float xyz)
        {
            X = xyz;
            Y = xyz;
            Z = xyz;
        }

        public override string ToString()
        {
            return "X=" + X.ToString() + " Y=" + Y.ToString() + " Z=" + Z.ToString();
        }

        public override bool Equals(object obj)
        {
            if (!(obj is JVector other))
            {
                return false;
            }

            return (X == other.X)
                && (Y == other.Y)
                && (Z == other.Z);
        }

        public static bool operator ==(JVector value1, JVector value2)
        {
            return (value1.X == value2.X)
                && (value1.Y == value2.Y)
                && (value1.Z == value2.Z);
        }

        public static bool operator !=(JVector value1, JVector value2)
        {
            if ((value1.X == value2.X) && (value1.Y == value2.Y))
            {
                return value1.Z != value2.Z;
            }
            return true;
        }

        public static JVector Min(JVector value1, JVector value2)
        {
            Min(ref value1, ref value2, out var result);
            return result;
        }

        public static void Min(ref JVector value1, ref JVector value2, out JVector result)
        {
            result.X = (value1.X < value2.X) ? value1.X : value2.X;
            result.Y = (value1.Y < value2.Y) ? value1.Y : value2.Y;
            result.Z = (value1.Z < value2.Z) ? value1.Z : value2.Z;
        }

        public static JVector Max(JVector value1, JVector value2)
        {
            Max(ref value1, ref value2, out var result);
            return result;
        }

        public static void Max(ref JVector value1, ref JVector value2, out JVector result)
        {
            result.X = (value1.X > value2.X) ? value1.X : value2.X;
            result.Y = (value1.Y > value2.Y) ? value1.Y : value2.Y;
            result.Z = (value1.Z > value2.Z) ? value1.Z : value2.Z;
        }

        public void MakeZero()
        {
            X = 0.0f;
            Y = 0.0f;
            Z = 0.0f;
        }

        public bool IsZero()
        {
            return LengthSquared() == 0.0f;
        }

        public bool IsNearlyZero()
        {
            return LengthSquared() < ZeroEpsilonSq;
        }

        public static JVector Transform(JVector position, JMatrix matrix)
        {
            Transform(ref position, ref matrix, out var result);
            return result;
        }

        public static void Transform(ref JVector position, ref JMatrix matrix, out JVector result)
        {
            float num0 = (position.X * matrix.M11) + (position.Y * matrix.M21) + (position.Z * matrix.M31);
            float num1 = (position.X * matrix.M12) + (position.Y * matrix.M22) + (position.Z * matrix.M32);
            float num2 = (position.X * matrix.M13) + (position.Y * matrix.M23) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        public static void TransposedTransform(ref JVector position, ref JMatrix matrix, out JVector result)
        {
            float num0 = (position.X * matrix.M11) + (position.Y * matrix.M12) + (position.Z * matrix.M13);
            float num1 = (position.X * matrix.M21) + (position.Y * matrix.M22) + (position.Z * matrix.M23);
            float num2 = (position.X * matrix.M31) + (position.Y * matrix.M32) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        public static float Dot(JVector vector1, JVector vector2)
        {
            return Dot(ref vector1, ref vector2);
        }

        public static float Dot(ref JVector vector1, ref JVector vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        public static JVector Add(JVector value1, JVector value2)
        {
            Add(ref value1, ref value2, out var result);
            return result;
        }

        public static void Add(ref JVector value1, ref JVector value2, out JVector result)
        {
            float num0 = value1.X + value2.X;
            float num1 = value1.Y + value2.Y;
            float num2 = value1.Z + value2.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        public static JVector Subtract(JVector value1, JVector value2)
        {
            Subtract(ref value1, ref value2, out var result);
            return result;
        }

        public static void Subtract(ref JVector value1, ref JVector value2, out JVector result)
        {
            float num0 = value1.X - value2.X;
            float num1 = value1.Y - value2.Y;
            float num2 = value1.Z - value2.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        public static JVector Cross(JVector vector1, JVector vector2)
        {
            Cross(ref vector1, ref vector2, out var result);
            return result;
        }

        public static void Cross(ref JVector vector1, ref JVector vector2, out JVector result)
        {
            float num3 = (vector1.Y * vector2.Z) - (vector1.Z * vector2.Y);
            float num2 = (vector1.Z * vector2.X) - (vector1.X * vector2.Z);
            float num = (vector1.X * vector2.Y) - (vector1.Y * vector2.X);
            result.X = num3;
            result.Y = num2;
            result.Z = num;
        }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }

        public void Negate()
        {
            X = -X;
            Y = -Y;
            Z = -Z;
        }

        public static JVector Negate(JVector value)
        {
            Negate(ref value, out var result);
            return result;
        }

        public static void Negate(ref JVector value, out JVector result)
        {
            float num0 = -value.X;
            float num1 = -value.Y;
            float num2 = -value.Z;

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        public static JVector Normalize(JVector value)
        {
            Normalize(ref value, out var result);
            return result;
        }

        public void Normalize()
        {
            float num2 = (X * X) + (Y * Y) + (Z * Z);
            float num = 1f / (JMath.Sqrt(num2));
            X *= num;
            Y *= num;
            Z *= num;
        }

        public static void Normalize(ref JVector value, out JVector result)
        {
            float num2 = (value.X * value.X) + (value.Y * value.Y) + (value.Z * value.Z);
            float num = 1f / (JMath.Sqrt(num2));
            result.X = value.X * num;
            result.Y = value.Y * num;
            result.Z = value.Z * num;
        }

        public float LengthSquared()
        {
            return (X * X) + (Y * Y) + (Z * Z);
        }

        public float Length()
        {
            float num = (X * X) + (Y * Y) + (Z * Z);
            return JMath.Sqrt(num);
        }

        public static void Swap(ref JVector vector1, ref JVector vector2)
        {
            float temp = vector1.X;
            vector1.X = vector2.X;
            vector2.X = temp;

            temp = vector1.Y;
            vector1.Y = vector2.Y;
            vector2.Y = temp;

            temp = vector1.Z;
            vector1.Z = vector2.Z;
            vector2.Z = temp;
        }

        public static JVector Multiply(JVector value1, float scaleFactor)
        {
            Multiply(ref value1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(ref JVector value1, float scaleFactor, out JVector result)
        {
            result.X = value1.X * scaleFactor;
            result.Y = value1.Y * scaleFactor;
            result.Z = value1.Z * scaleFactor;
        }

        public static JVector operator %(JVector value1, JVector value2)
        {
            Cross(ref value1, ref value2, out var result);
            return result;
        }

        public static float operator *(JVector value1, JVector value2)
        {
            return Dot(ref value1, ref value2);
        }

        public static JVector operator *(JVector value1, float value2)
        {
            Multiply(ref value1, value2, out var result);
            return result;
        }

        public static JVector operator *(float value1, JVector value2)
        {
            Multiply(ref value2, value1, out var result);
            return result;
        }

        public static JVector operator -(JVector value1, JVector value2)
        {
            Subtract(ref value1, ref value2, out var result);
            return result;
        }

        public static JVector operator +(JVector value1, JVector value2)
        {
            Add(ref value1, ref value2, out var result);
            return result;
        }
    }
}
