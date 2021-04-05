namespace Jitter.LinearMath
{
    public readonly struct JVector
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

        public float X { get; }
        public float Y { get; }
        public float Z { get; }

        public JVector(float x, float y, float z)
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
            if ((value1.X == value2.X)
                && (value1.Y == value2.Y))
            {
                return value1.Z != value2.Z;
            }
            return true;
        }

        public static JVector Min(JVector value1, JVector value2)
        {
            Min(value1, value2, out var result);
            return result;
        }

        public static void Min(in JVector value1, in JVector value2, out JVector result)
        {
            result = new JVector(
                JMath.Min(value1.X, value2.X),
                JMath.Min(value1.Y, value2.Y),
                JMath.Min(value1.Z, value2.Z));
        }

        public static JVector Max(JVector value1, JVector value2)
        {
            Max(value1, value2, out var result);
            return result;
        }

        public static void Max(in JVector value1, in JVector value2, out JVector result)
        {
            result = new JVector(
                JMath.Max(value1.X, value2.X),
                JMath.Max(value1.Y, value2.Y),
                JMath.Max(value1.Z, value2.Z));
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
            Transform(position, matrix, out var result);
            return result;
        }

        public static void Transform(in JVector position, in JMatrix matrix, out JVector result)
        {
            result = new JVector(
                (position.X * matrix.M11) + (position.Y * matrix.M21) + (position.Z * matrix.M31),
                (position.X * matrix.M12) + (position.Y * matrix.M22) + (position.Z * matrix.M32),
                (position.X * matrix.M13) + (position.Y * matrix.M23) + (position.Z * matrix.M33));
        }

        public static JVector Transform(in JVector position, in JMatrix matrix)
        {
            return new JVector(
                (position.X * matrix.M11) + (position.Y * matrix.M21) + (position.Z * matrix.M31),
                (position.X * matrix.M12) + (position.Y * matrix.M22) + (position.Z * matrix.M32),
                (position.X * matrix.M13) + (position.Y * matrix.M23) + (position.Z * matrix.M33));
        }

        public static JVector TransposedTransform(in JVector position, in JMatrix matrix)
        {
            return new JVector(
                (position.X * matrix.M11) + (position.Y * matrix.M12) + (position.Z * matrix.M13),
                (position.X * matrix.M21) + (position.Y * matrix.M22) + (position.Z * matrix.M23),
                (position.X * matrix.M31) + (position.Y * matrix.M32) + (position.Z * matrix.M33));
        }

        public static float Dot(JVector vector1, JVector vector2)
        {
            return Dot(vector1, vector2);
        }

        public static float Dot(in JVector vector1, in JVector vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        public static JVector Add(JVector value1, JVector value2)
        {
            Add(value1, value2, out var result);
            return result;
        }

        public static void Add(in JVector value1, in JVector value2, out JVector result)
        {
            result = new JVector(
                value1.X + value2.X,
                value1.Y + value2.Y,
                value1.Z + value2.Z);
        }

        public static JVector Subtract(JVector value1, JVector value2)
        {
            Subtract(value1, value2, out var result);
            return result;
        }

        public static void Subtract(in JVector value1, in JVector value2, out JVector result)
        {
            result = new JVector(
                value1.X - value2.X,
                value1.Y - value2.Y,
                value1.Z - value2.Z);
        }

        public static JVector Cross(JVector vector1, JVector vector2)
        {
            Cross(vector1, vector2, out var result);
            return result;
        }

        public static void Cross(in JVector vector1, in JVector vector2, out JVector result)
        {
            result = new JVector(
                (vector1.Y * vector2.Z) - (vector1.Z * vector2.Y),
                (vector1.Z * vector2.X) - (vector1.X * vector2.Z),
                (vector1.X * vector2.Y) - (vector1.Y * vector2.X));
        }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }

        public static JVector Negate(JVector value)
        {
            Negate(value, out var result);
            return result;
        }

        public static void Negate(in JVector value, out JVector result)
        {
            result = new JVector(-value.X, -value.Y, -value.Z);
        }

        public static JVector Normalize(JVector value)
        {
            Normalize(value, out var result);
            return result;
        }

        public static void Normalize(in JVector value, out JVector result)
        {
            float num2 = (value.X * value.X) + (value.Y * value.Y) + (value.Z * value.Z);
            float num = 1f / JMath.Sqrt(num2);

            result = new JVector(
                value.X * num,
                value.Y * num,
                value.Z * num);
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
            var tempVector = vector1;
            vector1 = vector2;
            vector2 = tempVector;
        }

        public static JVector Multiply(JVector value1, float scaleFactor)
        {
            Multiply(value1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JVector value1, float scaleFactor, out JVector result)
        {
            result = new JVector(
                value1.X * scaleFactor,
                value1.Y * scaleFactor,
                value1.Z * scaleFactor);
        }

        public static JVector operator %(JVector value1, JVector value2)
        {
            Cross(value1, value2, out var result);
            return result;
        }

        public static float operator *(JVector value1, JVector value2)
        {
            return Dot(value1, in value2);
        }

        public static JVector operator *(JVector value1, float value2)
        {
            Multiply(value1, value2, out var result);
            return result;
        }

        public static JVector operator *(float value1, JVector value2)
        {
            Multiply(value2, value1, out var result);
            return result;
        }

        public static JVector operator -(JVector value1, JVector value2)
        {
            Subtract(value1, value2, out var result);
            return result;
        }

        public static JVector operator +(JVector value1, JVector value2)
        {
            Add(value1, value2, out var result);
            return result;
        }
    }
}
