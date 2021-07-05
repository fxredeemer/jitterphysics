namespace Jitter.LinearMath
{
    public readonly struct JVector
    {
        public static JVector Zero { get; } = new JVector(0, 0, 0);
        public static JVector Left { get; } = new JVector(1, 0, 0);
        public static JVector Right { get; } = new JVector(-1, 0, 0);
        public static JVector Up { get; } = new JVector(0, 1, 0);
        public static JVector Down { get; } = new JVector(0, -1, 0);
        public static JVector Backward { get; } = new JVector(0, 0, 1);
        public static JVector Forward { get; } = new JVector(0, 0, -1);
        public static JVector One { get; } = new JVector(1, 1, 1);
        public static JVector MinValue { get; } = new JVector(float.MinValue);
        public static JVector MaxValue { get; } = new JVector(float.MaxValue);

        internal static JVector Arbitrary { get; } = new JVector(1, 1, 1);

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

        public static bool operator ==(in JVector value1, in JVector value2)
        {
            return (value1.X == value2.X)
                && (value1.Y == value2.Y)
                && (value1.Z == value2.Z);
        }

        public static bool operator !=(in JVector value1, in JVector value2)
        {
            if ((value1.X == value2.X) && (value1.Y == value2.Y))
            {
                return value1.Z != value2.Z;
            }
            return true;
        }

        public static JVector Min(in JVector value1, in JVector value2)
        {
            Min(in value1, in value2, out var result);
            return result;
        }

        public static void Min(in JVector value1, in JVector value2, out JVector result)
        {
            result = new JVector(
                JMath.Min(value1.X, value2.X),
                JMath.Min(value1.Y, value2.Y),
                JMath.Min(value1.Z, value2.Z));
        }

        public static JVector Max(in JVector value1, in JVector value2)
        {
            Max(in value1, in value2, out var result);
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

        public static JVector Transform(in JVector position, JMatrix matrix)
        {
            Transform(in position, in matrix, out var result);
            return result;
        }

        public static void Transform(in JVector position, in JMatrix matrix, out JVector result)
        {
            var x = (position.X * matrix.M11) + (position.Y * matrix.M21) + (position.Z * matrix.M31);
            var y = (position.X * matrix.M12) + (position.Y * matrix.M22) + (position.Z * matrix.M32);
            var z = (position.X * matrix.M13) + (position.Y * matrix.M23) + (position.Z * matrix.M33);

            result = new JVector(x, y, z);
        }

        public static void TransposedTransform(in JVector position, in JMatrix matrix, out JVector result)
        {
            var num0 = (position.X * matrix.M11) + (position.Y * matrix.M12) + (position.Z * matrix.M13);
            var num1 = (position.X * matrix.M21) + (position.Y * matrix.M22) + (position.Z * matrix.M23);
            var num2 = (position.X * matrix.M31) + (position.Y * matrix.M32) + (position.Z * matrix.M33);

            result = new JVector(num0, num1, num2);
        }

        public static float Dot(in JVector vector1, in JVector vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        public static JVector Add(in JVector value1, in JVector value2)
        {
            Add(in value1, in value2, out var result);
            return result;
        }

        public static void Add(in JVector value1, in JVector value2, out JVector result)
        {
            var num0 = value1.X + value2.X;
            var num1 = value1.Y + value2.Y;
            var num2 = value1.Z + value2.Z;

            result = new JVector(num0, num1, num2);
        }

        public static JVector Subtract(in JVector value1, in JVector value2)
        {
            Subtract(in value1, in value2, out var result);
            return result;
        }

        public static void Subtract(in JVector value1, in JVector value2, out JVector result)
        {
            var num0 = value1.X - value2.X;
            var num1 = value1.Y - value2.Y;
            var num2 = value1.Z - value2.Z;

            result = new JVector(num0, num1, num2);
        }

        public static JVector Cross(in JVector vector1, in JVector vector2)
        {
            Cross(in vector1, in vector2, out var result);
            return result;
        }

        public static void Cross(in JVector vector1, in JVector vector2, out JVector result)
        {
            var num3 = (vector1.Y * vector2.Z) - (vector1.Z * vector2.Y);
            var num2 = (vector1.Z * vector2.X) - (vector1.X * vector2.Z);
            var num = (vector1.X * vector2.Y) - (vector1.Y * vector2.X);

            result = new JVector(num3, num2, num);
        }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }

        public static JVector Negate(in JVector value)
        {
            Negate(in value, out var result);
            return result;
        }

        public static void Negate(in JVector value, out JVector result)
        {
            var num0 = -value.X;
            var num1 = -value.Y;
            var num2 = -value.Z;

            result = new JVector(num0, num1, num2);
        }

        public static JVector Normalize(in JVector value)
        {
            Normalize(in value, out var result);
            return result;
        }

        public static void Normalize(in JVector value, out JVector result)
        {
            var num2 = (value.X * value.X) + (value.Y * value.Y) + (value.Z * value.Z);
            var num = 1f / (JMath.Sqrt(num2));

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
            var num = (X * X) + (Y * Y) + (Z * Z);
            return JMath.Sqrt(num);
        }

        public static void Swap(ref JVector vector1, ref JVector vector2)
        {
            var tempVector = vector1;
            vector1 = vector2;
            vector2 = tempVector;
        }

        public static JVector Multiply(in JVector value1, float scaleFactor)
        {
            Multiply(in value1, scaleFactor, out var result);
            return result;
        }

        public static void Multiply(in JVector value1, float scaleFactor, out JVector result)
        {
            result = new JVector(
                value1.X * scaleFactor,
                value1.Y * scaleFactor,
                value1.Z * scaleFactor);
        }

        public static JVector operator %(in JVector value1, in JVector value2)
        {
            Cross(in value1, in value2, out var result);
            return result;
        }

        public static float operator *(in JVector value1, in JVector value2)
        {
            return Dot(in value1, in value2);
        }

        public static JVector operator *(in JVector value1, float value2)
        {
            Multiply(in value1, value2, out var result);
            return result;
        }

        public static JVector operator *(float value1, in JVector value2)
        {
            Multiply(in value2, value1, out var result);
            return result;
        }

        public static JVector operator -(in JVector value1, in JVector value2)
        {
            Subtract(in value1, in value2, out var result);
            return result;
        }

        public static JVector operator +(in JVector value1, in JVector value2)
        {
            Add(in value1, in value2, out var result);
            return result;
        }
    }
}
