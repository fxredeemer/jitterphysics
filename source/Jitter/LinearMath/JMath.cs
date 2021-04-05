﻿using System;

namespace Jitter.LinearMath
{
    public sealed class JMath
    {
        public const float Pi = 3.1415926535f;

        public const float PiOver2 = 1.570796326794f;

        public const float Epsilon = 1.192092896e-012f;

        public static float Sqrt(float number)
        {
            return (float)Math.Sqrt(number);
        }

        public static float Max(float val1, float val2)
        {
            return (val1 > val2) ? val1 : val2;
        }

        public static float Min(float val1, float val2)
        {
            return (val1 < val2) ? val1 : val2;
        }

        public static float Max(float val1, float val2, float val3)
        {
            float max12 = (val1 > val2) ? val1 : val2;
            return (max12 > val3) ? max12 : val3;
        }

        public static float Clamp(float value, float min, float max)
        {
            value = (value > max) ? max : value;
            value = (value < min) ? min : value;
            return value;
        }

        public static JMatrix Absolute(in JMatrix matrix)
        {
            Absolute(matrix, out var absolute);
            return absolute;
        }

        public static void Absolute(in JMatrix matrix, out JMatrix result)
        {
            result.M11 = Math.Abs(matrix.M11);
            result.M12 = Math.Abs(matrix.M12);
            result.M13 = Math.Abs(matrix.M13);
            result.M21 = Math.Abs(matrix.M21);
            result.M22 = Math.Abs(matrix.M22);
            result.M23 = Math.Abs(matrix.M23);
            result.M31 = Math.Abs(matrix.M31);
            result.M32 = Math.Abs(matrix.M32);
            result.M33 = Math.Abs(matrix.M33);
        }
    }
}
