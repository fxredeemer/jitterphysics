namespace Jitter.Dynamics
{
    public class ContactSettings
    {
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        internal float maximumBias = 10.0f;
        internal float bias = 0.25f;
        internal float minVelocity = 0.001f;
        internal float allowedPenetration = 0.01f;
        internal float breakThreshold = 0.01f;

        internal MaterialCoefficientMixingType materialMode = MaterialCoefficientMixingType.UseAverage;

        public float MaximumBias { get => maximumBias; set => maximumBias = value; }

        public float BiasFactor { get => bias; set => bias = value; }

        public float MinimumVelocity { get => minVelocity; set => minVelocity = value; }

        public float AllowedPenetration { get => allowedPenetration; set => allowedPenetration = value; }

        public float BreakThreshold { get => breakThreshold; set => breakThreshold = value; }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get => materialMode; set => materialMode = value; }
    }
}
