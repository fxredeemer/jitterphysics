namespace Jitter.Dynamics
{
    public class Material
    {
        internal float kineticFriction = 0.3f;
        internal float staticFriction = 0.6f;
        internal float restitution;

        public float Restitution
        {
            get => restitution;
            set => restitution = value;
        }

        public float StaticFriction
        {
            get => staticFriction;
            set => staticFriction = value;
        }

        public float KineticFriction
        {
            get => kineticFriction;
            set => kineticFriction = value;
        }
    }
}
