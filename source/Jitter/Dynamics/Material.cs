namespace Jitter.Dynamics
{
    public class Material
    {
        internal float kineticFriction = 0.3f;
        internal float staticFriction = 0.6f;
        internal float restitution = 0.0f;

        public float Restitution
        {
            get { return restitution; }
            set { restitution = value; }
        }

        public float StaticFriction
        {
            get { return staticFriction; }
            set { staticFriction = value; }
        }

        public float KineticFriction
        {
            get { return kineticFriction; }
            set { kineticFriction = value; }
        }
    }
}
