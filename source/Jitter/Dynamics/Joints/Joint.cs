namespace Jitter.Dynamics.Joints
{
    public abstract class Joint
    {
        public World World { get; }

        public Joint(World world)
        {
            World = world;
        }

        public abstract void Activate();

        public abstract void Deactivate();
    }
}
