namespace Jitter.Dynamics
{
    public interface IConstraint
    {

        RigidBody Body1 { get; }

        RigidBody Body2 { get; }

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        void PrepareForIteration(float timestep);

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        void Iterate();
    }
}
