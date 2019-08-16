using Jitter.DataStructures;
using Jitter.Dynamics;
using Jitter.Dynamics.Constraints;
using System.Collections.Generic;

namespace Jitter.Collision
{
    public class CollisionIsland
    {
        internal IslandManager islandManager;
        internal HashSet<RigidBody> bodies = new HashSet<RigidBody>();
        internal HashSet<Arbiter> arbiter = new HashSet<Arbiter>();
        internal HashSet<Constraint> constraints = new HashSet<Constraint>();

        public ReadOnlyHashset<RigidBody> Bodies { get; }
        public ReadOnlyHashset<Arbiter> Arbiter { get; }
        public ReadOnlyHashset<Constraint> Constraints { get; }

        public CollisionIsland()
        {
            Bodies = new ReadOnlyHashset<RigidBody>(bodies);
            Arbiter = new ReadOnlyHashset<Arbiter>(arbiter);
            Constraints = new ReadOnlyHashset<Constraint>(constraints);
        }

        public bool IsActive()
        {
            var enumerator = bodies.GetEnumerator();
            enumerator.MoveNext();

            if (enumerator.Current == null)
            {
                return false;
            }
            else
            {
                return enumerator.Current.isActive;
            }
        }

        public void SetStatus(bool active)
        {
            foreach (var body in bodies)
            {
                body.IsActive = active;
                if (active && !body.IsActive)
                {
                    body.inactiveTime = 0.0f;
                }
            }
        }

        internal void ClearLists()
        {
            arbiter.Clear();
            bodies.Clear();
            constraints.Clear();
        }
    }
}
