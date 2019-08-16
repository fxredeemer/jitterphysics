using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using System.Collections.Generic;
using System.Diagnostics;

namespace Jitter.Collision
{
    public interface IBroadphaseEntity
    {
        JBBox BoundingBox { get; }
        bool IsStaticOrInactive { get; }
    }
}
