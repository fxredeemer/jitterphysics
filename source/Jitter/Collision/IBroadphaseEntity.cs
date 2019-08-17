using Jitter.LinearMath;

namespace Jitter.Collision
{
    public interface IBroadphaseEntity
    {
        JBBox BoundingBox { get; }
        bool IsStaticOrInactive { get; }
    }
}
