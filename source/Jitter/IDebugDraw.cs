using Jitter.LinearMath;

namespace Jitter
{
    public interface IDebugDrawable
    {
        void DebugDraw(IDebugDrawer drawer);
    }

    public interface IDebugDrawer
    {

        void DrawLine(JVector start, JVector end);
        void DrawPoint(JVector pos);
        void DrawTriangle(JVector pos1, JVector pos2, JVector pos3);
    }
}
