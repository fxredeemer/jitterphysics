using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    class BroadphaseStress : Scene
    {
        public BroadphaseStress(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            var shape = new BoxShape(JVector.One);

            // CollisionSystemBrute        170 ms
            // CollisionSystemSAP          7   ms
            // CollisionSystemPersistenSAP 1   ms

            for (int i = 0; i < 15; i++)
            {
                for (int e = 0; e < 15; e++)
                {
                    for (int k = 0; k < 15; k++)
                    {
                        var b = new RigidBody(shape);
                        Demo.World.AddBody(b);
                        b.Position = new JVector(i, e, k) * 2.0f;
                        b.AffectedByGravity = false;
                    }
                }
            }
        }
    }
}