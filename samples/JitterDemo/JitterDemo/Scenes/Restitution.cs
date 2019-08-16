using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    class Restitution : Scene
    {
        public Restitution(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 11; i++)
            {
                var box = new RigidBody(new BoxShape(1,0.01f,1));
                Demo.World.AddBody(box);
                var boxPos = new JVector(-15 + i * 3 + 1, 5, 0);

                box.Position = boxPos;
                box.IsStatic = true;

                var sphere = new RigidBody(new SphereShape(0.5f));
                Demo.World.AddBody(sphere);

                sphere.Position = boxPos + JVector.Up * 30;
                sphere.EnableSpeculativeContacts = true;

                // set restitution
                sphere.Material.Restitution = box.Material.Restitution = 1.0f / 10.0f * i;
                sphere.LinearVelocity = new JVector(0, 0, 0);
    

                sphere.Damping = RigidBody.DampingType.Angular;
            }
        }
    }
}