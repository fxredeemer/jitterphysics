﻿using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    internal class CylinderWall : Scene
    {
        public CylinderWall(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 20; i++)
            {
                for (int e = 0; e < 20; e++)
                {
                    var body = new RigidBody(new CylinderShape(1.0f, 0.5f))
                    {
                        Position = new JVector((e * 1.01f) + ((i % 2 == 0) ? 0.5f : 0.0f), 0.5f + (i * 1.0f), 0.0f)
                    };
                    Demo.World.AddBody(body);
                }
            }
        }
    }
}