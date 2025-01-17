﻿using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    internal class Wall : Scene
    {
        public Wall(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int k = 0; k < 1; k++)
            {
                for (int i = 0; i < 20; i++)
                {
                    for (int e = 0; e < 20; e++)
                    {
                        var body = new RigidBody(new BoxShape(2, 1, 1))
                        {
                            Position = new JVector((e * 2.01f) + ((i % 2 == 0) ? 1f : 0.0f), 0.5f + (i * 1.0f), k * 5)
                        };
                        Demo.World.AddBody(body);
                    }
                }
            }
        }
    }
}