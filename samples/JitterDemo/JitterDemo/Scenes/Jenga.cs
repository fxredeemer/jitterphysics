﻿using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    internal class Jenga : Scene
    {
        public Jenga(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 15; i++)
            {
                bool even = i % 2 == 0;

                for (int e = 0; e < 3; e++)
                {
                    var size = even ? new JVector(1, 1, 3) : new JVector(3, 1, 1);
                    var body = new RigidBody(new BoxShape(size))
                    {
                        Position = new JVector(3.0f + (even ? e : 1.0f), i + 0.5f, -13.0f + (even ? 1.0f : e))
                    };

                    Demo.World.AddBody(body);
                }
            }

            //BoxShape bs = new BoxShape(10, 10, 0.01f);
            //RigidBody bb = new RigidBody(bs);

            //bb.Position = new JVector(10, 5, 0);

            //Demo.World.AddBody(bb);
            //bb.IsStatic = true;
            
        }
    }
}