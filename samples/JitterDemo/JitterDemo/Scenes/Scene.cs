﻿using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using JitterDemo.Vehicle;

namespace JitterDemo.Scenes
{
    public abstract class Scene
    {
        public JitterDemo Demo { get; }

        public Scene(JitterDemo demo)
        {
            Demo = demo;
        }

        public abstract void Build();

        private QuadDrawer quadDrawer = null;
        protected RigidBody ground = null;
        protected CarObject car = null;

        public void AddGround()
        {
            ground = new RigidBody(new BoxShape(new JVector(200, 20, 200)))
            {
                Position = new JVector(0, -10, 0),
                Tag = BodyTag.DontDrawMe,
                IsStatic = true
            };
            Demo.World.AddBody(ground);
            //ground.Restitution = 1.0f;
            ground.Material.KineticFriction = 0.0f;

            quadDrawer = new QuadDrawer(Demo,100);
            Demo.Components.Add(quadDrawer);
        }

        public void RemoveGround()
        {
            Demo.World.RemoveBody(ground);
            Demo.Components.Remove(quadDrawer);
            quadDrawer.Dispose();
        }

        public void AddCar(JVector position)
        {
            car = new CarObject(Demo);
            Demo.Components.Add(car);

            car.carBody.Position = position;
        }

        public void RemoveCar()
        {
            Demo.World.RemoveBody(car.carBody);
            Demo.Components.Remove(quadDrawer);
            Demo.Components.Remove(car);
        }

        public virtual void Draw() { }
    }
}
