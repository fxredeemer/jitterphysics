using Jitter.Collision;
using Jitter.DataStructures;
using Jitter.Dynamics;
using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;

namespace Jitter
{
    public class World
    {
        public delegate void WorldStep(float timestep);

        public class WorldEvents
        {
            public event WorldStep PreStep;
            public event WorldStep PostStep;

            public event Action<RigidBody> AddedRigidBody;
            public event Action<RigidBody> RemovedRigidBody;
            public event Action<Constraint> AddedConstraint;
            public event Action<Constraint> RemovedConstraint;
            public event Action<SoftBody> AddedSoftBody;
            public event Action<SoftBody> RemovedSoftBody;

            public event Action<RigidBody, RigidBody> BodiesBeginCollide;
            public event Action<RigidBody, RigidBody> BodiesEndCollide;
            public event Action<Contact> ContactCreated;

            public event Action<RigidBody> DeactivatedBody;
            public event Action<RigidBody> ActivatedBody;

            internal WorldEvents() { }


            internal void RaiseWorldPreStep(float timestep)
            {
                PreStep?.Invoke(timestep);
            }

            internal void RaiseWorldPostStep(float timestep)
            {
                PostStep?.Invoke(timestep);
            }

            internal void RaiseAddedRigidBody(RigidBody body)
            {
                AddedRigidBody?.Invoke(body);
            }

            internal void RaiseRemovedRigidBody(RigidBody body)
            {
                RemovedRigidBody?.Invoke(body);
            }

            internal void RaiseAddedConstraint(Constraint constraint)
            {
                AddedConstraint?.Invoke(constraint);
            }

            internal void RaiseRemovedConstraint(Constraint constraint)
            {
                RemovedConstraint?.Invoke(constraint);
            }

            internal void RaiseAddedSoftBody(SoftBody body)
            {
                AddedSoftBody?.Invoke(body);
            }

            internal void RaiseRemovedSoftBody(SoftBody body)
            {
                RemovedSoftBody?.Invoke(body);
            }

            internal void RaiseBodiesBeginCollide(RigidBody body1, RigidBody body2)
            {
                BodiesBeginCollide?.Invoke(body1, body2);
            }

            internal void RaiseBodiesEndCollide(RigidBody body1, RigidBody body2)
            {
                BodiesEndCollide?.Invoke(body1, body2);
            }

            internal void RaiseActivatedBody(RigidBody body)
            {
                ActivatedBody?.Invoke(body);
            }

            internal void RaiseDeactivatedBody(RigidBody body)
            {
                DeactivatedBody?.Invoke(body);
            }

            internal void RaiseContactCreated(Contact contact)
            {
                ContactCreated?.Invoke(contact);
            }

        }

        private float inactiveAngularThresholdSq = 0.1f;
        private float inactiveLinearThresholdSq = 0.1f;
        private float deactivationTime = 2f;

        private float angularDamping = 0.85f;
        private float linearDamping = 0.85f;

        private int contactIterations = 10;
        private int smallIterations = 4;
        private float timestep;

        private readonly IslandManager islands = new IslandManager();

        private readonly HashSet<RigidBody> rigidBodies = new HashSet<RigidBody>();
        private readonly HashSet<Constraint> constraints = new HashSet<Constraint>();
        private readonly HashSet<SoftBody> softbodies = new HashSet<SoftBody>();

        public ReadOnlyHashset<RigidBody> RigidBodies { get; }
        public ReadOnlyHashset<Constraint> Constraints { get; }
        public ReadOnlyHashset<SoftBody> SoftBodies { get; }
        public WorldEvents Events { get; } = new WorldEvents();

        private readonly ThreadManager threadManager = ThreadManager.Instance;

        public ArbiterMap ArbiterMap { get; }

        private readonly Queue<Arbiter> removedArbiterQueue = new Queue<Arbiter>();
        private readonly Queue<Arbiter> addedArbiterQueue = new Queue<Arbiter>();

        private JVector gravity = new JVector(0, -9.81f, 0);

        public ContactSettings ContactSettings { get; } = new ContactSettings();

        public ReadOnlyCollection<CollisionIsland> Islands => islands;

        private readonly Action<object> arbiterCallback;
        private readonly Action<object> integrateCallback;

        private readonly CollisionDetectedHandler collisionDetectionHandler;

        public World(CollisionSystem collision)
        {
            CollisionSystem = collision ?? throw new ArgumentNullException("The CollisionSystem can't be null.", nameof(collision));

            arbiterCallback = new Action<object>(ArbiterCallback);
            integrateCallback = new Action<object>(IntegrateCallback);

            RigidBodies = new ReadOnlyHashset<RigidBody>(rigidBodies);
            Constraints = new ReadOnlyHashset<Constraint>(constraints);
            SoftBodies = new ReadOnlyHashset<SoftBody>(softbodies);

            collisionDetectionHandler = new CollisionDetectedHandler(CollisionDetected);

            CollisionSystem.CollisionDetected += collisionDetectionHandler;

            ArbiterMap = new ArbiterMap();

            AllowDeactivation = true;
        }

        public void AddBody(SoftBody body)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body), "body can't be null.");
            }

            if (softbodies.Contains(body))
            {
                throw new ArgumentException("The body was already added to the world.", nameof(body));
            }

            softbodies.Add(body);
            CollisionSystem.AddEntity(body);

            Events.RaiseAddedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
            {
                AddConstraint(constraint);
            }

            foreach (var massPoint in body.VertexBodies)
            {
                Events.RaiseAddedRigidBody(massPoint);
                rigidBodies.Add(massPoint);
            }
        }

        public bool RemoveBody(SoftBody body)
        {
            if (!softbodies.Remove(body))
            {
                return false;
            }

            CollisionSystem.RemoveEntity(body);

            Events.RaiseRemovedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
            {
                RemoveConstraint(constraint);
            }

            foreach (var massPoint in body.VertexBodies)
            {
                RemoveBody(massPoint, true);
            }

            return true;
        }

        public CollisionSystem CollisionSystem { set; get; }

        public static void ResetResourcePools()
        {
            IslandManager.Pool.ResetResourcePool();
            Arbiter.Pool.ResetResourcePool();
            Contact.Pool.ResetResourcePool();
        }

        public void Clear()
        {
            foreach (var body in rigidBodies)
            {
                CollisionSystem.RemoveEntity(body);

                if (body.island != null)
                {
                    body.island.ClearLists();
                    body.island = null;
                }

                body.connections.Clear();
                body.arbiters.Clear();
                body.constraints.Clear();

                Events.RaiseRemovedRigidBody(body);
            }

            foreach (var body in softbodies)
            {
                CollisionSystem.RemoveEntity(body);
            }

            rigidBodies.Clear();

            foreach (var constraint in constraints)
            {
                Events.RaiseRemovedConstraint(constraint);
            }
            constraints.Clear();
            softbodies.Clear();
            islands.RemoveAll();
            ArbiterMap.Clear();
            ResetResourcePools();
        }

        public JVector Gravity { get => gravity; set => gravity = value; }

        public bool AllowDeactivation { get; set; }

        public void SetDampingFactors(float angularDamping, float linearDamping)
        {
            if (angularDamping < 0.0f || angularDamping > 1.0f)
            {
                throw new ArgumentException("Angular damping factor has to be between 0.0 and 1.0", nameof(angularDamping));
            }

            if (linearDamping < 0.0f || linearDamping > 1.0f)
            {
                throw new ArgumentException("Linear damping factor has to be between 0.0 and 1.0", nameof(linearDamping));
            }

            this.angularDamping = angularDamping;
            this.linearDamping = linearDamping;
        }

        public void SetInactivityThreshold(float angularVelocity, float linearVelocity, float time)
        {
            if (angularVelocity < 0.0f)
            {
                throw new ArgumentException("Angular velocity threshold has to " +
                 "be larger than zero", nameof(angularVelocity));
            }

            if (linearVelocity < 0.0f)
            {
                throw new ArgumentException("Linear velocity threshold has to " +
                "be larger than zero", nameof(linearVelocity));
            }

            if (time < 0.0f)
            {
                throw new ArgumentException("Deactivation time threshold has to " +
                "be larger than zero", nameof(time));
            }

            inactiveAngularThresholdSq = angularVelocity * angularVelocity;
            inactiveLinearThresholdSq = linearVelocity * linearVelocity;
            deactivationTime = time;
        }

        public void SetIterations(int iterations, int smallIterations)
        {
            if (iterations < 1)
            {
                throw new ArgumentException("The number of collision " +
                 "iterations has to be larger than zero", nameof(iterations));
            }

            if (smallIterations < 1)
            {
                throw new ArgumentException("The number of collision " +
                "iterations has to be larger than zero", nameof(smallIterations));
            }

            contactIterations = iterations;
            this.smallIterations = smallIterations;
        }

        public bool RemoveBody(RigidBody body)
        {
            return RemoveBody(body, false);
        }

        private bool RemoveBody(RigidBody body, bool removeMassPoints)
        {
            if (!removeMassPoints && body.IsParticle)
            {
                return false;
            }

            if (!rigidBodies.Remove(body))
            {
                return false;
            }

            foreach (var arbiter in body.arbiters)
            {
                ArbiterMap.Remove(arbiter);
                Events.RaiseBodiesEndCollide(arbiter.body1, arbiter.body2);
            }

            foreach (var constraint in body.constraints)
            {
                constraints.Remove(constraint);
                Events.RaiseRemovedConstraint(constraint);
            }

            CollisionSystem.RemoveEntity(body);

            islands.RemoveBody(body);

            Events.RaiseRemovedRigidBody(body);

            return true;
        }

        public void AddBody(RigidBody body)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body), "body can't be null.");
            }

            if (rigidBodies.Contains(body))
            {
                throw new ArgumentException("The body was already added to the world.", nameof(body));
            }

            Events.RaiseAddedRigidBody(body);

            CollisionSystem.AddEntity(body);

            rigidBodies.Add(body);
        }

        public bool RemoveConstraint(Constraint constraint)
        {
            if (!constraints.Remove(constraint))
            {
                return false;
            }

            Events.RaiseRemovedConstraint(constraint);

            islands.ConstraintRemoved(constraint);

            return true;
        }

        public void AddConstraint(Constraint constraint)
        {
            if (constraints.Contains(constraint))
            {
                throw new ArgumentException("The constraint was already added to the world.", nameof(constraint));
            }

            constraints.Add(constraint);

            islands.ConstraintCreated(constraint);

            Events.RaiseAddedConstraint(constraint);
        }

        private float currentLinearDampFactor = 1.0f;
        private float currentAngularDampFactor = 1.0f;

        private readonly Stopwatch sw = new Stopwatch();

        public enum DebugType
        {
            CollisionDetect, BuildIslands, HandleArbiter, UpdateContacts,
            PreStep, DeactivateBodies, IntegrateForces, Integrate, PostStep, ClothUpdate, Num
        }

        public double[] DebugTimes { get; } = new double[(int)DebugType.Num];

        public void Step(float timestep, bool multithread)
        {
            this.timestep = timestep;

            if (timestep == 0.0f)
            {
                return;
            }

            if (timestep < 0.0f)
            {
                throw new ArgumentException("The timestep can't be negative.", nameof(timestep));
            }

            currentAngularDampFactor = (float)Math.Pow(angularDamping, timestep);
            currentLinearDampFactor = (float)Math.Pow(linearDamping, timestep);

            sw.Restart();
            Events.RaiseWorldPreStep(timestep);
            foreach (var body in rigidBodies)
            {
                body.PreStep(timestep);
            }

            sw.Stop(); DebugTimes[(int)DebugType.PreStep] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            UpdateContacts();
            sw.Stop(); DebugTimes[(int)DebugType.UpdateContacts] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            while (removedArbiterQueue.Count > 0)
            {
                islands.ArbiterRemoved(removedArbiterQueue.Dequeue());
            }

            sw.Stop();
            double ms = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            foreach (var body in softbodies)
            {
                body.Update(timestep);
                body.DoSelfCollision(collisionDetectionHandler);
            }
            sw.Stop();
            DebugTimes[(int)DebugType.ClothUpdate] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            CollisionSystem.Detect(multithread);
            sw.Stop();
            DebugTimes[(int)DebugType.CollisionDetect] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();

            while (addedArbiterQueue.Count > 0)
            {
                islands.ArbiterCreated(addedArbiterQueue.Dequeue());
            }

            sw.Stop(); DebugTimes[(int)DebugType.BuildIslands] = sw.Elapsed.TotalMilliseconds + ms;

            sw.Restart();
            CheckDeactivation();
            sw.Stop(); DebugTimes[(int)DebugType.DeactivateBodies] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            IntegrateForces();
            sw.Stop(); DebugTimes[(int)DebugType.IntegrateForces] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            HandleArbiter(multithread);
            sw.Stop(); DebugTimes[(int)DebugType.HandleArbiter] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            Integrate(multithread);
            sw.Stop(); DebugTimes[(int)DebugType.Integrate] = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            foreach (var body in rigidBodies)
            {
                body.PostStep(timestep);
            }

            Events.RaiseWorldPostStep(timestep);
            sw.Stop(); DebugTimes[(int)DebugType.PostStep] = sw.Elapsed.TotalMilliseconds;
        }

        private float accumulatedTime;

        public void Step(float totalTime, bool multithread, float timestep, int maxSteps)
        {
            int counter = 0;
            accumulatedTime += totalTime;

            while (accumulatedTime > timestep)
            {
                Step(timestep, multithread);

                accumulatedTime -= timestep;
                counter++;

                if (counter > maxSteps)
                {
                    accumulatedTime = 0.0f;
                    break;
                }
            }
        }

        private void UpdateArbiterContacts(Arbiter arbiter)
        {
            if (arbiter.contactList.Count == 0)
            {
                lock (removedArbiterStack) { removedArbiterStack.Push(arbiter); }
                return;
            }

            for (int i = arbiter.contactList.Count - 1; i >= 0; i--)
            {
                var c = arbiter.contactList[i];
                c.UpdatePosition();

                if (c.penetration < -ContactSettings.breakThreshold)
                {
                    Contact.Pool.GiveBack(c);
                    arbiter.contactList.RemoveAt(i);
                    continue;
                }
                else
                {
                    JVector.Subtract(c.p1, c.p2, out var diff);
                    float distance = JVector.Dot(diff, c.normal);

                    diff -= (distance * c.normal);
                    distance = diff.LengthSquared();

                    if (distance > ContactSettings.breakThreshold * ContactSettings.breakThreshold * 100)
                    {
                        Contact.Pool.GiveBack(c);
                        arbiter.contactList.RemoveAt(i);
                        continue;
                    }
                }
            }
        }

        private readonly Stack<Arbiter> removedArbiterStack = new Stack<Arbiter>();

        private void UpdateContacts()
        {
            foreach (var arbiter in ArbiterMap.Arbiters)
            {
                UpdateArbiterContacts(arbiter);
            }

            while (removedArbiterStack.Count > 0)
            {
                var arbiter = removedArbiterStack.Pop();
                Arbiter.Pool.GiveBack(arbiter);
                ArbiterMap.Remove(arbiter);

                removedArbiterQueue.Enqueue(arbiter);
                Events.RaiseBodiesEndCollide(arbiter.body1, arbiter.body2);
            }
        }

        private void ArbiterCallback(object obj)
        {
            var island = obj as CollisionIsland;

            int thisIterations;
            if (island.Bodies.Count + island.Constraints.Count > 3)
            {
                thisIterations = contactIterations;
            }
            else
            {
                thisIterations = smallIterations;
            }

            for (int i = -1; i < thisIterations; i++)
            {
                foreach (var arbiter in island.arbiter)
                {
                    int contactCount = arbiter.contactList.Count;
                    for (int e = 0; e < contactCount; e++)
                    {
                        if (i == -1)
                        {
                            arbiter.contactList[e].PrepareForIteration(timestep);
                        }
                        else
                        {
                            arbiter.contactList[e].Iterate();
                        }
                    }
                }

                foreach (var c in island.constraints)
                {
                    if (c.body1 != null && !c.body1.IsActive && c.body2?.IsActive == false)
                    {
                        continue;
                    }

                    if (i == -1)
                    {
                        c.PrepareForIteration(timestep);
                    }
                    else
                    {
                        c.Iterate();
                    }
                }
            }
        }

        private void HandleArbiter(bool multiThreaded)
        {
            if (multiThreaded)
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if (islands[i].IsActive())
                    {
                        threadManager.AddTask(arbiterCallback, islands[i]);
                    }
                }

                threadManager.Execute();
            }
            else
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if (islands[i].IsActive())
                    {
                        arbiterCallback(islands[i]);
                    }
                }
            }
        }

        private void IntegrateForces()
        {
            foreach (var body in rigidBodies)
            {
                if (!body.isStatic && body.IsActive)
                {
                    JVector.Multiply(body.force, body.inverseMass * timestep, out var temp);
                    JVector.Add(temp, body.linearVelocity, out body.linearVelocity);

                    if (!body.isParticle)
                    {
                        JVector.Multiply(body.torque, timestep, out temp);
                        JVector.Transform(temp, body.invInertiaWorld, out temp);
                        JVector.Add(temp, body.angularVelocity, out body.angularVelocity);
                    }

                    if (body.affectedByGravity)
                    {
                        JVector.Multiply(gravity, timestep, out temp);
                        JVector.Add(body.linearVelocity, temp, out body.linearVelocity);
                    }
                }

                body.force = new JVector();
                body.torque = new JVector();
            }
        }

        private void IntegrateCallback(object obj)
        {
            var body = obj as RigidBody;

            JVector.Multiply(body.linearVelocity, timestep, out var temp);
            JVector.Add(temp, body.position, out body.position);

            if (!body.isParticle)
            {
                JVector axis;
                float angle = body.angularVelocity.Length();

                if (angle < 0.001f)
                {
                    JVector.Multiply(body.angularVelocity, (0.5f * timestep) - (timestep * timestep * timestep * 0.020833333333f * angle * angle), out axis);
                }
                else
                {
                    JVector.Multiply(body.angularVelocity, (float)Math.Sin(0.5f * angle * timestep) / angle, out axis);
                }

                var dorn = new JQuaternion(axis.X, axis.Y, axis.Z, (float)Math.Cos(angle * timestep * 0.5f));
                JQuaternion.CreateFromMatrix(ref body.orientation, out var ornA);

                JQuaternion.Multiply(ref dorn, ref ornA, out dorn);

                dorn.Normalize(); JMatrix.CreateFromQuaternion(ref dorn, out body.orientation);
            }

            if ((body.Damping & RigidBody.DampingType.Linear) != 0)
            {
                JVector.Multiply(body.linearVelocity, currentLinearDampFactor, out body.linearVelocity);
            }

            if ((body.Damping & RigidBody.DampingType.Angular) != 0)
            {
                JVector.Multiply(body.angularVelocity, currentAngularDampFactor, out body.angularVelocity);
            }

            body.Update();


            if (CollisionSystem.EnableSpeculativeContacts || body.EnableSpeculativeContacts)
            {
                body.SweptExpandBoundingBox(timestep);
            }
        }

        private void Integrate(bool multithread)
        {
            if (multithread)
            {
                foreach (var body in rigidBodies)
                {
                    if (body.isStatic || !body.IsActive)
                    {
                        continue;
                    }

                    threadManager.AddTask(integrateCallback, body);
                }

                threadManager.Execute();
            }
            else
            {
                foreach (var body in rigidBodies)
                {
                    if (body.isStatic || !body.IsActive)
                    {
                        continue;
                    }

                    integrateCallback(body);
                }
            }
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, JVector point1, JVector point2, JVector normal, float penetration)
        {
            Arbiter arbiter = null;
            Contact contact;

            lock (ArbiterMap)
            {
                ArbiterMap.LookUpArbiter(body1, body2, out arbiter);
                if (arbiter == null)
                {
                    arbiter = Arbiter.Pool.GetNew();
                    arbiter.body1 = body1; arbiter.body2 = body2;
                    ArbiterMap.Add(new ArbiterKey(body1, body2), arbiter);

                    addedArbiterQueue.Enqueue(arbiter);

                    Events.RaiseBodiesBeginCollide(body1, body2);
                }
            }

            if (arbiter.body1 == body1)
            {
                JVector.Negate(normal, out normal);
                contact = arbiter.AddContact(point1, point2, normal, penetration, ContactSettings);
            }
            else
            {
                contact = arbiter.AddContact(point2, point1, normal, penetration, ContactSettings);
            }

            if (contact != null)
            {
                Events.RaiseContactCreated(contact);
            }
        }

        private void CheckDeactivation()
        {

            foreach (var island in islands)
            {
                bool deactivateIsland = true;

                if (!AllowDeactivation)
                {
                    deactivateIsland = false;
                }
                else
                {
                    foreach (var body in island.bodies)
                    {
                        if (body.AllowDeactivation && body.angularVelocity.LengthSquared() < inactiveAngularThresholdSq
                            && (body.linearVelocity.LengthSquared() < inactiveLinearThresholdSq))
                        {
                            body.inactiveTime += timestep;
                            if (body.inactiveTime < deactivationTime)
                            {
                                deactivateIsland = false;
                            }
                        }
                        else
                        {
                            body.inactiveTime = 0.0f;
                            deactivateIsland = false;
                        }
                    }
                }

                foreach (var body in island.bodies)
                {
                    if (body.isActive == deactivateIsland)
                    {
                        if (body.isActive)
                        {
                            body.IsActive = false;
                            Events.RaiseDeactivatedBody(body);
                        }
                        else
                        {
                            body.IsActive = true;
                            Events.RaiseActivatedBody(body);
                        }
                    }
                }
            }
        }
    }
}
