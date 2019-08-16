﻿/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision;
using Jitter.Dynamics.Constraints;
using Jitter.DataStructures;
#endregion

namespace Jitter
{
    /// <summary>
    /// This class brings 'dynamics' and 'collisions' together. It handles
    /// all bodies and constraints.
    /// </summary>
    public class World
    {
        public delegate void WorldStep(float timestep);

        public class WorldEvents
        {
            // Post&Prestep
            public event WorldStep PreStep;
            public event WorldStep PostStep;

            // Add&Remove
            public event Action<RigidBody> AddedRigidBody;
            public event Action<RigidBody> RemovedRigidBody;
            public event Action<Constraint> AddedConstraint;
            public event Action<Constraint> RemovedConstraint;
            public event Action<SoftBody> AddedSoftBody;
            public event Action<SoftBody> RemovedSoftBody;

            // Collision
            public event Action<RigidBody, RigidBody> BodiesBeginCollide;
            public event Action<RigidBody, RigidBody> BodiesEndCollide;
            public event Action<Contact> ContactCreated;

            // Deactivation
            public event Action<RigidBody> DeactivatedBody;
            public event Action<RigidBody> ActivatedBody;

            internal WorldEvents() { }

            #region Raise Events

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

            internal void RaiseBodiesBeginCollide(RigidBody body1,RigidBody body2)
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

            #endregion
        }

        private float inactiveAngularThresholdSq = 0.1f;
        private float inactiveLinearThresholdSq = 0.1f;
        private float deactivationTime = 2f;

        private float angularDamping = 0.85f;
        private float linearDamping = 0.85f;

        private int contactIterations = 10;
        private int smallIterations = 4;
        private float timestep = 0.0f;

        private readonly Jitter.Collision.IslandManager islands = new IslandManager();

        private readonly HashSet<RigidBody> rigidBodies = new HashSet<RigidBody>();
        private readonly HashSet<Constraint> constraints = new HashSet<Constraint>();
        private readonly HashSet<SoftBody> softbodies = new HashSet<SoftBody>();

        public ReadOnlyHashset<RigidBody> RigidBodies { get; }
        public ReadOnlyHashset<Constraint> Constraints { get; }
        public ReadOnlyHashset<SoftBody> SoftBodies { get; }
        public WorldEvents Events { get; } = new WorldEvents();

        private readonly ThreadManager threadManager = ThreadManager.Instance;

        /// <summary>
        /// Holds a list of <see cref="Arbiter"/>. All currently
        /// active arbiter in the <see cref="World"/> are stored in this map.
        /// </summary>
        public ArbiterMap ArbiterMap { get; }

        private readonly Queue<Arbiter> removedArbiterQueue = new Queue<Arbiter>();
        private readonly Queue<Arbiter> addedArbiterQueue = new Queue<Arbiter>();

        private JVector gravity = new JVector(0, -9.81f, 0);

        public ContactSettings ContactSettings { get; } = new ContactSettings();

        /// <summary>
        /// Gets a read only collection of the <see cref="Jitter.Collision.CollisionIsland"/> objects managed by
        /// this class.
        /// </summary>
        public ReadOnlyCollection<CollisionIsland> Islands { get { return islands; } }

        private readonly Action<object> arbiterCallback;
        private readonly Action<object> integrateCallback;

        private readonly CollisionDetectedHandler collisionDetectionHandler;

        /// <summary>
        /// Create a new instance of the <see cref="World"/> class.
        /// </summary>
        /// <param name="collision">The collisionSystem which is used to detect
        /// collisions. See for example: <see cref="CollisionSystemSAP"/>
        /// or <see cref="CollisionSystemBrute"/>.
        /// </param>
        public World(CollisionSystem collision)
        {
            if (collision == null)
                throw new ArgumentNullException("The CollisionSystem can't be null.", "collision");

            arbiterCallback = new Action<object>(ArbiterCallback);
            integrateCallback = new Action<object>(IntegrateCallback);

            // Create the readonly wrappers
            RigidBodies = new ReadOnlyHashset<RigidBody>(rigidBodies);
            Constraints = new ReadOnlyHashset<Constraint>(constraints);
            SoftBodies = new ReadOnlyHashset<SoftBody>(softbodies);

            CollisionSystem = collision;

            collisionDetectionHandler = new CollisionDetectedHandler(CollisionDetected);

            CollisionSystem.CollisionDetected += collisionDetectionHandler;

            ArbiterMap = new ArbiterMap();

            AllowDeactivation = true;
        }

        public void AddBody(SoftBody body)
        {
            if (body == null) throw new ArgumentNullException("body", "body can't be null.");
            if (softbodies.Contains(body)) throw new ArgumentException("The body was already added to the world.", "body");

            softbodies.Add(body);
            CollisionSystem.AddEntity(body);

            Events.RaiseAddedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
                AddConstraint(constraint);

            foreach (var massPoint in body.VertexBodies)
            {
                Events.RaiseAddedRigidBody(massPoint);
                rigidBodies.Add(massPoint);
            }
        }

        public bool RemoveBody(SoftBody body)
        {
            if (!softbodies.Remove(body)) return false;

            CollisionSystem.RemoveEntity(body);

            Events.RaiseRemovedSoftBody(body);

            foreach (Constraint constraint in body.EdgeSprings)
                RemoveConstraint(constraint);

            foreach (var massPoint in body.VertexBodies)
                RemoveBody(massPoint, true);

            return true;
        }

        /// <summary>
        /// Gets the <see cref="CollisionSystem"/> used
        /// to detect collisions.
        /// </summary>
        public CollisionSystem CollisionSystem { set; get; }

        /// <summary>
        /// In Jitter many objects get added to stacks after they were used.
        /// If a new object is needed the old object gets removed from the stack
        /// and is reused. This saves some time and also garbage collections.
        /// Calling this method removes all cached objects from all
        /// stacks.
        /// </summary>
        public void ResetResourcePools()
        {
            IslandManager.Pool.ResetResourcePool();
            Arbiter.Pool.ResetResourcePool();
            Contact.Pool.ResetResourcePool();
        }

        /// <summary>
        /// Removes all objects from the world and removes all memory cached objects.
        /// </summary>
        public void Clear()
        {
            // remove bodies from collision system
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

            // remove bodies from the world
            rigidBodies.Clear();

            // remove constraints
            foreach (var constraint in constraints)
            {
                Events.RaiseRemovedConstraint(constraint);
            }
            constraints.Clear();

            softbodies.Clear();

            // remove all islands
            islands.RemoveAll();

            // delete the arbiters
            ArbiterMap.Clear();

            ResetResourcePools();
        }

        /// <summary>
        /// Gets or sets the gravity in this <see cref="World"/>. The default gravity
        /// is (0,-9.81,0)
        /// </summary>
        public JVector Gravity { get { return gravity; } set { gravity = value; } }

        /// <summary>
        /// Global sets or gets if a body is able to be temporarily deactivated by the engine to
        /// safe computation time. Use <see cref="SetInactivityThreshold"/> to set parameters
        /// of the deactivation process.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        /// <summary>
        /// Every computation <see cref="Step"/> the angular and linear velocity 
        /// of a <see cref="RigidBody"/> gets multiplied by this value.
        /// </summary>
        /// <param name="angularDamping">The factor multiplied with the angular velocity.
        /// The default value is 0.85.</param>
        /// <param name="linearDamping">The factor multiplied with the linear velocity.
        /// The default value is 0.85</param>
        public void SetDampingFactors(float angularDamping, float linearDamping)
        {
            if (angularDamping < 0.0f || angularDamping > 1.0f)
                throw new ArgumentException("Angular damping factor has to be between 0.0 and 1.0", "angularDamping");

            if (linearDamping < 0.0f || linearDamping > 1.0f)
                throw new ArgumentException("Linear damping factor has to be between 0.0 and 1.0", "linearDamping");

            this.angularDamping = angularDamping;
            this.linearDamping = linearDamping;
        }

        /// <summary>
        /// Sets parameters for the <see cref="RigidBody"/> deactivation process.
        /// If the bodies angular velocity is less than the angular velocity threshold
        /// and its linear velocity is lower then the linear velocity threshold for a 
        /// specific time the body gets deactivated. A body can be reactivated by setting
        /// <see cref="RigidBody.IsActive"/> to true. A body gets also automatically
        /// reactivated if another moving object hits it or the <see cref="CollisionIsland"/>
        /// the object is in gets activated.
        /// </summary>
        /// <param name="angularVelocity">The threshold value for the angular velocity. The default value
        /// is 0.1.</param>
        /// <param name="linearVelocity">The threshold value for the linear velocity. The default value
        /// is 0.1</param>
        /// <param name="time">The threshold value for the time in seconds. The default value is 2.</param>
        public void SetInactivityThreshold(float angularVelocity, float linearVelocity, float time)
        {
            if (angularVelocity < 0.0f) throw new ArgumentException("Angular velocity threshold has to " +
                 "be larger than zero", "angularVelocity");

            if (linearVelocity < 0.0f) throw new ArgumentException("Linear velocity threshold has to " +
                "be larger than zero", "linearVelocity");

            if (time < 0.0f) throw new ArgumentException("Deactivation time threshold has to " +
                "be larger than zero", "time");

            inactiveAngularThresholdSq = angularVelocity * angularVelocity;
            inactiveLinearThresholdSq = linearVelocity * linearVelocity;
            deactivationTime = time;
        }

        /// <summary>
        /// Jitter uses an iterativ approach to solve collisions and contacts. You can set the number of
        /// iterations Jitter should do. In general the more iterations the more stable a simulation gets
        /// but also costs computation time.
        /// </summary>
        /// <param name="iterations">The number of contact iterations. Default value 10.</param>
        /// <param name="smallIterations">The number of contact iteration used for smaller (two and three constraint) systems. Default value 4.</param>
        /// <remarks>The number of iterations for collision and contact should be between 3 - 30.
        /// More iterations means more stability and also a longer calculation time.</remarks>
        public void SetIterations(int iterations, int smallIterations)
        {
            if (iterations < 1) throw new ArgumentException("The number of collision " +
                 "iterations has to be larger than zero", "iterations");

            if (smallIterations < 1) throw new ArgumentException("The number of collision " +
                "iterations has to be larger than zero", "smallIterations");

            contactIterations = iterations;
            this.smallIterations = smallIterations;
        }

        /// <summary>
        /// Removes a <see cref="RigidBody"/> from the world.
        /// </summary>
        /// <param name="body">The body which should be removed.</param>
        /// <returns>Returns false if the body could not be removed from the world.</returns>
        public bool RemoveBody(RigidBody body)
        {
            return RemoveBody(body, false);
        }

        private bool RemoveBody(RigidBody body, bool removeMassPoints)
        {
            // Its very important to clean up, after removing a body
            if (!removeMassPoints && body.IsParticle) return false;

            // remove the body from the world list
            if (!rigidBodies.Remove(body)) return false;

            // Remove all connected constraints and arbiters
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

            // remove the body from the collision system
            CollisionSystem.RemoveEntity(body);

            // remove the body from the island manager
            islands.RemoveBody(body);

            Events.RaiseRemovedRigidBody(body);

            return true;
        }

        /// <summary>
        /// Adds a <see cref="RigidBody"/> to the world.
        /// </summary>
        /// <param name="body">The body which should be added.</param>
        public void AddBody(RigidBody body)
        {
            if (body == null) throw new ArgumentNullException("body", "body can't be null.");
            if(rigidBodies.Contains(body)) throw new ArgumentException("The body was already added to the world.", "body");

            Events.RaiseAddedRigidBody(body);

            CollisionSystem.AddEntity(body);

            rigidBodies.Add(body);
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world. Fast, O(1).
        /// </summary>
        /// <param name="constraint">The constraint which should be added.</param>
        /// <returns>True if the constraint was successfully removed.</returns>
        public bool RemoveConstraint(Constraint constraint)
        {
            if (!constraints.Remove(constraint)) return false;
            Events.RaiseRemovedConstraint(constraint);

            islands.ConstraintRemoved(constraint);

            return true;
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world.
        /// </summary>
        /// <param name="constraint">The constraint which should be removed.</param>
        public void AddConstraint(Constraint constraint)
        {
            if(constraints.Contains(constraint)) 
                throw new ArgumentException("The constraint was already added to the world.", "constraint");

            constraints.Add(constraint);

            islands.ConstraintCreated(constraint);

            Events.RaiseAddedConstraint(constraint);
        }

        private float currentLinearDampFactor = 1.0f;
        private float currentAngularDampFactor = 1.0f;

#if (!WINDOWS_PHONE)
        private readonly Stopwatch sw = new Stopwatch();

        public enum DebugType
        {
            CollisionDetect, BuildIslands, HandleArbiter, UpdateContacts,
            PreStep, DeactivateBodies, IntegrateForces, Integrate, PostStep, ClothUpdate, Num
        }

        public double[] DebugTimes { get; } = new double[(int)DebugType.Num];
#endif

        /// <summary>
        /// Integrates the whole world a timestep further in time.
        /// </summary>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="multithread">If true the engine uses several threads to
        /// integrate the world. This is faster on multicore CPUs.</param>
        public void Step(float timestep, bool multithread)
        {
            this.timestep = timestep;

            // yeah! nothing to do!
            if (timestep == 0.0f) return;

            // throw exception if the timestep is smaller zero.
            if (timestep < 0.0f) throw new ArgumentException("The timestep can't be negative.", "timestep");

            // Calculate this
            currentAngularDampFactor = (float)Math.Pow(angularDamping, timestep);
            currentLinearDampFactor = (float)Math.Pow(linearDamping, timestep);

#if(WINDOWS_PHONE)
            events.RaiseWorldPreStep(timestep);
            foreach (RigidBody body in rigidBodies) body.PreStep(timestep);
            UpdateContacts();

            while (removedArbiterQueue.Count > 0) islands.ArbiterRemoved(removedArbiterQueue.Dequeue());

            foreach (SoftBody body in softbodies)
            {
                body.Update(timestep);
                body.DoSelfCollision(collisionDetectionHandler);
            }

            CollisionSystem.Detect(multithread);
           
            while (addedArbiterQueue.Count > 0) islands.ArbiterCreated(addedArbiterQueue.Dequeue());

            CheckDeactivation();

            IntegrateForces();
            HandleArbiter(contactIterations, multithread);
            Integrate(multithread);

            foreach (RigidBody body in rigidBodies) body.PostStep(timestep);
            events.RaiseWorldPostStep(timestep);
#else
            sw.Reset(); sw.Start();
            Events.RaiseWorldPreStep(timestep);
            foreach (var body in rigidBodies) body.PreStep(timestep);

            sw.Stop(); DebugTimes[(int)DebugType.PreStep] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            UpdateContacts();
            sw.Stop(); DebugTimes[(int)DebugType.UpdateContacts] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            double ms = 0;
            while (removedArbiterQueue.Count > 0) islands.ArbiterRemoved(removedArbiterQueue.Dequeue());
            sw.Stop(); ms = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            foreach (var body in softbodies)
            {
                body.Update(timestep);
                body.DoSelfCollision(collisionDetectionHandler);
            }
            sw.Stop(); DebugTimes[(int)DebugType.ClothUpdate] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            CollisionSystem.Detect(multithread);
            sw.Stop(); DebugTimes[(int)DebugType.CollisionDetect] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();

            while (addedArbiterQueue.Count > 0) islands.ArbiterCreated(addedArbiterQueue.Dequeue());

            sw.Stop(); DebugTimes[(int)DebugType.BuildIslands] = sw.Elapsed.TotalMilliseconds + ms;

            sw.Reset(); sw.Start();
            CheckDeactivation();
            sw.Stop(); DebugTimes[(int)DebugType.DeactivateBodies] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            IntegrateForces();
            sw.Stop(); DebugTimes[(int)DebugType.IntegrateForces] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            HandleArbiter(contactIterations, multithread);
            sw.Stop(); DebugTimes[(int)DebugType.HandleArbiter] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            Integrate(multithread);
            sw.Stop(); DebugTimes[(int)DebugType.Integrate] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            foreach (var body in rigidBodies) body.PostStep(timestep);
            Events.RaiseWorldPostStep(timestep);
            sw.Stop(); DebugTimes[(int)DebugType.PostStep] = sw.Elapsed.TotalMilliseconds;
#endif
        }

        private float accumulatedTime = 0.0f;

        /// <summary>
        /// Integrates the whole world several fixed timestep further in time.
        /// </summary>
        /// <param name="totalTime">The time to integrate.</param>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="multithread">If true the engine uses several threads to
        /// integrate the world. This is faster on multicore CPUs.</param>
        /// <param name="maxSteps">The maximum number of substeps. After that Jitter gives up
        /// to keep up with the given totalTime.</param>
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
                    // okay, okay... we can't keep up
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
                    JVector.Subtract(ref c.p1, ref c.p2, out var diff);
                    float distance = JVector.Dot(ref diff, ref c.normal);

                    diff = diff - (distance * c.normal);
                    distance = diff.LengthSquared();

                    // hack (multiplication by factor 100) in the
                    // following line.
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

        #region private void ArbiterCallback(object obj)
        private void ArbiterCallback(object obj)
        {
            var island = obj as CollisionIsland;

            int thisIterations;
            if (island.Bodies.Count + island.Constraints.Count > 3) thisIterations = contactIterations;
            else thisIterations = smallIterations;

            for (int i = -1; i < thisIterations; i++)
            {
                // Contact and Collision
                foreach (var arbiter in island.arbiter)
                {
                    int contactCount = arbiter.contactList.Count;
                    for (int e = 0; e < contactCount; e++)
                    {
                        if (i == -1) arbiter.contactList[e].PrepareForIteration(timestep);
                        else arbiter.contactList[e].Iterate();
                    }
                }

                //  Constraints
                foreach (var c in island.constraints)
                {
                    if (c.body1 != null && !c.body1.IsActive && c.body2 != null && !c.body2.IsActive)
                        continue;

                    if (i == -1) c.PrepareForIteration(timestep);
                    else c.Iterate();
                }
            }
        }
        #endregion

        private void HandleArbiter(int iterations, bool multiThreaded)
        {
            if (multiThreaded)
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if(islands[i].IsActive()) threadManager.AddTask(arbiterCallback, islands[i]);
                }

                threadManager.Execute();
            }
            else
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if (islands[i].IsActive()) arbiterCallback(islands[i]);
                }
            }
        }

        private void IntegrateForces()
        {
            foreach (var body in rigidBodies)
            {
                if (!body.isStatic && body.IsActive)
                {
                    JVector.Multiply(ref body.force, body.inverseMass * timestep, out var temp);
                    JVector.Add(ref temp, ref body.linearVelocity, out body.linearVelocity);

                    if (!(body.isParticle))
                    {
                        JVector.Multiply(ref body.torque, timestep, out temp);
                        JVector.Transform(ref temp, ref body.invInertiaWorld, out temp);
                        JVector.Add(ref temp, ref body.angularVelocity, out body.angularVelocity);
                    }

                    if (body.affectedByGravity)
                    {
                        JVector.Multiply(ref gravity, timestep, out temp);
                        JVector.Add(ref body.linearVelocity, ref temp, out body.linearVelocity);
                    }
                }

                body.force.MakeZero();
                body.torque.MakeZero();
            }
        }

        #region private void IntegrateCallback(object obj)
        private void IntegrateCallback(object obj)
        {
            var body = obj as RigidBody;

            JVector.Multiply(ref body.linearVelocity, timestep, out var temp);
            JVector.Add(ref temp, ref body.position, out body.position);

            if (!(body.isParticle))
            {
                //exponential map
                JVector axis;
                float angle = body.angularVelocity.Length();

                if (angle < 0.001f)
                {
                    // use Taylor's expansions of sync function
                    // axis = body.angularVelocity * (0.5f * timestep - (timestep * timestep * timestep) * (0.020833333333f) * angle * angle);
                    JVector.Multiply(ref body.angularVelocity, ((0.5f * timestep) - ((timestep * timestep * timestep) * (0.020833333333f) * angle * angle)), out axis);
                }
                else
                {
                    // sync(fAngle) = sin(c*fAngle)/t
                    JVector.Multiply(ref body.angularVelocity, ((float)Math.Sin(0.5f * angle * timestep) / angle), out axis);
                }

                var dorn = new JQuaternion(axis.X, axis.Y, axis.Z, (float)Math.Cos(angle * timestep * 0.5f));
                JQuaternion.CreateFromMatrix(ref body.orientation, out var ornA);

                JQuaternion.Multiply(ref dorn, ref ornA, out dorn);

                dorn.Normalize(); JMatrix.CreateFromQuaternion(ref dorn, out body.orientation);
            }

            if ((body.Damping & RigidBody.DampingType.Linear) != 0)
                JVector.Multiply(ref body.linearVelocity, currentLinearDampFactor, out body.linearVelocity);

            if ((body.Damping & RigidBody.DampingType.Angular) != 0)
                JVector.Multiply(ref body.angularVelocity, currentAngularDampFactor, out body.angularVelocity);

            body.Update();

            
            if (CollisionSystem.EnableSpeculativeContacts || body.EnableSpeculativeContacts)
                body.SweptExpandBoundingBox(timestep);
        }
        #endregion

        private void Integrate(bool multithread)
        {
            if (multithread)
            {
                foreach (var body in rigidBodies)
                {
                    if (body.isStatic || !body.IsActive) continue;
                    threadManager.AddTask(integrateCallback, body);
                }

                threadManager.Execute();
            }
            else
            {
                foreach (var body in rigidBodies)
                {
                    if (body.isStatic || !body.IsActive) continue;
                    integrateCallback(body);
                }
            }
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, JVector point1, JVector point2, JVector normal, float penetration)
        {
            Arbiter arbiter = null;

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

            Contact contact = null;

            if (arbiter.body1 == body1)
            {
                JVector.Negate(ref normal, out normal);
                contact = arbiter.AddContact(point1, point2, normal, penetration, ContactSettings);
            }
            else
            {
                contact = arbiter.AddContact(point2, point1, normal, penetration, ContactSettings);
            }

            if (contact != null) Events.RaiseContactCreated(contact);
        }

        private void CheckDeactivation()
        {
            // A body deactivation DOESN'T kill the contacts - they are stored in
            // the arbitermap within the arbiters. So, waking up ist STABLE - old
            // contacts are reused. Also the collisionislands build every frame (based 
            // on the contacts) keep the same.

            foreach (var island in islands)
            {
                bool deactivateIsland = true;

                // global allowdeactivation
                if (!AllowDeactivation) deactivateIsland = false;
                else
                {
                    foreach (var body in island.bodies)
                    {
                        // body allowdeactivation
                        if (body.AllowDeactivation && (body.angularVelocity.LengthSquared() < inactiveAngularThresholdSq &&
                        (body.linearVelocity.LengthSquared() < inactiveLinearThresholdSq)))
                        {
                            body.inactiveTime += timestep;
                            if (body.inactiveTime < deactivationTime)
                                deactivateIsland = false;
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
