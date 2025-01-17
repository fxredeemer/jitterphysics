﻿using Jitter.Collision;
using Jitter.Collision.Shapes;
using Jitter.DataStructures;
using Jitter.Dynamics.Constraints;
using Jitter.LinearMath;
using System;
using System.Collections.Generic;
using System.Threading;

namespace Jitter.Dynamics
{
    public enum RigidBodyIndex
    {
        RigidBody1, RigidBody2
    }

    public class RigidBody : IBroadphaseEntity, IDebugDrawable, IEquatable<RigidBody>, IComparable<RigidBody>
    {
        [Flags]
        public enum DampingType { None = 0x00, Angular = 0x01, Linear = 0x02 }

        internal JMatrix inertia;
        internal JMatrix invInertia;

        internal JMatrix invInertiaWorld;
        internal JMatrix orientation;
        internal JMatrix invOrientation;
        internal JVector position;
        internal JVector linearVelocity;
        internal JVector angularVelocity;

        internal Material material;

        internal JBBox boundingBox;

        internal float inactiveTime;

        internal bool isActive = true;
        internal bool isStatic;
        internal bool affectedByGravity = true;

        internal CollisionIsland island;
        internal float inverseMass;

        internal JVector force, torque;

        private readonly int hashCode;

        private ShapeUpdatedHandler updatedHandler;

        internal List<RigidBody> connections = new List<RigidBody>();

        internal HashSet<Arbiter> arbiters = new HashSet<Arbiter>();
        internal HashSet<Constraint> constraints = new HashSet<Constraint>();
        internal int marker;

        public RigidBody(Shape shape)
            : this(shape, new Material(), false)
        {
        }

        internal bool isParticle;

        public bool IsParticle
        {
            get => isParticle;
            set
            {
                if (isParticle && !value)
                {
                    updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);
                    Shape.ShapeUpdated += updatedHandler;
                    SetMassProperties();
                    isParticle = false;
                }
                else if (!isParticle && value)
                {
                    inertia = JMatrix.Zero;
                    invInertia = invInertiaWorld = JMatrix.Zero;
                    invOrientation = orientation = JMatrix.Identity;
                    inverseMass = 1.0f;

                    Shape.ShapeUpdated -= updatedHandler;

                    isParticle = true;
                }

                Update();
            }
        }

        public RigidBody(Shape shape, Material material) : this(shape, material, false)
        {
        }

        public RigidBody(Shape shape, Material material, bool isParticle)
        {
            Arbiters = new ReadOnlyHashset<Arbiter>(arbiters);
            Constraints = new ReadOnlyHashset<Constraint>(constraints);

            instance = Interlocked.Increment(ref instanceCount);
            hashCode = CalculateHash(instance);

            Shape = shape;
            orientation = JMatrix.Identity;

            if (!isParticle)
            {
                updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);
                Shape.ShapeUpdated += updatedHandler;
                SetMassProperties();
            }
            else
            {
                inertia = JMatrix.Zero;
                invInertia = invInertiaWorld = JMatrix.Zero;
                invOrientation = orientation = JMatrix.Identity;
                inverseMass = 1.0f;
            }

            this.material = material;

            AllowDeactivation = true;
            EnableSpeculativeContacts = false;

            this.isParticle = isParticle;

            Update();
        }

        public override int GetHashCode()
        {
            return hashCode;
        }

        public ReadOnlyHashset<Arbiter> Arbiters { get; }
        public ReadOnlyHashset<Constraint> Constraints { get; }

        public bool AllowDeactivation { get; set; }

        public bool EnableSpeculativeContacts { get; set; }

        public JBBox BoundingBox => boundingBox;

        private static int instanceCount;
        private readonly int instance;

        private int CalculateHash(int a)
        {
            a = a ^ 61 ^ (a >> 16);
            a += a << 3;
            a ^= a >> 4;
            a *= 0x27d4eb2d;
            a ^= a >> 15;
            return a;
        }

        public CollisionIsland CollisionIsland => island;

        public bool IsActive
        {
            get => isActive;
            set
            {
                if (!isActive && value)
                {
                    inactiveTime = 0.0f;
                }
                else if (isActive && !value)
                {
                    inactiveTime = float.PositiveInfinity;
                    angularVelocity = JVector.Zero;
                    linearVelocity = JVector.Zero;
                }

                isActive = value;
            }
        }

        public void ApplyImpulse(JVector impulse)
        {
            if (isStatic)
            {
                throw new InvalidOperationException("Can't apply an impulse to a static body.");
            }

            JVector.Multiply(impulse, inverseMass, out var temp);
            JVector.Add(linearVelocity, temp, out linearVelocity);
        }

        public void ApplyImpulse(JVector impulse, JVector relativePosition)
        {
            if (isStatic)
            {
                throw new InvalidOperationException("Can't apply an impulse to a static body.");
            }

            JVector.Multiply(impulse, inverseMass, out var temp);
            JVector.Add(linearVelocity, temp, out linearVelocity);
            JVector.Cross(relativePosition, impulse, out temp);
            JVector.Transform(temp, invInertiaWorld, out temp);
            JVector.Add(angularVelocity, temp, out angularVelocity);
        }

        public void AddForce(JVector force)
        {
            JVector.Add(force, this.force, out this.force);
        }

        public void AddForce(JVector force, JVector pos)
        {
            JVector.Add(this.force, force, out this.force);
            JVector.Subtract(pos, position, out pos);
            JVector.Cross(pos, force, out pos);
            JVector.Add(pos, torque, out torque);
        }

        public JVector Torque => torque;

        public JVector Force { get => force; set => force = value; }

        public void AddTorque(JVector torque)
        {
            JVector.Add(torque, this.torque, out this.torque);
        }

        protected bool useShapeMassProperties = true;

        public void SetMassProperties()
        {
            inertia = Shape.inertia;
            JMatrix.Inverse(inertia, out invInertia);
            inverseMass = 1.0f / Shape.mass;
            useShapeMassProperties = true;
        }

        public void SetMassProperties(JMatrix inertia, float mass, bool setAsInverseValues)
        {
            if (setAsInverseValues)
            {
                if (!isParticle)
                {
                    invInertia = inertia;
                    JMatrix.Inverse(inertia, out this.inertia);
                }
                inverseMass = mass;
            }
            else
            {
                if (!isParticle)
                {
                    this.inertia = inertia;
                    JMatrix.Inverse(inertia, out invInertia);
                }
                inverseMass = 1.0f / mass;
            }

            useShapeMassProperties = false;
            Update();
        }

        private void ShapeUpdated()
        {
            if (useShapeMassProperties)
            {
                SetMassProperties();
            }

            Update();
            UpdateHullData();
        }

        public object Tag { get; set; }

        public Shape Shape
        {
            get => shape;
            set
            {
                if (shape != null)
                {
                    shape.ShapeUpdated -= updatedHandler;
                }

                shape = value;
                shape.ShapeUpdated += new ShapeUpdatedHandler(ShapeUpdated);
            }
        }

        private Shape shape;

        public DampingType Damping { get; set; } = DampingType.Angular | DampingType.Linear;

        public Material Material { get => material; set => material = value; }

        public JMatrix Inertia => inertia;

        public JMatrix InverseInertia => invInertia;

        public JVector LinearVelocity
        {
            get => linearVelocity;
            set
            {
                if (isStatic)
                {
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                }

                linearVelocity = value;
            }
        }

        public JVector AngularVelocity
        {
            get => angularVelocity;
            set
            {
                if (isStatic)
                {
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                }

                angularVelocity = value;
            }
        }

        public JVector Position
        {
            get => position;
            set
            {
                position = value;
                Update();
            }
        }

        public JMatrix Orientation
        {
            get => orientation;
            set
            {
                orientation = value;
                Update();
            }
        }

        public bool IsStatic
        {
            get => isStatic;
            set
            {
                if (value && !isStatic)
                {
                    if (island != null)
                    {
                        island.islandManager.MakeBodyStatic(this);
                    }

                    angularVelocity = JVector.Zero;
                    linearVelocity = JVector.Zero;
                }
                isStatic = value;
            }
        }

        public bool AffectedByGravity { get => affectedByGravity; set => affectedByGravity = value; }

        public JMatrix InverseInertiaWorld => invInertiaWorld;

        public float Mass
        {
            get => 1.0f / inverseMass;
            set
            {
                if (value <= 0.0f)
                {
                    throw new ArgumentException("Mass can't be less or equal zero.");
                }

                if (!isParticle)
                {
                    JMatrix.Multiply(Shape.inertia, value / Shape.mass, out inertia);
                    JMatrix.Inverse(inertia, out invInertia);
                }

                inverseMass = 1.0f / value;
            }
        }

        internal JVector sweptDirection = JVector.Zero;

        public void SweptExpandBoundingBox(float timestep)
        {
            sweptDirection = linearVelocity * timestep;

            float deltaMinX = 0.0f;
            float deltaMaxX = 0.0f;
            float deltaMinY = 0.0f;
            float deltaMaxY = 0.0f;
            float deltaMinZ = 0.0f;
            float deltaMaxZ = 0.0f;

            if (sweptDirection.X < 0.0f)
            {
                deltaMinX = sweptDirection.X;
            }
            else
            {
                deltaMaxX = sweptDirection.X;
            }

            if (sweptDirection.Y < 0.0f)
            {
                deltaMinY = sweptDirection.Y;
            }
            else
            {
                deltaMaxY = sweptDirection.Y;
            }

            if (sweptDirection.Z < 0.0f)
            {
                deltaMinZ = sweptDirection.Z;
            }
            else
            {
                deltaMaxZ = sweptDirection.Z;
            }

            boundingBox = new JBBox(
                new JVector(boundingBox.Min.X + deltaMinX, boundingBox.Min.Y + deltaMinY, boundingBox.Min.Z + deltaMinZ),
                new JVector(boundingBox.Max.X + deltaMaxX, boundingBox.Max.Y + deltaMaxY, boundingBox.Max.Z + deltaMaxZ));
        }

        public virtual void Update()
        {
            if (isParticle)
            {
                inertia = JMatrix.Zero;
                invInertia = invInertiaWorld = JMatrix.Zero;
                invOrientation = orientation = JMatrix.Identity;
                boundingBox = shape.boundingBox;

                boundingBox = new JBBox(
                    JVector.Add(boundingBox.Min, position),
                    JVector.Add(boundingBox.Max, position));

                angularVelocity = new JVector();
            }
            else
            {
                JMatrix.Transpose(orientation, out invOrientation);
                Shape.GetBoundingBox( orientation, out boundingBox);
                
                boundingBox = new JBBox(
                    JVector.Add(boundingBox.Min, position),
                    JVector.Add(boundingBox.Max, position));

                if (!isStatic)
                {
                    JMatrix.Multiply(invOrientation, invInertia, out invInertiaWorld);
                    JMatrix.Multiply(invInertiaWorld, orientation, out invInertiaWorld);
                }
            }
        }

        public bool Equals(RigidBody other)
        {
            return other.instance == instance;
        }

        public int CompareTo(RigidBody other)
        {
            if (other.instance < instance)
            {
                return -1;
            }
            else if (other.instance > instance)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }

        public int BroadphaseTag { get; set; }

        public virtual void PreStep(float timestep)
        {
        }

        public virtual void PostStep(float timestep)
        {
        }

        public bool IsStaticOrInactive => !isActive || isStatic;

        private bool enableDebugDraw;
        public bool EnableDebugDraw
        {
            get => enableDebugDraw;
            set
            {
                enableDebugDraw = value;
                UpdateHullData();
            }
        }

        private List<JVector> hullPoints = new List<JVector>();

        private void UpdateHullData()
        {
            hullPoints.Clear();

            if (enableDebugDraw)
            {
                shape.MakeHull(hullPoints, 3);
            }
        }

        public void DebugDraw(IDebugDrawer drawer)
        {
            JVector pos1, pos2, pos3;

            for (int i = 0; i < hullPoints.Count; i += 3)
            {
                pos1 = hullPoints[i + 0];
                pos2 = hullPoints[i + 1];
                pos3 = hullPoints[i + 2];

                JVector.Transform(pos1, orientation, out pos1);
                JVector.Add(pos1, position, out pos1);

                JVector.Transform(pos2, orientation, out pos2);
                JVector.Add(pos2, position, out pos2);

                JVector.Transform(pos3, orientation, out pos3);
                JVector.Add(pos3, position, out pos3);

                drawer.DrawTriangle(pos1, pos2, pos3);
            }
        }
    }
}
