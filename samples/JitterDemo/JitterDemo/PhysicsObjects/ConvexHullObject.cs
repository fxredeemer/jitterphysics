﻿using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Jitter.LinearMath;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using Microsoft.Xna.Framework.Graphics;
using Jitter.Collision;

namespace JitterDemo
{
    public class ConvexHullObject : DrawableGameComponent
    {
        private static readonly Random random = new Random();

        public RigidBody body;

        private Model model;

        public ConvexHullObject(Game game)
            : base(game)
        {
        }

        public static void ExtractData(List<JVector> vertices, List<TriangleVertexIndices> indices, Model model)
        {
            var bones_ = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(bones_);
            foreach (var modelmesh in model.Meshes)
            {
                var xform = Conversion.ToJitterMatrix(bones_[modelmesh.ParentBone.Index]);
                foreach (var meshPart in modelmesh.MeshParts)
                {
                    // Before we add any more where are we starting from 
                    int offset = vertices.Count;

                    // Read the format of the vertex buffer 
                    var declaration = meshPart.VertexBuffer.VertexDeclaration;
                    var vertexElements = declaration.GetVertexElements();
                    // Find the element that holds the position 
                    var vertexPosition = new VertexElement();
                    foreach (var vert in vertexElements)
                    {
                        if (vert.VertexElementUsage == VertexElementUsage.Position
                        && vert.VertexElementFormat == VertexElementFormat.Vector3)
                        {
                            vertexPosition = vert;
                            // There should only be one 
                            break;
                        }
                    }
                    // Check the position element found is valid 
                    if (vertexPosition == null
                    || vertexPosition.VertexElementUsage != VertexElementUsage.Position
                    || vertexPosition.VertexElementFormat != VertexElementFormat.Vector3)
                    {
                        throw new Exception("Model uses unsupported vertex format!");
                    }
                    // This where we store the vertices until transformed 
                    var allVertex = new JVector[meshPart.NumVertices];
                    // Read the vertices from the buffer in to the array 
                    meshPart.VertexBuffer.GetData(
                        (meshPart.VertexOffset * declaration.VertexStride) + vertexPosition.Offset,
                        allVertex,
                        0,
                        meshPart.NumVertices,
                        declaration.VertexStride);
                    // Transform them based on the relative bone location and the world if provided 
                    for (int i = 0; i != allVertex.Length; ++i)
                    {
                        JVector.Transform(ref allVertex[i], ref xform, out allVertex[i]);
                    }
                    // Store the transformed vertices with those from all the other meshes in this model 
                    vertices.AddRange(allVertex);

                    // Find out which vertices make up which triangles 
                    if (meshPart.IndexBuffer.IndexElementSize != IndexElementSize.SixteenBits)
                    {
                        // This could probably be handled by using int in place of short but is unnecessary 
                        throw new Exception("Model uses 32-bit indices, which are not supported.");
                    }
                    // Each primitive is a triangle 
                    short[] indexElements = new short[meshPart.PrimitiveCount * 3];
                    meshPart.IndexBuffer.GetData(
                    meshPart.StartIndex * 2,
                    indexElements,
                    0,
                    meshPart.PrimitiveCount * 3);
                    // Each TriangleVertexIndices holds the three indexes to each vertex that makes up a triangle 
                    var tvi = new TriangleVertexIndices[meshPart.PrimitiveCount];
                    for (int i = 0; i != tvi.Length; ++i)
                    {
                        // The offset is because we are storing them all in the one array and the 
                        // vertices were added to the end of the array. 
                        tvi[i].I0 = indexElements[(i * 3) + 0] + offset;
                        tvi[i].I1 = indexElements[(i * 3) + 1] + offset;
                        tvi[i].I2 = indexElements[(i * 3) + 2] + offset;
                    }
                    // Store our triangles 
                    indices.AddRange(tvi);
                }
            }
        }

        private static ConvexHullShape cvhs = null;

        protected override void LoadContent()
        {
            model = Game.Content.Load<Model>("convexhull");

            if (cvhs == null)
            {
                var jvecs = new List<JVector>();
                var indices = new List<TriangleVertexIndices>();

                ExtractData(jvecs, indices, model);

                int[] convexHullIndices = JConvexHull.Build(jvecs, JConvexHull.Approximation.Level6);

                var hullPoints = new List<JVector>();
                for (int i = 0; i < convexHullIndices.Length; i++)
                {
                    hullPoints.Add(jvecs[convexHullIndices[i]]);
                }

                cvhs = new ConvexHullShape(hullPoints);
            }

            body = new RigidBody(cvhs)
            {
                Tag = BodyTag.DontDrawMe
            };
        }

        public override void Draw(GameTime gameTime)
        {
            var hullShape = body.Shape as ConvexHullShape;

            var world = Conversion.ToXNAMatrix(body.Orientation);

            // RigidBody.Position gives you the position of the center of mass of the shape.
            // But this is not the center of our graphical represantion, use the
            // "shift" property of the more complex shapes to deal with this.
            world.Translation = Conversion.ToXNAVector(body.Position
                + JVector.Transform(hullShape.Shift, body.Orientation));

            var boneTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(boneTransforms);

            foreach (var mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.View = ((JitterDemo)Game).Camera.View;
                    effect.EnableDefaultLighting();
                    effect.Projection = ((JitterDemo)Game).Camera.Projection;
                    //effect.DiffuseColor = Color.Gray.ToVector3();
                    effect.World = boneTransforms[mesh.ParentBone.Index] * world;
                }
                mesh.Draw();
            }

            base.Draw(gameTime);
        }
    }
}
