using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.Collision;

namespace JitterDemo.Scenes
{
    public class TriangleMesh : Scene
    {
        private Model model;

        public TriangleMesh(JitterDemo demo)
            : base(demo)
        {
        }

        /// <summary>
        /// Helper Method to get the vertex and index List from the model.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="indices"></param>
        /// <param name="model"></param>
        public void ExtractData(List<Vector3> vertices, List<TriangleVertexIndices> indices, Model model)
        {
            var bones_ = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(bones_);
            foreach (var mm in model.Meshes)
            {
                var xform = bones_[mm.ParentBone.Index];
                foreach (var mmp in mm.MeshParts)
                {
                    int offset = vertices.Count;
                    var a = new Vector3[mmp.NumVertices];
                    mmp.VertexBuffer.GetData(mmp.VertexOffset * mmp.VertexBuffer.VertexDeclaration.VertexStride,
                        a, 0, mmp.NumVertices, mmp.VertexBuffer.VertexDeclaration.VertexStride);
                    for (int i = 0; i != a.Length; ++i)
                        Vector3.Transform(ref a[i], ref xform, out a[i]);
                    vertices.AddRange(a);

                    if (mmp.IndexBuffer.IndexElementSize != IndexElementSize.SixteenBits)
                        throw new Exception(
                            String.Format("Model uses 32-bit indices, which are not supported."));
                    short[] s = new short[mmp.PrimitiveCount * 3];
                    mmp.IndexBuffer.GetData(mmp.StartIndex * 2, s, 0, mmp.PrimitiveCount * 3);
                    var tvi = new TriangleVertexIndices[mmp.PrimitiveCount];
                    for (int i = 0; i != tvi.Length; ++i)
                    {
                        tvi[i].I0 = s[(i * 3) + 0] + offset;
                        tvi[i].I1 = s[(i * 3) + 1] + offset;
                        tvi[i].I2 = s[(i * 3) + 2] + offset;
                    }
                    indices.AddRange(tvi);
                }
            }
        }

        public override void Draw()
        {
            var camera = Demo.Camera;

            foreach (var mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.World = boneTransforms[mesh.ParentBone.Index];
                    effect.View = camera.View;
                    effect.Projection = camera.Projection;

                    effect.EnableDefaultLighting();
                    effect.PreferPerPixelLighting = true;
                }
                mesh.Draw();
            }
        }

        private Matrix[] boneTransforms;

        public override void Build()
        {
            model = Demo.Content.Load<Model>("staticmesh");
            boneTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(boneTransforms);

            var indices = new List<TriangleVertexIndices>();
            var vertices = new List<Vector3>();

            ExtractData(vertices, indices, model);

            var jvertices = new List<JVector>(vertices.Count);
            foreach(var vertex in vertices) jvertices.Add(Conversion.ToJitterVector(vertex));

            var octree = new Octree(jvertices, indices);

            var tms = new TriangleMeshShape(octree);
            var body = new RigidBody(tms)
            {
                IsStatic = true,
                //body.EnableDebugDraw = true;
                Tag = BodyTag.DontDrawMe
            };

            Demo.World.AddBody(body);

            AddCar(new JVector(-20, 20, 0));
        }
    }
}
