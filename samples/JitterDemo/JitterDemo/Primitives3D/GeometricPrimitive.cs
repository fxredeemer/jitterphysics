using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JitterDemo.Primitives3D
{
    public abstract class GeometricPrimitive : IDisposable
    {
        private readonly List<VertexPositionNormal> vertices = new List<VertexPositionNormal>();
        private readonly List<ushort> indices = new List<ushort>();

        private VertexBuffer vertexBuffer;
        private IndexBuffer indexBuffer;

        protected void AddVertex(Vector3 position, Vector3 normal)
        {
            vertices.Add(new VertexPositionNormal(position, normal));
        }

        protected void AddIndex(int index)
        {
            if (index > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(index));

            indices.Add((ushort)index);
        }

        protected int CurrentVertex
        {
            get { return vertices.Count; }
        }

        protected void InitializePrimitive(GraphicsDevice graphicsDevice)
        {
            vertexBuffer = new VertexBuffer(graphicsDevice,
                                typeof(VertexPositionNormal),
                                vertices.Count, BufferUsage.None);

            vertexBuffer.SetData(vertices.ToArray());

            indexBuffer = new IndexBuffer(graphicsDevice, typeof(ushort),
                              indices.Count, BufferUsage.None);

            indexBuffer.SetData(indices.ToArray());
        }

        ~GeometricPrimitive()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                vertexBuffer?.Dispose();

                indexBuffer?.Dispose();
            }
        }

        private Matrix[] worlds = new Matrix[1];
        private int index = 0;

        public void AddWorldMatrix(Matrix matrix)
        {
            if (index == worlds.Length)
            {
                var temp = new Matrix[worlds.Length + 50];
                worlds.CopyTo(temp, 0);
                worlds = temp;
            }

            worlds[index] = matrix;
            index++;
        }

        public void Draw(BasicEffect effect)
        {
            if (index == 0) return;

            var graphicsDevice = effect.GraphicsDevice;

            graphicsDevice.SetVertexBuffer(vertexBuffer);
            graphicsDevice.Indices = indexBuffer;

            int primitiveCount = indices.Count / 3;

            for (int i = 0; i < index; i++)
            {
                effect.World = worlds[i]; effect.CurrentTechnique.Passes[0].Apply();
                graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, primitiveCount);
            }

            index = 0;
        }
    }
}
