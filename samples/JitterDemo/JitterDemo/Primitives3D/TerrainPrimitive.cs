using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

namespace JitterDemo.Primitives3D
{
    public class TerrainPrimitive : GeometricPrimitive
    {
        public delegate float TerrainFunction(int coordX,int coordZ);

        public float[,] heights;

        public TerrainPrimitive(GraphicsDevice device,TerrainFunction function)
        {
            heights = new float[100,100];

            for (int i = 0; i < 100; i++)
            {
                for (int e = 0; e < 100; e++)
                {
                    heights[i,e]=function(i,e);
                }
            }

            var neighbour = new Vector3[4];

            for (int i = 0; i < 100; i++)
            {
                for (int e = 0; e < 100; e++)
                {
                    var pos = new Vector3(i, heights[i,e], e);

                    if (i > 0) neighbour[0] = new Vector3(i - 1, heights[i - 1,e], e);
                    else neighbour[0] = pos;

                    if (e > 0) neighbour[1] = new Vector3(i, heights[i,e - 1], e - 1);
                    else neighbour[1] = pos;

                    if (i < 99) neighbour[2] = new Vector3(i +1, heights[i + 1,e], e);
                    else neighbour[2] = pos;

                    if (e < 99) neighbour[3] = new Vector3(i, heights[i,e+1], e+1);
                    else neighbour[3] = pos;

                    var normal = Vector3.Zero;

                    normal += Vector3.Cross(neighbour[1] - pos, neighbour[0] - pos);
                    normal += Vector3.Cross(neighbour[2] - pos, neighbour[1] - pos);
                    normal += Vector3.Cross(neighbour[3] - pos, neighbour[2] - pos);
                    normal += Vector3.Cross(neighbour[0] - pos, neighbour[3] - pos);
                    normal.Normalize();

                    AddVertex(new Vector3(i, heights[i,e], e), normal);
                }
            }

            for (int i = 1; i < 100; i++)
            {
                for (int e = 1; e < 100; e++)
                {
                    AddIndex(((i - 1) * 100) + e);
                    AddIndex((i * 100) + (e - 1));
                    AddIndex((i * 100) + e);

                    AddIndex((i * 100) + (e - 1));
                    AddIndex(((i - 1) * 100) + e);
                    AddIndex(((i - 1) * 100) + (e - 1));
                }
            }
            

            InitializePrimitive(device);
        }
    }
}
