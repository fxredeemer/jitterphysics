namespace JitterDemo
{
    internal static class Program
    {
        private static void Main()
        {
            using (var game = new JitterDemo())
            {

                game.Run();
            }
        }
    }
}

