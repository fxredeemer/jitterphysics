namespace JitterDemo.Scenes
{

    public class EmptyScene : Scene
    {
       
        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();


        }
    }


}
