using Microsoft.Msagl.Core;
namespace Microsoft.Msagl.Layout.Layered {
    internal class FlatEdgeRouter : AlgorithmBase {
        private readonly Routing routing;
        private SugiyamaLayoutSettings settings;

        private int[][] Layers { get { return this.routing.LayerArrays.Layers; } }

        internal FlatEdgeRouter(SugiyamaLayoutSettings settings, Routing routing)
        {
            this.settings = settings;
            this.routing = routing;
        }

        protected override void RunInternal()
        {
            for (int i = 0; i < this.Layers.Length; i++) {
                this.ProgressStep();
                this.RouteFlatEdgesBetweenTwoLayers(this.Layers[i],
                    i < this.Layers.Length - 1 ? this.Layers[i + 1] : new int[0]);
            }
        }

        private void RouteFlatEdgesBetweenTwoLayers(int[] lowerLayer, int[] upperLayer) {
            var twoLayerRouter = new TwoLayerFlatEdgeRouter(this.settings, this.routing,
                lowerLayer, upperLayer);
            twoLayerRouter.Run();
        }
    }
}
