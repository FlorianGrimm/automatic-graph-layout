using Microsoft.Msagl.Core.GraphAlgorithms;

namespace Microsoft.Msagl.Layout.Layered {
    internal class RecoveryLayerCalculator : LayerCalculator {
        private LayerArrays layers;

        public RecoveryLayerCalculator(LayerArrays recoveredLayerArrays) {
            this.layers =recoveredLayerArrays;
        }
        public int[] GetLayers() {
            return this.layers.Y;
        }
    }
}