#region Using directives



#endregion

using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Layering the DAG by longest path
    /// </summary>
    internal class LongestPathLayering : LayerCalculator {
        private BasicGraphOnEdges<PolyIntEdge> graph;

        public int[] GetLayers() {
            //sort the vertices in topological order
            int[] topoOrder = PolyIntEdge.GetOrder(this.graph);
            int[] layering = new int[this.graph.NodeCount];

            //going backward from leaves
            int k = this.graph.NodeCount;
            while (k-- > 0) {
                int v = topoOrder[k];
                foreach (PolyIntEdge e in this.graph.InEdges(v)) {
                    int u = e.Source;
                    int l = layering[v] + e.Separation;
                    if (layering[u] < l) {
                        layering[u] = l;
                    }
                }
            }
            return layering;

        }

        internal LongestPathLayering(BasicGraphOnEdges<PolyIntEdge> graph){
            this.graph=graph;
        }
        
    }
}
