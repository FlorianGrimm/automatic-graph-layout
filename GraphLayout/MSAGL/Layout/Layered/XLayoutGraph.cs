using System.Collections.Generic;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered
{

  

    /// <summary>
    /// Follows the idea from Gansner etc 93, creating a special graph
    /// for x-coordinates calculation
    /// </summary>
    internal class XLayoutGraph : BasicGraphOnEdges<PolyIntEdge>
    {
        private ProperLayeredGraph layeredGraph;//the result of layering

        private LayerArrays layerArrays;//the result of layering


        private int virtualVerticesStart;
        private int virtualVerticesEnd; // we have 0,,,virtualVerticesStart-1 - usual vertices
                                       //virtualVerticesStart,...,virtualVerticesEnd -virtual vertices
                                       //and virtualVirticesEnd+1, ...NumberOfVertices - nvertices

        private int weightMultiplierOfOriginalOriginal = 1; //weight multiplier for edges with Defaults or n end and start
        private int weightMultOfOneVirtual = 3; //weight multiplier for edges with only one virtual node
        private int weightMultiplierOfTwoVirtual = 8; //weight multiplier for edges with two virtual nodes

        internal XLayoutGraph(BasicGraphOnEdges<PolyIntEdge> graph, //DAG of the original graph with no multiple edges
                              ProperLayeredGraph layeredGraph,
                              LayerArrays layerArrays,
                              List<PolyIntEdge> edges,
                              int nov)
        {
            this.SetEdges(edges, nov);
            this.virtualVerticesStart = graph.NodeCount;
            this.virtualVerticesEnd = layeredGraph.NodeCount - 1;
            this.layeredGraph = layeredGraph;
            this.layerArrays = layerArrays;
        }

      
        /// <summary>
        /// following Gansner etc 93 returning weight multplier bigger if there are virtual nodes
        /// </summary>
        /// <param name="edge"></param>
        /// <returns></returns>
        internal int EdgeWeightMultiplier(PolyIntEdge edge)
        {


            int s = edge.Source;
            int t = edge.Target;

            if (s < this.layeredGraph.NodeCount &&
                this.layerArrays.Y[s] == this.layerArrays.Y[t] &&
                this.layerArrays.X[s] == this.layerArrays.X[t] + 1) {
                return 0; //this edge needed only for separation vertices in the same layer
            }

            int k = 0;
            System.Diagnostics.Debug.Assert(s >= this.layeredGraph.NodeCount); //check the graph on correctness`    
            //    throw new InvalidOperationException();//"XLayout graph is incorrect");

            //here (s0,t0) is the edge of underlying graph 
            int s0 = -1, t0 = -1; //t0 is set to -1 to only avoid the warning
            //there are only two edges in graph.OutEdges(s)
            foreach (PolyIntEdge intEdge in this.OutEdges(s))
            {
                if (s0 == -1) {
                    s0 = intEdge.Target;
                } else {
                    t0 = intEdge.Target;
                }
            }

            if (s0 >= this.virtualVerticesStart && s0 <= this.virtualVerticesEnd) {
                k++;
            }

            if (t0 >= this.virtualVerticesStart && t0 <= this.virtualVerticesEnd) {
                k++;
            }

            int ret = k == 0 ? this.weightMultiplierOfOriginalOriginal : (k == 1 ? this.weightMultOfOneVirtual : this.weightMultiplierOfTwoVirtual);
            return ret;
        }

        /// <summary>
        /// caching edges weights
        /// </summary>
        internal void SetEdgeWeights()
        {

            foreach (PolyIntEdge intEdge in this.Edges) {
                intEdge.Weight = intEdge.Weight * this.EdgeWeightMultiplier(intEdge);
            }
        }
       
    }
}
