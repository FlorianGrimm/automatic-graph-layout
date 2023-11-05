using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;

namespace Microsoft.Msagl.Core.GraphAlgorithms {
    internal class CycleRemovalWithConstraints<TEdge> where TEdge : IEdge {
        private BasicGraphOnEdges<TEdge> graph;
        private BasicGraphOnEdges<IntPair> graphOfConstraints;
        private IEnumerable<IntPair> constrainedEdges;
        internal CycleRemovalWithConstraints(BasicGraphOnEdges<TEdge> graph, Set<IntPair> constraints) {
            this.graph = graph;
            this.constrainedEdges = constraints;
            this.graphOfConstraints =new BasicGraphOnEdges<IntPair>(this.constrainedEdges, graph.NodeCount);
        }

        internal IEnumerable<IEdge> GetFeedbackSet() {
            foreach (GraphForCycleRemoval graphForCycleRemoval in this.CreateGraphsForCycleRemoval()) {
                foreach (IEdge edge in this.GetFeedbackEdgeSet(graphForCycleRemoval)) {
                    yield return edge;
                }
            }
        }

        /// <summary>
        /// following H.A.D Nascimento and P. Eades "User Hints for Directed Graph Drawing"
        /// </summary>
        /// <param name="graphForCycleRemoval">graphForCycleRemoval is connected</param>
        /// <returns></returns>
        private IEnumerable<IEdge> GetFeedbackEdgeSet(GraphForCycleRemoval graphForCycleRemoval) {
            graphForCycleRemoval.Initialize();
            //empty at the end of the method
            List<int> sl = new List<int>(); //sl - the sequence left part
            List<int> sr = new List<int>(); //sr - the sequence right part. In our case it is a reversed right part
            while (!graphForCycleRemoval.IsEmpty()) {
                int u;
                while (graphForCycleRemoval.TryGetSink(out u)) {
                    graphForCycleRemoval.RemoveNode(u);
                    sr.Add(u);
                }
                while (graphForCycleRemoval.TryGetSource(out u)) {
                    graphForCycleRemoval.RemoveNode(u);
                    sl.Add(u);
                }
                if (graphForCycleRemoval.TryFindVertexWithNoIncomingConstrainedEdgeAndMaximumOutDegreeMinusInDedree(out u)) {
                    graphForCycleRemoval.RemoveNode(u);
                    sl.Add(u);
                }
            }

            Dictionary<int, int> S = new Dictionary<int, int>(sl.Count + sr.Count);
            int j=0;
            foreach (int u in sl) {
                S[u] = j++;
            }

            for (int i = sr.Count - 1; i >= 0; i--) {
                S[sr[i]] = j++;
            }

            foreach (IntPair pair in graphForCycleRemoval.GetOriginalIntPairs()) {
                if (S[pair.First] > S[pair.Second]) {
                    yield return pair;
                }
            }
        }

        private IEnumerable<GraphForCycleRemoval> CreateGraphsForCycleRemoval() {
            foreach (IEnumerable<int> componentNodes in ConnectedComponentCalculator<IntPair>.GetComponents(this.GetCommonGraph())) {
                yield return this.CreateGraphForCycleRemoval(componentNodes);
            }
        }

        private BasicGraphOnEdges<IntPair> GetCommonGraph() {
            return new BasicGraphOnEdges<IntPair>((from edge in this.graph.Edges select new IntPair(edge.Source, edge.Target)).Concat(this.constrainedEdges), this.graph.NodeCount);
        }

        private GraphForCycleRemoval CreateGraphForCycleRemoval(IEnumerable<int> componentNodes) {
            GraphForCycleRemoval graphForCycleRemoval = new GraphForCycleRemoval();
            foreach (int i in componentNodes) {
                foreach (TEdge edge in this.graph.OutEdges(i)) {
                    graphForCycleRemoval.AddEdge(new IntPair(edge.Source,edge.Target));
                }

                foreach (IntPair intPair in this.graphOfConstraints.OutEdges(i)) {
                    graphForCycleRemoval.AddConstraintEdge(intPair);
                }
            }
            return graphForCycleRemoval;
        }
    }
}
