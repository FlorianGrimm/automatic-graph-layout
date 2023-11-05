using System;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree;
using Microsoft.Msagl.Layout.MDS;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Rectilinear;

namespace Microsoft.Msagl.Prototype.Ranking {
    /// <summary>
    /// Ranking layout for directed graphs.
    /// </summary>
    public class RankingLayout : AlgorithmBase {
        private GeometryGraph graph;
        private RankingLayoutSettings settings;

        /// <summary>
        /// Constructs the ranking layout algorithm.
        /// </summary>
        public RankingLayout(RankingLayoutSettings settings, GeometryGraph geometryGraph)
        {
            this.settings = settings;
            this.graph = geometryGraph;
        }

        private void SetNodePositionsAndMovedBoundaries(GeometryGraph graph) {
            
            int pivotNumber = Math.Min(graph.Nodes.Count, this.settings.PivotNumber);
            double scaleX = this.settings.ScaleX;
            double scaleY = this.settings.ScaleY;
          
            int[] pivotArray = new int[pivotNumber];
            PivotDistances pivotDistances = new PivotDistances(graph, false, pivotArray);
            pivotDistances.Run();
            double[][] c = pivotDistances.Result;
            double[] x, y;
            MultidimensionalScaling.LandmarkClassicalScaling(c, out x, out y, pivotArray);

            Standardize(x);
            double[] p = Centrality.PageRank(graph, .85, false);
            // double[] q = Centrality.PageRank(graph, .85, true);
            Standardize(p);
            // Standardize(q);

            int index = 0;
            foreach (Node node in graph.Nodes) {
                node.Center = new Point((int) (x[index]*scaleX), (int) (Math.Sqrt(p[index])*scaleY));
                index++;
            }

            GTreeOverlapRemoval.RemoveOverlaps(graph.Nodes.ToArray(), this.settings.NodeSeparation);
        }

        /// <summary>
        /// Executes the algorithm.
        /// </summary>
        protected override void RunInternal()
        {
            GeometryGraph[] graphs = GraphConnectedComponents.CreateComponents(this.graph.Nodes, this.graph.Edges, this.settings.NodeSeparation).ToArray();
            // layout components, compute bounding boxes

            for (int i = 0; i < graphs.Length; i++)
            {
                this.Calculate(graphs[i]);
            }

            if (graphs.Length > 1)
            {
                Microsoft.Msagl.Layout.MDS.MdsGraphLayout.PackGraphs(graphs, this.settings);
                //restore the parents
                foreach (var node in graphs.SelectMany(g => g.Nodes)) {
                    node.GeometryParent = this.graph;
                }
            }
        }

        private void Calculate(GeometryGraph graph)
        {
            if (graph.Nodes.Count == 0) {
                return;
            }

            this.SetNodePositionsAndMovedBoundaries(graph);
            double nodeSeparation = this.settings.NodeSeparation;
            if (nodeSeparation <= 0 )
            {
                nodeSeparation = 10;
            }
            GTreeOverlapRemoval.RemoveOverlaps(graph.Nodes.ToArray(), nodeSeparation);
            this.SetGraphBoundingBox(graph);
        }

        private void SetGraphBoundingBox(GeometryGraph graph) {
            graph.BoundingBox = graph.PumpTheBoxToTheGraphWithMargins();
        }

        /// <summary>
        /// Scales and translates a vector so that all values are exactly between 0 and 1.
        /// </summary>
        /// <param name="x">Vector to be standardized.</param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "x")]
        private static void Standardize(double[] x)
        {
            double min = Double.PositiveInfinity;
            double max = Double.NegativeInfinity;
            for (int i = 0; i < x.Length; i++)
            {
                max = Math.Max(max, x[i]);
                min = Math.Min(min, x[i]);
            }
            for (int i = 0; i < x.Length; i++)
            {
                x[i] = (x[i] - min) / (max - min);
            }
        }
    }
}
