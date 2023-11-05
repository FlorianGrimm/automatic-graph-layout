using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.Incremental;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Layout.MDS;

//using Microsoft.Msagl.Routing;

namespace Microsoft.Msagl.Layout.Initial
{
    /// <summary>
    /// Methods for obtaining an initial layout of a graph using various means.
    /// </summary>
    public class InitialLayout : AlgorithmBase
    {
        private GeometryGraph graph;
        private FastIncrementalLayoutSettings settings;

        private int componentCount;

        /// <summary>
        /// Set to true if the graph specified is a single connected component with no clusters
        /// </summary>
        public bool SingleComponent { get; set; }

        /// <summary>
        /// Static layout of graph by gradually adding constraints.
        /// Uses PivotMds to find initial layout.
        /// Breaks the graph into connected components (nodes in the same cluster are considered
        /// connected whether or not there is an edge between them), then lays out each component
        /// individually.  Finally, a simple packing is applied.
        /// ratio as close as possible to the PackingAspectRatio property (not currently used).
        /// </summary>
        public InitialLayout(GeometryGraph graph, FastIncrementalLayoutSettings settings)
        {
            ValidateArg.IsNotNull(graph, "graph");
            ValidateArg.IsNotNull(settings, "settings");
            this.graph = graph;

            this.settings = new FastIncrementalLayoutSettings(settings);
            this.settings.ApplyForces = true;
            this.settings.InterComponentForces = true;
            this.settings.RungeKuttaIntegration = false;
            this.settings.RespectEdgePorts = false;
        }

        /// <summary>
        /// The actual layout process
        /// </summary>
        protected override void RunInternal()
        {
            if (this.SingleComponent)
            {
                this.componentCount = 1;
                this.LayoutComponent(this.graph);
            }
            else
            {
                foreach (var c in this.graph.RootCluster.AllClustersDepthFirst())
                {
                    if (c == this.graph.RootCluster || c.RectangularBoundary == null) {
                        continue;
                    }

                    c.RectangularBoundary.GenerateFixedConstraints = false;
                }

                var components = this.graph.GetClusteredConnectedComponents().ToList();

                this.componentCount = components.Count;

                foreach (var component in components)
                {
                    this.LayoutComponent(component);
                }

                this.graph.BoundingBox = MdsGraphLayout.PackGraphs(components, this.settings);
                this.ProgressComplete();

                // update positions of original graph elements
                foreach (var v in this.graph.Nodes)
                {
                    var copy = v.AlgorithmData as GraphConnectedComponents.AlgorithmDataNodeWrap;
                    Debug.Assert(copy != null);
                    v.Center = copy.node.Center;
                }

                foreach (var e in this.graph.Edges)
                {
                    var copy = e.AlgorithmData as Edge;
                    if (copy != null)
                    {
                        e.EdgeGeometry = copy.EdgeGeometry;
                        e.EdgeGeometry.Curve = copy.Curve;
                    }
                }

                foreach (var c in this.graph.RootCluster.AllClustersDepthFirst().Where(c => c != this.graph.RootCluster))
                {
                    var copy = c.AlgorithmData as GraphConnectedComponents.AlgorithmDataNodeWrap;
                    var copyCluster = copy.node as Cluster;
                    Debug.Assert(copyCluster != null);
                    c.RectangularBoundary = copyCluster.RectangularBoundary;
                    c.RectangularBoundary.GenerateFixedConstraints = c.RectangularBoundary.GenerateFixedConstraintsDefault;
                    c.BoundingBox = c.RectangularBoundary.Rect;
                    c.RaiseLayoutDoneEvent();
                }
            }
        }

        private void LayoutComponent(GeometryGraph component)
        {
            if (component.Nodes.Count > 1 || component.RootCluster.Clusters.Any())
            {
                // for small graphs (below 100 nodes) do extra iterations
                this.settings.MaxIterations = LayoutAlgorithmHelpers.NegativeLinearInterpolation(
                    component.Nodes.Count,
                    /*lowerThreshold:*/ 50, /*upperThreshold:*/ 500, /*minIterations:*/ 5, /*maxIterations:*/ 10);
                this.settings.MinorIterations = LayoutAlgorithmHelpers.NegativeLinearInterpolation(component.Nodes.Count,
                    /*lowerThreshold:*/ 50, /*upperThreshold:*/ 500, /*minIterations:*/ 3, /*maxIterations:*/ 20);

                if (this.settings.MinConstraintLevel == 0)
                {
                    // run PivotMDS with a largish Scale so that the layout comes back oversized.
                    // subsequent incremental iterations do a better job of untangling when they're pulling it in
                    // rather than pushing it apart.
                    PivotMDS pivotMDS = new PivotMDS(component); ;
                    this.RunChildAlgorithm(pivotMDS, 0.5 / this.componentCount);
                }
                FastIncrementalLayout fil = new FastIncrementalLayout(component, this.settings, this.settings.MinConstraintLevel, anyCluster => this.settings);
                Debug.Assert(this.settings.Iterations == 0);

                foreach (var level in this.GetConstraintLevels(component))
                {
                    if (level > this.settings.MaxConstraintLevel)
                    {
                        break;
                    }
                    if (level > this.settings.MinConstraintLevel)
                    {
                        fil.SetCurrentConstraintLevel(level);
                    }
                    do
                    {
                        fil.Run();
                    } while (!this.settings.IsDone);
                }
            }

            // Pad the graph with margins so the packing will be spaced out.
            component.Margins = this.settings.NodeSeparation;
            component.UpdateBoundingBox();

            // Zero the graph
            component.Translate(-component.BoundingBox.LeftBottom);
        }

        /// <summary>
        /// Get the distinct ConstraintLevels that need to be applied to layout.
        /// Used by InitialLayout.
        /// Will only include ConstraintLevel == 1 if there are structural constraints
        /// Will only include ConstraintLevel == 2 if AvoidOverlaps is on and there are fewer than 2000 nodes
        /// </summary>
        /// <returns>0, 1 or 2</returns>
        private IEnumerable<int> GetConstraintLevels(GeometryGraph component)
        {
            var keys = (from c in this.settings.StructuralConstraints select c.Level).ToList();
            keys.Add(0);
            if (this.settings.IdealEdgeLength.Direction != Direction.None) { 
                keys.Add(1); 
            }
            if (this.settings.AvoidOverlaps && component.Nodes.Count < 2000) { keys.Add(2); }
            return keys.Distinct();
        }

    }
}
