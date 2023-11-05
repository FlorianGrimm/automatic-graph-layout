using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.ProjectionSolver;
using Microsoft.Msagl.Routing;

namespace Microsoft.Msagl.Layout.Layered {
    internal class TwoLayerFlatEdgeRouter : AlgorithmBase {
        private readonly int[] bottomLayer;
        private InteractiveEdgeRouter interactiveEdgeRouter;
        private double[] labelCenters;
        private Dictionary<Label, ICurve> labelsToLabelObstacles = new Dictionary<Label, ICurve>();
        private IntPair[] pairArray;
        private readonly Routing routing;
        private readonly int[] topLayer;
        private SugiyamaLayoutSettings settings;

        internal TwoLayerFlatEdgeRouter(SugiyamaLayoutSettings settings, Routing routing, int[] bottomLayer, int[] topLayer)
        {
            this.settings = settings;
            this.topLayer = topLayer;
            this.bottomLayer = bottomLayer;
            this.routing = routing;
            this.InitLabelsInfo();
        }

        private Database Database {
            get { return this.routing.Database; }
        }

        private int[] Layering {
            get { return this.routing.LayerArrays.Y; }
        }

        private BasicGraphOnEdges<PolyIntEdge> IntGraph {
            get { return this.routing.IntGraph; }
        }

        private LayerArrays LayerArrays {
            get { return this.routing.LayerArrays; }
        }

        private double PaddingForEdges {
            get { return this.settings.LayerSeparation / 8; }
        }

        private void InitLabelsInfo() {
            this.pairArray = new Set<IntPair>(from v in this.bottomLayer
                                         where v < this.IntGraph.NodeCount
                                         from edge in this.IntGraph.OutEdges(v)
                                         where edge.Source != edge.Target
                                         where this.Layering[edge.Target] == this.Layering[edge.Source]
                                         select new IntPair(edge.Source, edge.Target)).ToArray();
            this.labelCenters = new double[this.pairArray.Length];
            int i = 0;
            foreach (IntPair p in this.pairArray) {
                int leftNode, rightNode;
                if (this.LayerArrays.X[p.First] < this.LayerArrays.X[p.Second]) {
                    leftNode = p.First;
                    rightNode = p.Second;
                } else {
                    leftNode = p.Second;
                    rightNode = p.First;
                }
                this.labelCenters[i++] = (this.Database.Anchors[leftNode].Right + this.Database.Anchors[rightNode].Left)/2;
                //labelCenters contains ideal position for nodes at the moment
            }
            this.InitLabelsToLabelObstacles();
        }

        private void InitLabelsToLabelObstacles() {
            this.labelsToLabelObstacles = new Dictionary<Label, ICurve>();
            IEnumerable<Label> labels = from p in this.pairArray from label in this.PairLabels(p) select label;
            foreach (Label label in labels) {
                this.labelsToLabelObstacles[label] = CreatObstaceOnLabel(label);
            }
        }

        private double GetMaxLabelWidth(IntPair intPair) {
            IEnumerable<Label> multiEdgeLabels = this.PairLabels(intPair);

            if (multiEdgeLabels.Any()) {
                return multiEdgeLabels.Max(label => label.Width);
            }

            return 0;
        }

        private IEnumerable<Label> PairLabels(IntPair intPair) {
            return from edge in this.Database.GetMultiedge(intPair)
                   let label = edge.Edge.Label
                   where label != null
                   select label;
        }

        /// <summary>
        /// Executes the algorithm.
        /// </summary>
        protected override void RunInternal() {
            if (this.pairArray.Length > 0) {
                this.PositionLabelsOfFlatEdges();
                this.interactiveEdgeRouter = new InteractiveEdgeRouter(this.GetObstacles(), this.PaddingForEdges, this.PaddingForEdges /3, Math.PI/6);
                this.interactiveEdgeRouter.CalculateWholeTangentVisibilityGraph();
                foreach (PolyIntEdge intEdge in this.IntEdges())
                {
                    this.ProgressStep();
                    this.RouteEdge(intEdge);
                }
            }
        }

        private IEnumerable<ICurve> GetObstacles() {
            return (from v in this.topLayer.Concat(this.bottomLayer)
                    where v < this.routing.OriginalGraph.Nodes.Count
                    select this.routing.IntGraph.Nodes[v].BoundaryCurve).Concat(this.LabelCurves());
        }

        private IEnumerable<ICurve> LabelCurves() {
            return from edge in this.IntEdges()
                   let label = edge.Edge.Label
                   where label != null
                   select CreatObstaceOnLabel(label);
        }

        private static ICurve CreatObstaceOnLabel(Label label) {
            var c = new Curve();
            double obstacleBottom = label.Center.Y - label.Height/4;
            c.AddSegment(new LineSegment(new Point(label.BoundingBox.Left, obstacleBottom),
                                         new Point(label.BoundingBox.Right, obstacleBottom)));
            Curve.ContinueWithLineSegment(c, label.BoundingBox.RightTop);
            Curve.ContinueWithLineSegment(c, label.BoundingBox.LeftTop);
            Curve.CloseCurve(c);
            return c;
        }

        private IEnumerable<PolyIntEdge> IntEdges() {
            return from pair in this.pairArray from edge in this.Database.GetMultiedge(pair) select edge;
        }

        private void RouteEdge(PolyIntEdge edge) {
            if (edge.HasLabel) {
                this.RouteEdgeWithLabel(edge, edge.Edge.Label);
            } else {
                this.RouteEdgeWithNoLabel(edge);
            }
        }

        private void RouteEdgeWithLabel(PolyIntEdge intEdge, Label label) {
            //we allow here for the edge to cross its own label
            Node sourceNode = this.routing.IntGraph.Nodes[intEdge.Source];
            Node targetNode = this.routing.IntGraph.Nodes[intEdge.Target];
            var sourcePort = new FloatingPort(sourceNode.BoundaryCurve, sourceNode.Center);
            var targetPort = new FloatingPort(targetNode.BoundaryCurve, targetNode.Center);
            ICurve labelObstacle = this.labelsToLabelObstacles[label];
            var labelPort = new FloatingPort(labelObstacle, label.Center);
            SmoothedPolyline poly0;
            this.interactiveEdgeRouter.RouteSplineFromPortToPortWhenTheWholeGraphIsReady(sourcePort, labelPort, true, out poly0);
            SmoothedPolyline poly1;
            this.interactiveEdgeRouter.RouteSplineFromPortToPortWhenTheWholeGraphIsReady(labelPort, targetPort, true, out poly1);
            CornerSite site = poly1.HeadSite.Next;

            CornerSite lastSite = poly0.LastSite;
            lastSite.Next = site;
            site.Previous = lastSite;
            var eg = intEdge.Edge.EdgeGeometry;
            eg.SetSmoothedPolylineAndCurve(poly0);
            Arrowheads.TrimSplineAndCalculateArrowheads(eg, intEdge.Edge.Source.BoundaryCurve,
                                                             intEdge.Edge.Target.BoundaryCurve, eg.Curve, false);
        }

        private void RouteEdgeWithNoLabel(PolyIntEdge intEdge) {
            Node sourceNode = this.routing.IntGraph.Nodes[intEdge.Source];
            Node targetNode = this.routing.IntGraph.Nodes[intEdge.Target];
            var sourcePort = new FloatingPort(sourceNode.BoundaryCurve, sourceNode.Center);
            var targetPort = new FloatingPort(targetNode.BoundaryCurve, targetNode.Center);
            var eg = intEdge.Edge.EdgeGeometry;
            SmoothedPolyline sp;
            eg.Curve = this.interactiveEdgeRouter.RouteSplineFromPortToPortWhenTheWholeGraphIsReady(sourcePort, targetPort, true, out sp);
            Arrowheads.TrimSplineAndCalculateArrowheads(eg, intEdge.Edge.Source.BoundaryCurve,
                                                             intEdge.Edge.Target.BoundaryCurve, eg.Curve, false);
            intEdge.Edge.EdgeGeometry = eg;
        }

        private void PositionLabelsOfFlatEdges() {
            if (this.labelCenters == null || this.labelCenters.Length == 0) {
                return;
            }

            this.SortLabelsByX();
            this.CalculateLabelsX();
        }

        private void CalculateLabelsX() {
            int i;
            var solver = new SolverShell();
            for (i = 0; i < this.pairArray.Length; i++) {
                solver.AddVariableWithIdealPosition(i, this.labelCenters[i], this.GetLabelWeight(this.pairArray[i]));
            }

            //add non overlapping constraints between to neighbor labels
            double prevLabelWidth = this.GetMaxLabelWidth(this.pairArray[0]);
            for (i = 0; i < this.pairArray.Length - 1; i++) {
                solver.AddLeftRightSeparationConstraint(i, i + 1,
                                                        (prevLabelWidth +
                                                         (prevLabelWidth = this.GetMaxLabelWidth(this.pairArray[i + 1])))/2 +
                                                        this.settings.NodeSeparation);
            }

            for (i = 0; i < this.labelCenters.Length; i++) {
                double x = this.labelCenters[i] = solver.GetVariableResolvedPosition(i);
                foreach (Label label in this.PairLabels(this.pairArray[i])) {
                    label.Center = new Point(x, label.Center.Y);
                }
            }
        }

        private double GetLabelWeight(IntPair intPair) {
            return this.Database.GetMultiedge(intPair).Count;
        }

        private void SortLabelsByX() {
            Array.Sort(this.labelCenters, this.pairArray);
        }
    }
}