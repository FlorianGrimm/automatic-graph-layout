using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// This is almost a copy, considering the data, of LayeredLayoutEngine.
    /// The class restores itself from a given GeometryGraph
    /// </summary>
    internal class RecoveryLayeredLayoutEngine {
        private bool Brandes { get; set; }

        private LayerArrays engineLayerArrays;
        private GeometryGraph originalGraph;
        private ProperLayeredGraph properLayeredGraph;

        /// <summary>
        /// the sugiyama layout settings responsible for the algorithm parameters
        /// </summary>
        internal SugiyamaLayoutSettings sugiyamaSettings;

        /// <summary>
        /// the width of dummy nodes
        /// </summary>
        private static double VirtualNodeWidth {
            get { return 1; }
        }

        /// <summary>
        /// the height of dummy nodes
        /// </summary>
        private static double VirtualNodeHeight(SugiyamaLayoutSettings settings) {
            return settings.MinNodeHeight * 1.5f / 8;
        }

        private Database database;

        /// <summary>
        /// Keeps assorted data associated with the graph.
        /// </summary>
        internal Database Database {
            set { this.database = value; }
            get { return this.database; }
        }

        internal BasicGraph<Node, PolyIntEdge> IntGraph; //the input graph

        private BasicGraph<Node, PolyIntEdge> GluedDagSkeletonForLayering { get; set; }

        //the graph obtained after X coord calculation
        private XLayoutGraph xLayoutGraph;
        private Dictionary<Node, int> nodeIdToIndex;
        private Anchor[] anchors;
        private readonly LayerArrays recoveredLayerArrays;
        private double[] layerToRecoveredYCoordinates;
        private readonly Dictionary<int, double> skeletonVirtualVerticesToX = new Dictionary<int, double>();


        internal RecoveryLayeredLayoutEngine(GeometryGraph originalGraph) {
            if (originalGraph != null) {
                this.Init(originalGraph);
                this.recoveredLayerArrays = RecoverOriginalLayersAndSettings(originalGraph, out this.sugiyamaSettings);                
            }
        }

        private void FillLayersToRecoveredYCoordinates() {
            this.layerToRecoveredYCoordinates = new double[this.recoveredLayerArrays.Layers.Length];
            for (int i = 0; i < this.layerToRecoveredYCoordinates.Length; i++) {
                this.layerToRecoveredYCoordinates[i] = this.originalGraph.Nodes[this.recoveredLayerArrays.Layers[i][0]].Center.Y;
            }
        }

        private void Init(GeometryGraph geometryGraph) {
            this.nodeIdToIndex = new Dictionary<Node, int>();

            IList<Node> nodes = geometryGraph.Nodes;

            this.InitNodesToIndex(nodes);

            PolyIntEdge[] intEdges = this.CreateIntEdges(geometryGraph);

            this.IntGraph = new BasicGraph<Node, PolyIntEdge>(intEdges, geometryGraph.Nodes.Count) { Nodes = nodes };
            this.originalGraph = geometryGraph;
            if (this.sugiyamaSettings == null) {
                this.sugiyamaSettings = new SugiyamaLayoutSettings();
            }

            this.CreateDatabaseAndRegisterIntEdgesInMultiEdges();
        }

        private void CreateDatabaseAndRegisterIntEdgesInMultiEdges() {
            this.Database = new Database();
            foreach (PolyIntEdge e in this.IntGraph.Edges) {
                this.database.RegisterOriginalEdgeInMultiedges(e);
            }
        }

        private PolyIntEdge[] CreateIntEdges(GeometryGraph geometryGraph) {
            var edges = geometryGraph.Edges;


            var intEdges = new PolyIntEdge[edges.Count];
            int i = 0;
            foreach (var edge in edges) {
                if (edge.Source == null || edge.Target == null) {
                    throw new InvalidOperationException(); //"creating an edge with null source or target");
                }

                var intEdge = new PolyIntEdge(this.nodeIdToIndex[edge.Source], this.nodeIdToIndex[edge.Target], edge);

                intEdges[i] = intEdge;
                i++;
            }

            return intEdges;
        }

        private void InitNodesToIndex(IList<Node> nodes) {
            int index = 0;
            foreach (Node n in nodes) {
                this.nodeIdToIndex[n] = index;
                index++;
            }
        }

        internal static int EdgeSpan(int[] layers, PolyIntEdge e) {
            return layers[e.Source] - layers[e.Target];
        }

        /// <summary>
        /// Creating a DAG which glues same layer vertices into one original, 
        /// replacing original multiple edges with a single edge.
        /// upDown constraints will be added as edges
        /// </summary>
        private void CreateGluedDagSkeletonForLayering() {
            this.GluedDagSkeletonForLayering = new BasicGraph<Node, PolyIntEdge>(this.GluedDagSkeletonEdges(),
                                                                        this.originalGraph.Nodes.Count);
            this.SetGluedEdgesWeights();
        }

        private void SetGluedEdgesWeights() {
            var gluedPairsToGluedEdge = new Dictionary<IntPair, PolyIntEdge>();
            foreach (PolyIntEdge ie in this.GluedDagSkeletonForLayering.Edges) {
                gluedPairsToGluedEdge[new IntPair(ie.Source, ie.Target)] = ie;
            }

            foreach (var t in this.database.Multiedges) {
                if (t.Key.x != t.Key.y) {
                    IntPair gluedPair = this.VerticalConstraints.GluedIntPair(t.Key);
                    if (gluedPair.x == gluedPair.y) {
                        continue;
                    }

                    PolyIntEdge gluedIntEdge = gluedPairsToGluedEdge[gluedPair];
                    foreach (PolyIntEdge ie in t.Value) {
                        gluedIntEdge.Weight += ie.Weight;
                    }
                }
            }
        }

        private IEnumerable<PolyIntEdge> GluedDagSkeletonEdges() {
            var ret =
                new Set<PolyIntEdge>(from kv in this.database.Multiedges
                                 where kv.Key.x != kv.Key.y
                                 let e = this.VerticalConstraints.GluedIntEdge(kv.Value[0])
                                 where e.Source != e.Target
                                 select e);

            IEnumerable<PolyIntEdge> gluedUpDownConstraints = from p in this.VerticalConstraints.GluedUpDownIntConstraints
                                                          select CreateUpDownConstrainedIntEdge(p);
            foreach (PolyIntEdge edge in gluedUpDownConstraints) {
                ret.Insert(edge);
            }

            return ret;
        }

        private static PolyIntEdge CreateUpDownConstrainedIntEdge(IntPair intPair) {
            var intEdge = new PolyIntEdge(intPair.x, intPair.y);
            intEdge.Weight = 0;
            //we do not want the edge weight to contribute in to the sum but just take the constraint into account
            intEdge.Separation = 1;
            return intEdge;
        }


        /// <summary>
        /// The main calculation is done here
        /// </summary>       
        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider",
            MessageId = "System.String.Format(System.String,System.Object,System.Object,System.Object)"),
         SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider",
             MessageId = "System.String.Format(System.String,System.Object)")]
        internal void Run() {
            if (this.originalGraph.Nodes.Count > 0) {
                this.engineLayerArrays = this.CalculateLayers();
            } else {
                this.originalGraph.boundingBox.SetToEmpty();
            }
        }

        private LayerArrays CalculateLayers() {
            this.CreateGluedDagSkeletonForLayering();
            LayerArrays layerArrays = this.CalculateLayersWithoutAspectRatio();

            this.UpdateNodePositionData();


            return layerArrays;
        }


        /// <summary>
        /// pushes positions from the anchors to node Centers
        /// </summary>
        private void UpdateNodePositionData() {
            this.TryToSatisfyMinWidhtAndMinHeight();

            for (int i = 0; i < this.IntGraph.Nodes.Count && i < this.database.Anchors.Length; i++) {
                this.IntGraph.Nodes[i].Center = this.database.Anchors[i].Origin;
            }
        }

        private void TryToSatisfyMinWidhtAndMinHeight() {
            this.TryToSatisfyMinWidth();
            this.TryToSatisfyMinHeight();
        }

        private void TryToSatisfyMinWidth() {
            if (this.sugiyamaSettings.MinimalWidth == 0) {
                return;
            }

            double w = this.GetCurrentWidth();
            if (w < this.sugiyamaSettings.MinimalWidth) {
                this.StretchWidth();
            }
        }

        /// <summary>
        /// </summary>
        private void StretchWidth() {
            //calculate the desired span of the anchor centers and the current span of anchor center
            var desiredSpan = new RealNumberSpan();
            foreach (Node node in this.originalGraph.Nodes) {
                desiredSpan.AddValue(node.BoundingBox.Width / 2);
                desiredSpan.AddValue(this.sugiyamaSettings.MinimalWidth - node.BoundingBox.Width / 2);
            }

            var currentSpan = new RealNumberSpan();
            foreach (Anchor anchor in this.NodeAnchors()) {
                currentSpan.AddValue(anchor.X);
            }

            if (currentSpan.Length > ApproximateComparer.DistanceEpsilon) {
                double stretch = desiredSpan.Length / currentSpan.Length;
                if (stretch > 1) {
                    foreach (Anchor a in this.anchors) {
                        a.X *= stretch;
                    }
                }
            }
        }

        private void TryToSatisfyMinHeight() {
            if (this.sugiyamaSettings.MinimalHeight == 0) {
                return;
            }

            double h = this.GetCurrentHeight();
            if (h < this.sugiyamaSettings.MinimalHeight) {
                this.StretchHeight();
            }
        }

        private double GetCurrentHeight() {
            var span = new RealNumberSpan();
            foreach (Anchor anchor in this.NodeAnchors()) {
                span.AddValue(anchor.Top);
                span.AddValue(anchor.Bottom);
            }
            return span.Length;
        }

        private void StretchHeight() {
            var desiredSpan = new RealNumberSpan();
            foreach (Node node in this.originalGraph.Nodes) {
                desiredSpan.AddValue(node.BoundingBox.Height / 2);
                desiredSpan.AddValue(this.sugiyamaSettings.MinimalHeight - node.BoundingBox.Height / 2);
            }

            var currentSpan = new RealNumberSpan();
            foreach (Anchor anchor in this.NodeAnchors()) {
                currentSpan.AddValue(anchor.Y);
            }

            if (currentSpan.Length > ApproximateComparer.DistanceEpsilon) {
                double stretch = desiredSpan.Length / currentSpan.Length;
                if (stretch > 1) {
                    foreach (Anchor a in this.anchors) {
                        a.Y *= stretch;
                    }
                }
            }
        }

        private IEnumerable<Anchor> NodeAnchors() {
            return this.anchors.Take(Math.Min(this.IntGraph.Nodes.Count, this.anchors.Length));
        }

        private double GetCurrentWidth() {
            var span = new RealNumberSpan();
            foreach (Anchor anchor in this.NodeAnchors()) {
                span.AddValue(anchor.Left);
                span.AddValue(anchor.Right);
            }
            return span.Length;
        }

        private void StretchToDesiredAspectRatio(double aspectRatio, double desiredAr) {
            if (aspectRatio > desiredAr) {
                this.StretchInYDirection(aspectRatio / desiredAr);
            } else if (aspectRatio < desiredAr) {
                this.StretchInXDirection(desiredAr / aspectRatio);
            }
        }

        private void StretchInYDirection(double scaleFactor) {
            double center = (this.originalGraph.BoundingBox.Top + this.originalGraph.BoundingBox.Bottom) / 2;
            foreach (Anchor a in this.Database.Anchors) {
                a.BottomAnchor *= scaleFactor;
                a.TopAnchor *= scaleFactor;
                a.Y = center + scaleFactor * (a.Y - center);
            }
            double h = this.originalGraph.Height * scaleFactor;
            this.originalGraph.BoundingBox = new Rectangle(this.originalGraph.BoundingBox.Left, center + h / 2,
                                                      this.originalGraph.BoundingBox.Right, center - h / 2);
        }

        private void StretchInXDirection(double scaleFactor) {
            double center = (this.originalGraph.BoundingBox.Left + this.originalGraph.BoundingBox.Right) / 2;
            foreach (Anchor a in this.Database.Anchors) {
                a.LeftAnchor *= scaleFactor;
                a.RightAnchor *= scaleFactor;
                a.X = center + scaleFactor * (a.X - center);
            }
            double w = this.originalGraph.Width * scaleFactor;
            this.originalGraph.BoundingBox = new Rectangle(center - w / 2, this.originalGraph.BoundingBox.Top, center + w / 2,
                                                      this.originalGraph.BoundingBox.Bottom);
        }

        private LayerArrays CalculateLayersWithoutAspectRatio() {
            LayerArrays layerArrays = this.CalculateYLayers();

            if (this.constrainedOrdering == null) {
                this.DecideIfUsingFastXCoordCalculation(layerArrays);

                this.CalculateAnchorsAndYPositions(layerArrays);

                if (this.Brandes) {
                    this.CalculateXPositionsByBrandes(layerArrays);
                } else {
                    this.CalculateXLayersByGansnerNorth(layerArrays);
                }
            } else {
                this.anchors = this.database.Anchors;
            }

            this.OptimizeEdgeLabelsLocations();

            this.engineLayerArrays = layerArrays;
            this.StraightensShortEdges();

            double aspectRatio;
            this.CalculateOriginalGraphBox(out aspectRatio);

            if (this.sugiyamaSettings.AspectRatio != 0) {
                this.StretchToDesiredAspectRatio(aspectRatio, this.sugiyamaSettings.AspectRatio);
            }

            return layerArrays;
        }

        private void DecideIfUsingFastXCoordCalculation(LayerArrays layerArrays) {
            if (layerArrays.X.Length >= this.sugiyamaSettings.BrandesThreshold) {
                this.Brandes = true;
            } else {
                string s = Environment.GetEnvironmentVariable("Brandes");
                if (!String.IsNullOrEmpty(s) && String.Compare(s, "on", true, CultureInfo.CurrentCulture) == 0) {
                    this.Brandes = true;
                }
            }
        }

        private void StraightensShortEdges() {
            for (; this.StraightenEdgePaths(); ) { }
        }

        private bool StraightenEdgePaths() {
            bool ret = false;
            foreach (PolyIntEdge e in this.database.AllIntEdges) {
                if (e.LayerSpan == 2) {
                    ret =
                        this.ShiftVertexWithNeighbors(e.LayerEdges[0].Source, e.LayerEdges[0].Target, e.LayerEdges[1].Target) ||
                        ret;
                }
            }

            return ret;
            //foreach (LayerEdge[][] edgeStrings in this.dataBase.RefinedEdges.Values)
            //    if (edgeStrings[0].Length == 2)
            //        foreach (LayerEdge[] edgePath in edgeStrings)
            //            ret = ShiftVertexWithNeighbors(edgePath[0].Source, edgePath[0].Target, edgePath[1].Target) || ret;
            //return ret;
        }

        private bool ShiftVertexWithNeighbors(int u, int i, int v) {
            Anchor upper = this.database.Anchors[u];
            Anchor lower = this.database.Anchors[v];
            Anchor iAnchor = this.database.Anchors[i];
            //calculate the ideal x position for i
            // (x- upper.x)/(iAnchor.y-upper.y)=(lower.x-upper.x)/(lower.y-upper.y)

            double x = (iAnchor.Y - upper.Y) * (lower.X - upper.X) / (lower.Y - upper.Y) + upper.X;
            const double eps = 0.0001;
            if (x > iAnchor.X + eps) {
                return this.TryShiftToTheRight(x, i);
            }

            if (x < iAnchor.X - eps) {
                return this.TryShiftToTheLeft(x, i);
            }

            return false;
        }

        private bool TryShiftToTheLeft(double x, int v) {
            int[] layer = this.engineLayerArrays.Layers[this.engineLayerArrays.Y[v]];
            int vPosition = this.engineLayerArrays.X[v];
            if (vPosition > 0) {
                Anchor uAnchor = this.database.Anchors[layer[vPosition - 1]];
                double allowedX = Math.Max(
                    uAnchor.Right + this.sugiyamaSettings.NodeSeparation + this.database.Anchors[v].LeftAnchor, x);
                if (allowedX < this.database.Anchors[v].X - 1) {
                    this.database.Anchors[v].X = allowedX;
                    return true;
                }
                return false;
            }
            this.database.Anchors[v].X = x;
            return true;
        }

        private bool TryShiftToTheRight(double x, int v) {
            int[] layer = this.engineLayerArrays.Layers[this.engineLayerArrays.Y[v]];
            int vPosition = this.engineLayerArrays.X[v];
            if (vPosition < layer.Length - 1) {
                Anchor uAnchor = this.database.Anchors[layer[vPosition + 1]];
                double allowedX = Math.Min(
                    uAnchor.Left - this.sugiyamaSettings.NodeSeparation - this.database.Anchors[v].RightAnchor, x);
                if (allowedX > this.database.Anchors[v].X + 1) {
                    this.database.Anchors[v].X = allowedX;
                    return true;
                }
                return false;
            }
            this.database.Anchors[v].X = x;
            return true;
        }

        private LayerArrays CalculateYLayers() {
            LayerArrays layerArrays =
                this.YLayeringAndOrdering(new RecoveryLayerCalculator(this.recoveredLayerArrays));
            if (this.constrainedOrdering != null) {
                return layerArrays;
            }

            return this.InsertLayersIfNeeded(layerArrays);
        }

        private VerticalConstraintsForSugiyama VerticalConstraints {
            get { return this.sugiyamaSettings.VerticalConstraints; }
        }

        private HorizontalConstraintsForSugiyama HorizontalConstraints {
            get { return this.sugiyamaSettings.HorizontalConstraints; }
        }

        private void CalculateAnchorsAndYPositions(LayerArrays layerArrays) {
            CalculateAnchorSizes(this.database, out this.anchors, this.properLayeredGraph, this.originalGraph, this.IntGraph, this.sugiyamaSettings);
            CalcInitialYAnchorLocations(layerArrays, 500, this.originalGraph, this.database, this.IntGraph, this.sugiyamaSettings,
                                        this.LayersAreDoubled);
        }

        /// <summary>
        /// put some labels to the left of the splines if it makes sense
        /// </summary>
        private void OptimizeEdgeLabelsLocations() {
            for (int i = 0; i < this.anchors.Length; i++) {
                Anchor a = this.anchors[i];
                if (a.LabelToTheRightOfAnchorCenter) {
                    //by default the label is put to the right of the spline
                    Anchor predecessor;
                    Anchor successor;
                    this.GetSuccessorAndPredecessor(i, out predecessor, out successor);
                    if (!TryToPutLabelOutsideOfAngle(a, predecessor, successor)) {
                        double sumNow = (predecessor.Origin - a.Origin).Length + (successor.Origin - a.Origin).Length;
                        double nx = a.Right - a.LeftAnchor; //new potential anchor center 
                        var xy = new Point(nx, a.Y);
                        double sumWouldBe = (predecessor.Origin - xy).Length + (successor.Origin - xy).Length;
                        if (sumWouldBe < sumNow) //we need to swap
{
                            PutLabelToTheLeft(a);
                        }
                    }
                }
            }
        }

        private static bool TryToPutLabelOutsideOfAngle(Anchor a, Anchor predecessor, Anchor successor) {
            if (a.LabelToTheRightOfAnchorCenter) {
                if (Point.GetTriangleOrientation(predecessor.Origin, a.Origin, successor.Origin) ==
                    TriangleOrientation.Clockwise) {
                    return true;
                }

                double la = a.LeftAnchor;
                double ra = a.RightAnchor;
                double x = a.X;
                PutLabelToTheLeft(a);
                if (Point.GetTriangleOrientation(predecessor.Origin, a.Origin, successor.Origin) ==
                    TriangleOrientation.Counterclockwise) {
                    return true;
                }

                a.X = x;
                a.LeftAnchor = la;
                a.RightAnchor = ra;
                a.LabelToTheRightOfAnchorCenter = true;
                a.LabelToTheLeftOfAnchorCenter = false;
                return false;
            }
            return false;
        }

        private static void PutLabelToTheLeft(Anchor a) {
            double r = a.Right;
            double t = a.LeftAnchor;
            a.LeftAnchor = a.RightAnchor;
            a.RightAnchor = t;
            a.X = r - a.RightAnchor;

            a.LabelToTheLeftOfAnchorCenter = true;
            a.LabelToTheRightOfAnchorCenter = false;
        }

        private void GetSuccessorAndPredecessor(int i, out Anchor p, out Anchor s) {
            int predecessor = 10; //the value does not matter, just to silence the compiler
            foreach (LayerEdge ie in this.properLayeredGraph.InEdges(i)) {
                predecessor = ie.Source; // there will be only one
            }

            int successor = 10; // the value does not matter, just to silence the compiler
            foreach (LayerEdge ie in this.properLayeredGraph.OutEdges(i)) {
                successor = ie.Target; //there will be only one
            }

            //we compare the sum of length of projections of edges (predecessor,i), (i,successor) to x in cases when the label is to the right and to the left
            p = this.anchors[predecessor];

            s = this.anchors[successor];
        }

        //void MakeXOfAnchorsPositive(LayerArrays layerArrays)
        //{
        //    //find the minimum of x of anchors
        //    //we don't care about y since they are not part of the unknown variables

        //    double min;
        //    if (anchors.Length > 0)
        //    {
        //        min = anchors[0].X;


        //        for (int i = 1; i < anchors.Length; i++)
        //        {
        //            double x = anchors[i].X;
        //            if (x < min)
        //                min = x;
        //        }

        //        //span of the layers in Y direction
        //        int[][] layers = layerArrays.Layers;
        //        double s = anchors[layers[layers.Length - 1][0]].Y - anchors[layers[0][0]].Y;
        //        //shift anchors to the right that their minimum would be equal to min, unless min is already not less than s.
        //        if (min < s)
        //        {
        //            double shift = s - min;
        //            for (int i = 0; i < anchors.Length; i++)
        //                anchors[i].X += shift;
        //        }
        //    }
        //}


        /// <summary>
        /// Create a DAG from the given graph
        /// </summary>
        private void CycleRemoval() {
            var feedbackSet = this.IntGraph.Edges.Where(e => this.recoveredLayerArrays.Y[e.Source] < this.recoveredLayerArrays.Y[e.Target]).ToArray();
            this.database.AddFeedbackSet(feedbackSet);
        }



        /// <summary>
        /// The function calculates y-layers and x-layers, 
        /// thus, in fact, defining node, including dummy nodes, locations.
        /// </summary>
        /// <param name="layerArrays"></param>
        private void CalculateXLayersByGansnerNorth(LayerArrays layerArrays) {
            this.xLayoutGraph = this.CreateXLayoutGraph(layerArrays);
            this.CalculateXLayersByGansnerNorthOnProperLayeredGraph();
        }

        private void CalculateXLayersByGansnerNorthOnProperLayeredGraph() {
            int[] xLayers = (new NetworkSimplex(this.xLayoutGraph, null)).GetLayers();

            //TestYXLayers(layerArrays, xLayers);//this will not be called in the release version

            for (int i = 0; i < this.database.Anchors.Length; i++) {
                this.anchors[i].X = xLayers[i];
            }
        }

        //[System.Diagnostics.Conditional("TEST_MSAGL")]
        //private void TestYXLayers(LayerArrays layerArrays, int[] xLayers) {
        //    foreach (IntEdge e in this.xLayoutGraph.Edges) {
        //        int s = e.Source; int targ = e.Target;
        //        if (e.Source >= layeredGraph.Nodes.Count) {
        //            if (xLayoutGraph.OutEdges(s).Count != 2 || xLayoutGraph.InEdges(s).Count != 0)
        //                Report("must be two out edges and none incoming");

        //            if (targ >= layeredGraph.Nodes.Count)
        //                Report("an illegal edge");

        //        } else {

        //            if (layerArrays.Y[s] != layerArrays.Y[targ])
        //                Report("layers don't coincide");

        //            if (layerArrays.X[s] - 1 != layerArrays.X[targ])
        //                Report("wrong input");

        //            if (xLayers[s] <= xLayers[targ])
        //                Report("wrong xlayering");

        //        }
        //    }
        //}

        /// <summary>
        /// Creating a proper layered graph, a graph where each 
        /// edge goes only one layer down from the i+1-th layer to the i-th layer.
        /// </summary>
        /// <param name="layering"></param>
        /// <param name="layerArrays"></param>
        private void CreateProperLayeredGraph(int[] layering, out LayerArrays layerArrays) {
            int n = layering.Length;
            int nOfVv = 0;

            foreach (PolyIntEdge e in this.database.SkeletonEdges()) {
                int span = EdgeSpan(layering, e);
                Debug.Assert(span >= 0);
                if (span > 0) {
                    e.LayerEdges = new LayerEdge[span];
                }

                int pe = 0; //offset in the string
                if (span > 1) {
                    //we create span-2 dummy nodes and span new edges
                    int d0 = n + nOfVv++;
                    var layerEdge = new LayerEdge(e.Source, d0, e.CrossingWeight);
                    e.LayerEdges[pe++] = layerEdge;
                    //create span-2 internal edges all from dummy nodes
                    for (int j = 0; j < span - 2; j++) {
                        d0++;
                        nOfVv++;
                        layerEdge = new LayerEdge(d0 - 1, d0, e.CrossingWeight);
                        e.LayerEdges[pe++] = layerEdge;
                    }

                    layerEdge = new LayerEdge(d0, e.Target, e.CrossingWeight);
                    e.LayerEdges[pe] = layerEdge;
                } else if (span == 1) {
                    var layerEdge = new LayerEdge(e.Source, e.Target, e.CrossingWeight);
                    e.LayerEdges[pe] = layerEdge;
                }
            }

            var extendedVertexLayering = new int[this.originalGraph.Nodes.Count + nOfVv];

            foreach (PolyIntEdge e in this.database.SkeletonEdges()) {
                if (e.LayerEdges != null) {
                    int l = layering[e.Source];
                    extendedVertexLayering[e.Source] = l--;
                    foreach (LayerEdge le in e.LayerEdges) {
                        extendedVertexLayering[le.Target] = l--;
                    }
                } else {
                    extendedVertexLayering[e.Source] = layering[e.Source];
                    extendedVertexLayering[e.Target] = layering[e.Target];
                }
            }

            this.properLayeredGraph =
                new ProperLayeredGraph(new BasicGraph<Node, PolyIntEdge>(this.database.SkeletonEdges(), layering.Length));
            this.properLayeredGraph.BaseGraph.Nodes = this.IntGraph.Nodes;
            layerArrays = new LayerArrays(extendedVertexLayering);
        }

        private ConstrainedOrdering constrainedOrdering;

        private LayerArrays YLayeringAndOrdering(LayerCalculator layering) {

            int[] yLayers = layering.GetLayers();
            yLayers = this.ExtendLayeringToUngluedSameLayerVertices(yLayers);
            var layerArrays = new LayerArrays(yLayers);
            //if (!SugiyamaSettings.UseEdgeBundling && (HorizontalConstraints == null || HorizontalConstraints.IsEmpty)) {
            if (this.HorizontalConstraints == null || this.HorizontalConstraints.IsEmpty) {
                layerArrays = this.YLayeringAndOrderingWithoutHorizontalConstraints(layerArrays);
                return layerArrays;
            }
            this.constrainedOrdering = new ConstrainedOrdering(this.originalGraph, this.IntGraph, layerArrays.Y, this.nodeIdToIndex,
                                                          this.database, this.sugiyamaSettings);
            this.constrainedOrdering.Calculate();
            this.properLayeredGraph = this.constrainedOrdering.ProperLayeredGraph;


            // SugiyamaLayoutSettings.ShowDatabase(this.database);
            return this.constrainedOrdering.LayerArrays;
        }

        private LayerArrays YLayeringAndOrderingWithoutHorizontalConstraints(LayerArrays layerArrays) {
            this.CreateProperLayeredGraph(layerArrays.Y, out layerArrays);
            this.GetXCoordinatesOfVirtualNodesOfTheProperLayeredGraph(layerArrays);
            this.OrderLayers(layerArrays);
             MetroMapOrdering.UpdateLayerArrays(this.properLayeredGraph, layerArrays);
            return layerArrays;
        }

        private void GetXCoordinatesOfVirtualNodesOfTheProperLayeredGraph(LayerArrays layerArrays) {
            foreach (var skeletonEdge in this.database.SkeletonEdges()) {
                this.GetXCoordinatesOfVirtualNodesOfTheProperLayeredGraphForSkeletonEdge(skeletonEdge, layerArrays);
            }
        }

        private void GetXCoordinatesOfVirtualNodesOfTheProperLayeredGraphForSkeletonEdge(PolyIntEdge intEdge, LayerArrays layerArrays) {
            if (intEdge.LayerEdges == null || intEdge.LayerEdges.Count < 2) {
                return;
            }

            var edgeCurve = intEdge.Edge.Curve;
            var layerIndex = layerArrays.Y[intEdge.Source] - 1;//it is the layer of the highest virtual node of the edge
            for (int i = 1; i < intEdge.LayerEdges.Count; i++) {
                var v = intEdge.LayerEdges[i].Source;
                var layerY = this.layerToRecoveredYCoordinates[layerIndex--];
                var layerLine = new LineSegment(new Point(this.originalGraph.Left, layerY), new Point(this.originalGraph.Right, layerY));
                var intersection=Curve.CurveCurveIntersectionOne(edgeCurve, layerLine, false);

                this.skeletonVirtualVerticesToX[v] =intersection.IntersectionPoint.X;
                
            }
        }

        private void OrderLayers(LayerArrays layerArrays) {
            foreach (var layer in layerArrays.Layers) {
                this.OrderLayerBasedOnRecoveredXCoords(layer);
            }
        }

        private void OrderLayerBasedOnRecoveredXCoords(int[] layer) {
            if (layer.Length <= 1) {
                return;
            }

            var keys = new double[layer.Length];
            for (int i = 0; i < layer.Length; i++) {
                int v = layer[i];
                keys[i] = this.properLayeredGraph.IsVirtualNode(v)
                              ? this.skeletonVirtualVerticesToX[v]
                              : this.originalGraph.Nodes[v].Center.X;
            }

            Array.Sort(keys, layer);

        }

        private void CalculateXPositionsByBrandes(LayerArrays layerArrays) {
            XCoordsWithAlignment.CalculateXCoordinates(layerArrays, this.properLayeredGraph, this.originalGraph.Nodes.Count,
                                                       this.database.Anchors, this.sugiyamaSettings.NodeSeparation);
        }

        private void CalculateOriginalGraphBox(out double aspectRatio) {
            aspectRatio = 0;
            if (this.anchors.Length > 0) {
                var box = new Rectangle(this.anchors[0].Left, this.anchors[0].Top, this.anchors[0].Right, this.anchors[0].Bottom);

                for (int i = 1; i < this.anchors.Length; i++) {
                    Anchor a = this.anchors[i];
                    box.Add(a.LeftTop);
                    box.Add(a.RightBottom);
                }

                aspectRatio = box.Width / box.Height;

                double delta = (box.LeftTop - box.RightBottom).Length / 2;

                var del = new Point(-delta, delta);
                box.Add(box.LeftTop + del);
                box.Add(box.RightBottom - del);

                this.originalGraph.BoundingBox = box;
            }
        }

        private LayerArrays InsertLayersIfNeeded(LayerArrays layerArrays) {
            bool needToInsertLayers = false;
            bool multipleEdges = false;

            this.InsertVirtualEdgesIfNeeded(layerArrays);

            this.AnalyzeNeedToInsertLayersAndHasMultiedges(layerArrays, ref needToInsertLayers, ref multipleEdges);

            if (needToInsertLayers) {
                LayerInserter.InsertLayers(ref this.properLayeredGraph, ref layerArrays, this.database, this.IntGraph);
                this.LayersAreDoubled = true;
            } else if (multipleEdges) {
                EdgePathsInserter.InsertPaths(ref this.properLayeredGraph, ref layerArrays, this.database, this.IntGraph);
            }

            this.RecreateIntGraphFromDataBase();

            return layerArrays;
        }

        private bool LayersAreDoubled { get; set; }

        private void RecreateIntGraphFromDataBase() {
            var edges = new List<PolyIntEdge>();
            foreach (var list in this.database.Multiedges.Values) {
                edges.AddRange(list);
            }

            this.IntGraph.SetEdges(edges, this.IntGraph.NodeCount);
        }

        private void AnalyzeNeedToInsertLayersAndHasMultiedges(LayerArrays layerArrays, ref bool needToInsertLayers,
                                                       ref bool multipleEdges) {
            foreach (PolyIntEdge ie in this.IntGraph.Edges) {
                if (ie.HasLabel && layerArrays.Y[ie.Source] != layerArrays.Y[ie.Target]) {
                    //if an edge is a flat edge then
                    needToInsertLayers = true;
                    break;
                }
            }

            if (needToInsertLayers == false && this.constrainedOrdering == null) {
                //if we have constrains the multiple edges have been already represented in layers
                foreach (var kv in this.database.Multiedges) {
                    if (kv.Value.Count > 1) {
                        multipleEdges = true;
                        if (layerArrays.Y[kv.Key.x] - layerArrays.Y[kv.Key.y] == 1) {
                            //there is a multi edge spanning exactly one layer; unfortunately we need to introduce virtual vertices for 
                            //the edges middle points 
                            needToInsertLayers = true;
                            break;
                        }
                    }
                }
            }
        }

        private void InsertVirtualEdgesIfNeeded(LayerArrays layerArrays) {
            if (this.constrainedOrdering != null) //if there are constraints we handle multiedges correctly
{
                return;
            }

            foreach (var kv in this.database.Multiedges) {
                // If there are an even number of multi-edges between two nodes then
                //  add a virtual edge in the multi-edge dict to improve the placement, but only in case when the edge goes down only one layer.         
                if (kv.Value.Count % 2 == 0 && layerArrays.Y[kv.Key.First] - 1 == layerArrays.Y[kv.Key.Second]) {
                    var newVirtualEdge = new PolyIntEdge(kv.Key.First, kv.Key.Second);
                    newVirtualEdge.Edge = new Edge();
                    newVirtualEdge.IsVirtualEdge = true;
                    kv.Value.Insert(kv.Value.Count / 2, newVirtualEdge);
                    this.IntGraph.AddEdge(newVirtualEdge);
                }
            }
        }

        private int[] ExtendLayeringToUngluedSameLayerVertices(int[] p) {
            VerticalConstraintsForSugiyama vc = this.VerticalConstraints;
            for (int i = 0; i < p.Length; i++) {
                p[i] = p[vc.NodeToRepr(i)];
            }

            return p;
        }

        internal static void CalculateAnchorSizes(Database database, out Anchor[] anchors,
                                                  ProperLayeredGraph properLayeredGraph, GeometryGraph originalGraph,
                                                  BasicGraph<Node, PolyIntEdge> intGraph, SugiyamaLayoutSettings settings) {
            database.Anchors = anchors = new Anchor[properLayeredGraph.NodeCount];

            for (int i = 0; i < anchors.Length; i++) {
                anchors[i] = new Anchor(settings.LabelCornersPreserveCoefficient);
            }

            //go over the old vertices
            for (int i = 0; i < originalGraph.Nodes.Count; i++) {
                CalcAnchorsForOriginalNode(i, intGraph, anchors, database, settings);
            }

            //go over virtual vertices
            foreach (PolyIntEdge intEdge in database.AllIntEdges) {
                if (intEdge.LayerEdges != null) {
                    foreach (LayerEdge layerEdge in intEdge.LayerEdges) {
                        int v = layerEdge.Target;
                        if (v != intEdge.Target) {
                            Anchor anchor = anchors[v];
                            if (!database.MultipleMiddles.Contains(v)) {
                                anchor.LeftAnchor = anchor.RightAnchor = VirtualNodeWidth / 2.0f;
                                anchor.TopAnchor = anchor.BottomAnchor = VirtualNodeHeight(settings) / 2.0f;
                            } else {
                                anchor.LeftAnchor = anchor.RightAnchor = VirtualNodeWidth * 4;
                                anchor.TopAnchor = anchor.BottomAnchor = VirtualNodeHeight(settings) / 2.0f;
                            }
                        }
                    }
                    //fix label vertices      
                    if (intEdge.HasLabel) {
                        int lj = intEdge.LayerEdges[intEdge.LayerEdges.Count / 2].Source;
                        Anchor a = anchors[lj];
                        double w = intEdge.LabelWidth, h = intEdge.LabelHeight;
                        a.RightAnchor = w;
                        a.LeftAnchor = VirtualNodeWidth * 8;

                        if (a.TopAnchor < h / 2.0) {
                            a.TopAnchor = a.BottomAnchor = h / 2.0;
                        }

                        a.LabelToTheRightOfAnchorCenter = true;
                    }
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>the height of the graph+spaceBeforeMargins</returns>
        internal static void CalcInitialYAnchorLocations(LayerArrays layerArrays, double spaceBeforeMargins,
                                                         GeometryGraph originalGraph, Database database,
                                                         BasicGraphOnEdges<PolyIntEdge> intGraph,
                                                         SugiyamaLayoutSettings settings,
                                                         bool layersAreDoubled) {
            Anchor[] anchors = database.Anchors;
            double ymax = originalGraph.Margins + spaceBeforeMargins; //setting up y coord - going up by y-layers
            int i = 0;
            foreach (var yLayer in layerArrays.Layers) {
                double bottomAnchorMax = 0;
                double topAnchorMax = 0;
                foreach (int j in yLayer) {
                    Anchor p = anchors[j];
                    if (p.BottomAnchor > bottomAnchorMax) {
                        bottomAnchorMax = p.BottomAnchor;
                    }

                    if (p.TopAnchor > topAnchorMax) {
                        topAnchorMax = p.TopAnchor;
                    }
                }

                MakeVirtualNodesHigh(yLayer, bottomAnchorMax, topAnchorMax, originalGraph.Nodes.Count, database.Anchors);

                double flatEdgesHeight = SetFlatEdgesForLayer(database, layerArrays, i, intGraph, settings, ymax);

                double y = ymax + bottomAnchorMax + flatEdgesHeight;
                foreach (int j in yLayer) {
                    anchors[j].Y = y;
                }

                double layerSep = settings.ActualLayerSeparation(layersAreDoubled);
                ymax = y + topAnchorMax + layerSep;
                i++;
            }
            SetFlatEdgesForLayer(database, layerArrays, i, intGraph, settings, ymax);
        }

        private static double SetFlatEdgesForLayer(Database database, LayerArrays layerArrays, int i,
                                           BasicGraphOnEdges<PolyIntEdge> intGraph, SugiyamaLayoutSettings settings, double ymax) {
            double flatEdgesHeight = 0;
            if (i > 0) {
                //looking for flat edges on the previous level                
                //we stack labels of multiple flat edges on top of each other
                IEnumerable<IntPair> flatPairs = GetFlatPairs(layerArrays.Layers[i - 1], layerArrays.Y,
                                                              intGraph);
                if (flatPairs.Any()) {
                    double dyOfFlatEdge = settings.LayerSeparation / 3;
                    double ym = ymax;
                    flatEdgesHeight =
                        (from pair in flatPairs
                         select SetFlatEdgesLabelsHeightAndPositionts(pair, ym, dyOfFlatEdge, database)).
                            Max();
                }
            }
            return flatEdgesHeight;
        }

        private static double SetFlatEdgesLabelsHeightAndPositionts(IntPair pair, double ymax, double dy, Database database) {
            double height = 0;
            List<PolyIntEdge> list = database.GetMultiedge(pair);
            foreach (PolyIntEdge edge in list) {
                height += dy;
                Label label = edge.Edge.Label;
                if (label != null) {
                    label.Center = new Point(label.Center.X, ymax + height + label.Height / 2);
                    height += label.Height;
                }
            }
            return height;
        }

        private static IEnumerable<IntPair> GetFlatPairs(int[] layer, int[] layering, BasicGraphOnEdges<PolyIntEdge> intGraph) {
            return new Set<IntPair>(from v in layer
                                    where v < intGraph.NodeCount
                                    from edge in intGraph.OutEdges(v)
                                    where layering[edge.Source] == layering[edge.Target]
                                    select new IntPair(edge.Source, edge.Target));
        }

        private static void MakeVirtualNodesHigh(int[] yLayer, double bottomAnchorMax, double topAnchorMax,
                                         int originalNodeCount, Anchor[] anchors) {
            if (LayerIsOriginal(yLayer, originalNodeCount)) {
                foreach (int j in yLayer) {
                    if (j >= originalNodeCount) {
                        Anchor p = anchors[j];
                        p.BottomAnchor = bottomAnchorMax;
                        p.TopAnchor = topAnchorMax;
                    }
                }
            }
        }

        private static bool LayerIsOriginal(int[] yLayer, int origNodeCount) {
            foreach (int j in yLayer) {
                if (j < origNodeCount) {
                    return true;
                }
            }

            return false;
        }

        private static void CalcAnchorsForOriginalNode(int i, BasicGraph<Node, PolyIntEdge> intGraph, Anchor[] anchors,
                                               Database database, SugiyamaLayoutSettings settings) {
            double leftAnchor = 0;
            double rightAnchor = leftAnchor;
            double topAnchor = 0;
            double bottomAnchor = topAnchor;

            //that's what we would have without the label and multiedges 

            if (intGraph.Nodes != null) {
                Node node = intGraph.Nodes[i];
                ExtendStandardAnchors(ref leftAnchor, ref rightAnchor, ref topAnchor, ref bottomAnchor, node);
            }

            RightAnchorMultiSelfEdges(i, ref rightAnchor, ref topAnchor, ref bottomAnchor, database, settings);

            double hw = settings.MinNodeWidth / 2;
            if (leftAnchor < hw) {
                leftAnchor = hw;
            }

            if (rightAnchor < hw) {
                rightAnchor = hw;
            }

            double hh = settings.MinNodeHeight / 2;

            if (topAnchor < hh) {
                topAnchor = hh;
            }

            if (bottomAnchor < hh) {
                bottomAnchor = hh;
            }

            anchors[i] = new Anchor(leftAnchor, rightAnchor, topAnchor, bottomAnchor, intGraph.Nodes[i],
                                    settings.LabelCornersPreserveCoefficient) { Padding = intGraph.Nodes[i].Padding };
#if TEST_MSAGL
            anchors[i].UserData = intGraph.Nodes[i].UserData;
#endif
        }

        private static void RightAnchorMultiSelfEdges(int i, ref double rightAnchor, ref double topAnchor,
                                              ref double bottomAnchor, Database database,
                                              SugiyamaLayoutSettings settings) {
            double delta = WidthOfSelfEdge(database, i, ref rightAnchor, ref topAnchor, ref bottomAnchor, settings);

            rightAnchor += delta;
        }

        private static double WidthOfSelfEdge(Database database, int i, ref double rightAnchor, ref double topAnchor,
                                      ref double bottomAnchor, SugiyamaLayoutSettings settings) {
            double delta = 0;
            List<PolyIntEdge> multiedges = database.GetMultiedge(i, i);
            //it could be a multiple self edge
            if (multiedges.Count > 0) {
                foreach (PolyIntEdge e in multiedges) {
                    if (e.Edge.Label != null) {
                        rightAnchor += e.Edge.Label.Width;
                        if (topAnchor < e.Edge.Label.Height / 2.0) {
                            topAnchor = bottomAnchor = e.Edge.Label.Height / 2.0f;
                        }
                    }
                }

                delta += (settings.NodeSeparation + settings.MinNodeWidth) * multiedges.Count;
            }
            return delta;
        }

        private static void ExtendStandardAnchors(ref double leftAnchor, ref double rightAnchor, ref double topAnchor,
                                          ref double bottomAnchor, Node node) {
            double w = node.Width;
            double h = node.Height;


            w /= 2.0;
            h /= 2.0;


            rightAnchor = leftAnchor = w;
            topAnchor = bottomAnchor = h;
        }

        ///// <summary>
        ///// A quote from Gansner93.
        ///// The method involves constructing an auxiliary graph as illustrated in figure 4-2.
        ///// This transformation is the graphical analogue of the algebraic 
        ///// transformation mentioned above for removing the absolute values 
        ///// from the optimization problem. The nodes of the auxiliary graph Gў are the nodes of 
        ///// the original graph G plus, for every edge e in G, there is a new node ne. 
        ///// There are two kinds of edges in Gў. One edge class encodes the 
        ///// cost of the original edges. Every edge e = (u,v) in G is replaced by two edges (ne ,u)
        ///// and (ne, v) with d = 0 and w = w(e)W(e). The other class of edges separates nodes in the same layer. 
        ///// If v is the left neighbor of w, then Gў has an edge f = e(v,w) with d( f ) = r(v,w) and 
        ///// w( f ) = 0. This edge forces the nodes to be sufficiently 
        ///// separated but does not affect the cost of the layout.
        private XLayoutGraph CreateXLayoutGraph(LayerArrays layerArrays) {
            int nOfVerts = this.properLayeredGraph.NodeCount;

            //create edges of XLayoutGraph
            var edges = new List<PolyIntEdge>();

            foreach (LayerEdge e in this.properLayeredGraph.Edges) {
                var n1 = new PolyIntEdge(nOfVerts, e.Source);
                var n2 = new PolyIntEdge(nOfVerts, e.Target);
                n1.Weight = n2.Weight = 1;
                n1.Separation = 0; //these edge have 0 separation
                n2.Separation = 0;
                nOfVerts++;
                edges.Add(n1);
                edges.Add(n2);
            }

            foreach (var layer in layerArrays.Layers) {
                for (int i = layer.Length - 1; i > 0; i--) {
                    int source = layer[i];
                    int target = layer[i - 1];
                    var ie = new PolyIntEdge(source, target);
                    Anchor sourceAnchor = this.database.Anchors[source];
                    Anchor targetAnchor = this.database.Anchors[target];

                    double sep = sourceAnchor.LeftAnchor + targetAnchor.RightAnchor + this.sugiyamaSettings.NodeSeparation;

                    ie.Separation = (int)(sep + 1);

                    edges.Add(ie);
                }
            }

            var ret = new XLayoutGraph(this.IntGraph, this.properLayeredGraph, layerArrays, edges, nOfVerts);
            ret.SetEdgeWeights();
            return ret;
        }

        private static LayerArrays RecoverOriginalLayersAndSettings(GeometryGraph geometryGraph, out SugiyamaLayoutSettings sugiyamaLayoutSettings) {
            sugiyamaLayoutSettings = new SugiyamaLayoutSettings();

            return RecoverOriginalHorizontalLayers(geometryGraph, sugiyamaLayoutSettings) ??
                   RecoverOriginalVerticalLayers(geometryGraph, sugiyamaLayoutSettings);
        }

        private static LayerArrays RecoverOriginalHorizontalLayers(GeometryGraph geometryGraph, SugiyamaLayoutSettings sugiyamaLayoutSettings) {
            var nodes = geometryGraph.Nodes;
            var list = new List<int>();
            for (int i = 0; i < nodes.Count; i++) {
                list.Add(i);
            }

            var layers = list.GroupBy(i => nodes[i].Center.Y);
            var layerList = new List<int[]>();
            foreach (var layer in layers) {
                layerList.Add(layer.ToArray());
            }

            sugiyamaLayoutSettings.LayerSeparation = double.PositiveInfinity;

            layerList = new List<int[]>(layerList.OrderBy(l => nodes[l[0]].Center.Y));
            //check that the layers are separated
            for (int i = 0; i < layerList.Count - 1; i++) {
                var topOfI = layerList[i].Max(j => nodes[j].BoundingBox.Top);
                var bottomOfINext = layerList[i + 1].Min(j => nodes[j].BoundingBox.Bottom);
                var layerSep = bottomOfINext - topOfI;
                if (layerSep <= 0) {
                    sugiyamaLayoutSettings.Transformation = null;
                    return null;
                }
                sugiyamaLayoutSettings.LayerSeparation = Math.Min(sugiyamaLayoutSettings.LayerSeparation, layerSep);
            }
            var nodesToLayers = new int[nodes.Count];

            for (int i = 0; i < layerList.Count; i++) {
                var layer = layerList[i];
                for (int j = 0; j < layer.Length; j++) {
                    nodesToLayers[layer[j]] = i;
                }
            }

            sugiyamaLayoutSettings.Transformation = geometryGraph.Edges.Any(e => e.Source.Center.Y > e.Target.Center.Y)
                                 ? new PlaneTransformation()
                                 : PlaneTransformation.Rotation(Math.PI);


            return new LayerArrays(nodesToLayers);
        }

        private static LayerArrays RecoverOriginalVerticalLayers(GeometryGraph geometryGraph, SugiyamaLayoutSettings sugiyamaLayoutSettings) {

            var nodes = geometryGraph.Nodes;
            var list = new List<int>();
            for (int i = 0; i < geometryGraph.Nodes.Count; i++) {
                list.Add(i);
            }

            var layers = list.GroupBy(i => nodes[i].Center.X);
            var layerList = new List<int[]>();
            foreach (var layer in layers) {
                layerList.Add(layer.ToArray());
            }
            layerList = new List<int[]>(layerList.OrderBy(l => nodes[l[0]].Center.X));
            sugiyamaLayoutSettings.LayerSeparation = double.PositiveInfinity;
            //check that the layers are separated
            for (int i = 0; i < layerList.Count - 1; i++) {
                var rightOfI = layerList[i].Max(j => nodes[j].BoundingBox.Right);
                var leftOfNext = layerList[i + 1].Min(j => nodes[j].BoundingBox.Left);
                var layerSep = leftOfNext - rightOfI;
                if (layerSep <= 0) {
                    return null;
                }
                sugiyamaLayoutSettings.LayerSeparation = Math.Min(sugiyamaLayoutSettings.LayerSeparation, layerSep);
            }
            var nodesToLayers = new int[nodes.Count];

            for (int i = 0; i < layerList.Count; i++) {
                var layer = layerList[i];
                for (int j = 0; j < layer.Length; j++) {
                    nodesToLayers[layer[j]] = i;
                }
            }

            sugiyamaLayoutSettings.Transformation = geometryGraph.Edges.Any(e => e.Source.Center.X > e.Target.Center.X)
                                                        ? PlaneTransformation.Rotation(-Math.PI/2)
                                                        : PlaneTransformation.Rotation(Math.PI/2);


            return new LayerArrays(nodesToLayers);
        }

        public LayeredLayoutEngine GetEngine() {
            if (this.recoveredLayerArrays == null) {
                return null; //it is not a layered layout
            }

            if (!this.sugiyamaSettings.Transformation.IsIdentity) {
                this.originalGraph.Transform(this.sugiyamaSettings.Transformation.Inverse);
            }

            this.FillLayersToRecoveredYCoordinates();
            this.CycleRemoval();
            this.Run();
            var engine= this.CreateEngine();
            if (!this.sugiyamaSettings.Transformation.IsIdentity) {
                this.originalGraph.Transform(this.sugiyamaSettings.Transformation);
            }
            return engine;
        }

        private LayeredLayoutEngine CreateEngine() {
            return new LayeredLayoutEngine(this.engineLayerArrays, this.originalGraph, this.properLayeredGraph,
                this.sugiyamaSettings, this.database, this.IntGraph, this.nodeIdToIndex, this.GluedDagSkeletonForLayering,
                this.LayersAreDoubled, this.constrainedOrdering, this.Brandes, this.xLayoutGraph);
        }
    }
}
