using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// The class is responsible for general edge bundling with ordered bundles.
    /// Currently the router will fail if there are node overlaps.
    /// </summary>
    public class BundleRouter : AlgorithmBase {
        private readonly BundlingSettings bundlingSettings;
        private readonly GeometryGraph geometryGraph;
        private readonly Edge[] regularEdges;

        private double LoosePadding { get; set; }
        //for the shortest path calculation we will use not loosePadding, but loosePadding*SuperLoosePaddingCoefficient
        internal const double SuperLoosePaddingCoefficient = 1.1;
        private readonly SdShortestPath shortestPathRouter;

        private RectangleNode<Polyline, Point> TightHierarchy { get; set; }
        private RectangleNode<Polyline, Point> LooseHierarchy { get; set; }

        ///<summary>
        /// reports the status of the bundling
        ///</summary>
        public BundlingStatus Status { get; set; }

        internal VisibilityGraph VisibilityGraph { get; set; }

        private Func<Port, Polyline> loosePolylineOfPort;

#if TEST_MSAGL
        private void CheckGraph() {
            foreach (var e in this.geometryGraph.Edges) {
                if (e.Source == e.Target) {
                    continue;
                }

                CheckPortOfNode(e.Source, e.SourcePort);
                CheckPortOfNode(e.Target, e.TargetPort);
            }
        }

        private static void CheckPortOfNode(Node node, Port nodePort) {
            if (node is Cluster) {
                Debug.Assert(nodePort is ClusterBoundaryPort || nodePort is HookUpAnywhereFromInsidePort || nodePort is CurvePort);
            }
        }
#endif

        internal BundleRouter(GeometryGraph geometryGraph, SdShortestPath shortestPathRouter,
                              VisibilityGraph visibilityGraph, BundlingSettings bundlingSettings, double loosePadding, RectangleNode<Polyline, Point> tightHierarchy,
                              RectangleNode<Polyline, Point> looseHierarchy,
                              Dictionary<EdgeGeometry, Set<Polyline>> edgeLooseEnterable, Dictionary<EdgeGeometry, Set<Polyline>> edgeTightEnterable, Func<Port, Polyline> loosePolylineOfPort) {
            ValidateArg.IsNotNull(geometryGraph, "geometryGraph");
            ValidateArg.IsNotNull(bundlingSettings, "bundlingSettings");

            this.geometryGraph = geometryGraph;
            this.bundlingSettings = bundlingSettings;
            this.regularEdges = geometryGraph.Edges.Where(e => e.Source != e.Target).ToArray();
            this.VisibilityGraph = visibilityGraph;
            this.shortestPathRouter = shortestPathRouter;
            this.LoosePadding = loosePadding;
            this.LooseHierarchy = looseHierarchy;
            this.TightHierarchy = tightHierarchy;
            this.EdgeLooseEnterable = edgeLooseEnterable;
            this.EdgeTightEnterable = edgeTightEnterable;
            this.loosePolylineOfPort = loosePolylineOfPort;
        }

        private bool ThereAreOverlaps(RectangleNode<Polyline, Point> hierarchy) {
            return RectangleNodeUtils.FindIntersectionWithProperty(hierarchy, hierarchy, Curve.CurvesIntersect);
        }

        /// <summary>
        /// edge routing with Ordered Bundles:
        /// 1. route edges with bundling
        /// 2. nudge bundles and hubs
        /// 3. order paths
        /// </summary>
        protected override void RunInternal() {
            //TimeMeasurer.DebugOutput("edge bundling started");
            if (this.ThereAreOverlaps(this.TightHierarchy)) {
                /*
                LayoutAlgorithmSettings.ShowDebugCurves(
                    TightHierarchy.GetAllLeaves().Select(p => new DebugCurve(100, 1, "black", p)).ToArray());*/
                this.Status = BundlingStatus.Overlaps;
                TimeMeasurer.DebugOutput("overlaps in edge bundling");
                return;
            }

            this.FixLocationsForHookAnywherePorts(this.geometryGraph.Edges);
            if (!this.RoutePathsWithSteinerDijkstra()) {
                this.Status = BundlingStatus.EdgeSeparationIsTooLarge;
                return;
            }
            this.FixChildParentEdges();
            if (!this.bundlingSettings.StopAfterShortestPaths) {

                var metroGraphData = new MetroGraphData(this.regularEdges.Select(e => e.EdgeGeometry).ToArray(),
                                                        this.LooseHierarchy,
                                                        this.TightHierarchy,
                                                        this.bundlingSettings,
                                                        this.shortestPathRouter.CdtProperty,
                                                        this.EdgeLooseEnterable,
                                                        this.EdgeTightEnterable,
                                                        this.loosePolylineOfPort);
                NodePositionsAdjuster.FixRouting(metroGraphData, this.bundlingSettings);
                new EdgeNudger(metroGraphData, this.bundlingSettings).Run();
                //TimeMeasurer.DebugOutput("edge bundling ended");
            }
            this.RouteSelfEdges();
            this.FixArrowheads();
        }

        /// <summary>
        /// set endpoint of the edge from child to parent (cluster) to the boundary of the parent
        /// TODO: is there a better solution?
        /// </summary>
        private void FixChildParentEdges() {
            foreach (var edge in this.regularEdges) {
                var sPort = edge.SourcePort;
                var ePort = edge.TargetPort;
                if (sPort.Curve.BoundingBox.Contains(ePort.Curve.BoundingBox)) {
                    IntersectionInfo ii = Curve.CurveCurveIntersectionOne(sPort.Curve, new LineSegment(edge.Curve.Start, edge.Curve.End), true);
                    ((Polyline)edge.Curve).StartPoint.Point = ii.IntersectionPoint;
                }
                if (ePort.Curve.BoundingBox.Contains(sPort.Curve.BoundingBox)) {
                    IntersectionInfo ii = Curve.CurveCurveIntersectionOne(ePort.Curve, new LineSegment(edge.Curve.Start, edge.Curve.End), true);
                    ((Polyline)edge.Curve).EndPoint.Point = ii.IntersectionPoint;
                }
            }
        }

        static internal Cdt CreateConstrainedDelaunayTriangulation(RectangleNode<Polyline, Point> looseHierarchy) {
            IEnumerable<Polyline> obstacles = looseHierarchy.GetAllLeaves();

            Rectangle rectangle = (Rectangle)looseHierarchy.Rectangle;
            rectangle.Pad(rectangle.Diagonal / 4);

            var additionalObstacles = new[] {
                rectangle.Perimeter() };

            return GetConstrainedDelaunayTriangulation(obstacles.Concat(additionalObstacles));
        }

        private static Cdt GetConstrainedDelaunayTriangulation(IEnumerable<Polyline> obstacles) {
            var constrainedDelaunayTriangulation = new Cdt(null, obstacles, null);
            constrainedDelaunayTriangulation.Run();
            return constrainedDelaunayTriangulation;
        }
#if TEST_MSAGL
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private
        // ReSharper disable UnusedMember.Local
        void ShowGraphLocal() {
            // ReSharper restore UnusedMember.Local
            var l = new List<ICurve>();
            l.Clear();
            foreach (var e in this.geometryGraph.Edges) {
                {
                    l.Add(new Ellipse(2, 2, e.Curve.Start));
                    l.Add(CurveFactory.CreateDiamond(5, 5, e.Curve.End));
                    l.Add(e.Curve);
                }
            }
            SplineRouter.ShowVisGraph(this.VisibilityGraph, this.LooseHierarchy.GetAllLeaves(), null, l);
        }
#endif

        private void FixLocationsForHookAnywherePorts(IEnumerable<Edge> edges) {
            foreach (var edge in edges) {
                var hookPort = edge.SourcePort as HookUpAnywhereFromInsidePort;
                if (hookPort != null) {
                    hookPort.SetLocation(this.FigureOutHookLocation(hookPort.LoosePolyline, edge.TargetPort, edge.EdgeGeometry));
                } else {
                    hookPort = edge.TargetPort as HookUpAnywhereFromInsidePort;
                    if (hookPort != null) {
                        hookPort.SetLocation(this.FigureOutHookLocation(hookPort.LoosePolyline, edge.SourcePort, edge.EdgeGeometry));
                    }
                }
            }
        }

        private Point FigureOutHookLocation(Polyline poly, Port otherEdgeEndPort, EdgeGeometry edgeGeom) {
            var clusterPort = otherEdgeEndPort as ClusterBoundaryPort;
            if (clusterPort == null) {
                return this.FigureOutHookLocationForSimpleOtherPort(poly, otherEdgeEndPort, edgeGeom);
            }
            return this.FigureOutHookLocationForClusterOtherPort(poly, clusterPort, edgeGeom);
        }

        private Point FigureOutHookLocationForClusterOtherPort(Polyline poly, ClusterBoundaryPort otherEdgeEndPort, EdgeGeometry edgeGeom) {
            var shapes = this.shortestPathRouter.MakeTransparentShapesOfEdgeGeometry(edgeGeom);
            //SplineRouter.ShowVisGraph(this.VisibilityGraph, this.LooseHierarchy.GetAllLeaves(),
            //    shapes.Select(sh => sh.BoundaryCurve), new[] { new LineSegment(edgeGeom.SourcePort.Location, edgeGeom.TargetPort.Location) });
            var s = new MultipleSourceMultipleTargetsShortestPathOnVisibilityGraph(otherEdgeEndPort.LoosePolyline.Select(p => this.VisibilityGraph.FindVertex(p)),             
                poly.Select(p => this.VisibilityGraph.FindVertex(p)), this.VisibilityGraph);
            var path = s.GetPath();
            foreach (var sh in shapes) {
                sh.IsTransparent = false;
            }

            return path.Last().Point;
        }

        private Point FigureOutHookLocationForSimpleOtherPort(Polyline poly, Port otherEdgeEndPort, EdgeGeometry edgeGeom) {
            Point otherEdgeEnd = otherEdgeEndPort.Location;
            var shapes = this.shortestPathRouter.MakeTransparentShapesOfEdgeGeometry(edgeGeom);
            //SplineRouter.ShowVisGraph(this.VisibilityGraph, this.LooseHierarchy.GetAllLeaves(),
            //    shapes.Select(sh => sh.BoundaryCurve), new[] { new LineSegment(edgeGeom.SourcePort.Location, edgeGeom.TargetPort.Location) });
            var s = new SingleSourceMultipleTargetsShortestPathOnVisibilityGraph(
                this.VisibilityGraph.FindVertex(otherEdgeEnd),
                poly.PolylinePoints.Select(p => this.VisibilityGraph.FindVertex(p.Point)), this.VisibilityGraph);
            var path = s.GetPath();
            foreach (var sh in shapes) {
                sh.IsTransparent = false;
            }

            return path.Last().Point;
        }

        private Dictionary<EdgeGeometry, Set<Polyline>> EdgeLooseEnterable { get; set; }
        private Dictionary<EdgeGeometry, Set<Polyline>> EdgeTightEnterable { get; set; }

        private bool RoutePathsWithSteinerDijkstra() {
            this.shortestPathRouter.VisibilityGraph = this.VisibilityGraph;
            this.shortestPathRouter.BundlingSettings = this.bundlingSettings;
            this.shortestPathRouter.EdgeGeometries = this.regularEdges.Select(e => e.EdgeGeometry).ToArray();
            this.shortestPathRouter.ObstacleHierarchy = this.LooseHierarchy;
            this.shortestPathRouter.RouteEdges();

            //find appropriate edge separation
            if (this.shortestPathRouter.CdtProperty != null) {
                if (!this.AnalyzeEdgeSeparation()) {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// calculates maximum possible edge separation for the computed routing
        ///   if it is greater than bundlingSettings.EdgeSeparation, then proceed 
        ///   if it is smaller, then either
        ///     stop edge bundling, or
        ///     reduce edge separation, or
        ///     move obstacles to get more free space
        /// </summary>
        private bool AnalyzeEdgeSeparation() {
            Dictionary<EdgeGeometry, Set<CdtEdge>> crossedCdtEdges = new Dictionary<EdgeGeometry, Set<CdtEdge>>();
            this.shortestPathRouter.FillCrossedCdtEdges(crossedCdtEdges);
            Dictionary<CdtEdge, Set<EdgeGeometry>> pathsOnCdtEdge = this.GetPathsOnCdtEdge(crossedCdtEdges);
            double es = this.CalculateMaxAllowedEdgeSeparation(pathsOnCdtEdge);
           // TimeMeasurer.DebugOutput("opt es: " + es);

            if (es >= this.bundlingSettings.EdgeSeparation) {
                return true; //we can even enlarge it here
            }

            if (es <= 0.02) {
                TimeMeasurer.DebugOutput("edge bundling can't be executed: not enough free space around obstacles");
                foreach (var e in this.regularEdges) {
                    e.Curve = null;
                }

                return false;
            }
            // reducing edge separation
            // TimeMeasurer.DebugOutput("reducing edge separation to " + es);
            this.bundlingSettings.EdgeSeparation = es;
            this.shortestPathRouter.RouteEdges();
            return true;
        }

        private Dictionary<CdtEdge, Set<EdgeGeometry>> GetPathsOnCdtEdge(Dictionary<EdgeGeometry, Set<CdtEdge>> crossedEdges) {
            Dictionary<CdtEdge, Set<EdgeGeometry>> res = new Dictionary<CdtEdge, Set<EdgeGeometry>>();
            foreach (var edge in crossedEdges.Keys) {
                foreach (var cdtEdge in crossedEdges[edge]) {
                    CollectionUtilities.AddToMap(res, cdtEdge, edge);
                }
            }

            return res;
        }

        private double CalculateMaxAllowedEdgeSeparation(Dictionary<CdtEdge, Set<EdgeGeometry>> pathsOnCdtEdge) {
            double l = 0.01;
            double r = 10;// ?TODO: change to bundlingSettings.EdgeSeparation;
            if (this.EdgeSeparationIsOk(pathsOnCdtEdge, r)) {
                return r;
            }

            while (Math.Abs(r - l) > 0.01) {
                double cen = (l + r) / 2;
                if (this.EdgeSeparationIsOk(pathsOnCdtEdge, cen)) {
                    l = cen;
                } else {
                    r = cen;
                }
            }
            return l;
        }

        private bool EdgeSeparationIsOk(Dictionary<CdtEdge, Set<EdgeGeometry>> pathsOnCdtEdge, double separation) {
            //total number of cdt edges
            double total = pathsOnCdtEdge.Count;
            if (total == 0) {
                return true;
            }

            //number of edges with requiredWidth <= availableWidth
            double ok = 0;
            foreach (var edge in pathsOnCdtEdge.Keys) {
                if (this.EdgeSeparationIsOk(edge, pathsOnCdtEdge[edge], separation)) {
                    ok++;
                }
            }

            //at least 95% of edges should be okay
            return (ok / total > this.bundlingSettings.MinimalRatioOfGoodCdtEdges);
        }

        private bool EdgeSeparationIsOk(CdtEdge edge, Set<EdgeGeometry> paths, double separation) {
            double requiredWidth = paths.Select(v => v.LineWidth).Sum() + (paths.Count - 1) * separation;
            double availableWidth = edge.Capacity;

            return (requiredWidth <= availableWidth);
        }

        private void RouteSelfEdges() {
            foreach (var edge in this.geometryGraph.Edges.Where(e => e.Source == e.Target)) {
                SmoothedPolyline sp;
                edge.Curve = Edge.RouteSelfEdge(edge.Source.BoundaryCurve, this.LoosePadding * 2, out sp);
            }
        }

        private void FixArrowheads() {
            foreach (var edge in this.geometryGraph.Edges) {
                Arrowheads.TrimSplineAndCalculateArrowheads(edge.EdgeGeometry,
                                                                 edge.Source.BoundaryCurve,
                                                                 edge.Target.BoundaryCurve,
                                                                 edge.Curve, false);
            }
        }
    }
}