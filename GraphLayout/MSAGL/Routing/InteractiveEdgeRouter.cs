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
using Microsoft.Msagl.Layout.Initial;
using Microsoft.Msagl.Layout.LargeGraphLayout;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;
using Microsoft.Msagl.Routing.Visibility;
#if TEST_MSAGL
using Microsoft.Msagl.DebugHelpers;
#endif

namespace Microsoft.Msagl.Routing {
    /// <summary>
    /// the router between nodes
    /// </summary>
    public class InteractiveEdgeRouter : AlgorithmBase {
       
        /// <summary>
        /// the obstacles for routing
        /// </summary>
        public IEnumerable<ICurve> Obstacles { get; private set; }

        /// <summary>
        /// the minimum angle between a node boundary curve and and an edge 
        /// curve at the place where the edge curve intersects the node boundary
        /// </summary>
        private double EnteringAngleBound { get; set; }

        private Polyline _sourceTightPolyline;

        private Polyline SourceTightPolyline {
            get { return this._sourceTightPolyline; }
            set { this._sourceTightPolyline = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        private Polyline SourceLoosePolyline { get; set; }

        private Polyline targetTightPolyline;

        /// <summary>
        /// 
        /// </summary>
        private Polyline TargetTightPolyline {
            get { return this.targetTightPolyline; }
            set { this.targetTightPolyline = value; }
        }

        private Polyline targetLoosePolyline;

        private Polyline TargetLoosePolyline {
            get { return this.targetLoosePolyline; }
            set { this.targetLoosePolyline = value; }
        }

        //RectangleNode<Polyline, Point> RootOfTightHierarchy {
        //    get { return this.obstacleCalculator.RootOfTightHierararchy; }
        //}

        private Rectangle activeRectangle = Rectangle.CreateAnEmptyBox();
        private VisibilityGraph visibilityGraph;

        internal VisibilityGraph VisibilityGraph {
            get { return this.visibilityGraph; }
            set { this.visibilityGraph = value; }
        }

        //List<Polyline> activeTightPolylines = new List<Polyline>();
        private List<Polygon> activePolygons = new List<Polygon>();
        private readonly Set<Polyline> alreadyAddedOrExcludedPolylines = new Set<Polyline>();

        //    Dictionary<Point, Polyline> pointsToObstacles = new Dicitonary<Point, Polyline>();

        private Port sourcePort;

        /// <summary>
        /// the port of the edge start
        /// </summary>
        public Port SourcePort {
            get { return this.sourcePort; }
            private set {
                this.sourcePort = value;
                if (this.sourcePort != null) {
                    this.SourceTightPolyline = GetFirstHitPolyline(this.sourcePort.Location,
                                                              this.ObstacleCalculator.RootOfTightHierarchy);
                    if (this.sourcePort is FloatingPort) {
                        this.alreadyAddedOrExcludedPolylines.Insert(this.SourceLoosePolyline);
                        //we need to exclude the loose polyline around the source port from the tangent visibily graph
                        this.StartPointOfEdgeRouting = this.SourcePort.Location;
                    }
                    else {
                        var bp = (CurvePort)this.sourcePort;
                        this.StartPointOfEdgeRouting = this.TakeBoundaryPortOutsideOfItsLoosePolyline(bp.Curve, bp.Parameter,
                                                                                            this.SourceLoosePolyline);
                    }
                }
            }
        }

        private Port targetPort;

        /// <summary>
        /// the port of the edge end
        /// </summary>
        private Port TargetPort {
            get { return this.targetPort; }
            set { this.targetPort = value; }
        }


        /// <summary>
        /// the curve should not come closer than Padding to the nodes
        /// </summary>
        public double TightPadding { get; set; }

        private double loosePadding;

        /// <summary>
        /// we further pad each node but not more than LoosePadding.
        /// </summary>
        public double LoosePadding {
            get {
                return this.loosePadding;
            }
            internal set {
                this.loosePadding = value;
                if(this.ObstacleCalculator !=null) {
                    this.ObstacleCalculator.LoosePadding = value;
                }
            }
        }

        private VisibilityVertex _sourceVisibilityVertex;

        private VisibilityVertex SourceVisibilityVertex {
            get { return this._sourceVisibilityVertex; } //            set { sourceVisibilityVertex = value; }
        }

        private VisibilityVertex targetVisibilityVertex;

        private VisibilityVertex TargetVisibilityVertex {
            get { return this.targetVisibilityVertex; } //            set { targetVisibilityVertex = value; }
        }

        private Polyline _polyline;


        /// <summary>
        /// 
        /// </summary>
        private double OffsetForPolylineRelaxing { get; set; }

        /// <summary>
        /// Set up the router and calculate the set of obstacles over which to route.
        /// </summary>
        /// <param name="obstacles">the obstacles for routing</param>
        /// <param name="padding">obstacles are inflated by this much to find an inner boundary within which edges cannot enter</param>
        /// <param name="loosePadding">
        /// obstacles are inflated again by this much to find initial 
        /// routing but then spline smoothing is allowed to come inside this outer boundary.
        /// Loose padding of 0 will give sharp corners (no spline smoothing)</param>
        /// <param name="coneSpannerAngle">if this is greater than 0 then a "cone spanner" visibility graph with be
        /// generated using cones of the specified angle to search for visibility edges.  The cone spanner graph is
        /// a sparser graph than the complete visibility graph and is hence much faster to generate and route over
        /// but may not give strictly shortest path routes</param>
        public InteractiveEdgeRouter(IEnumerable<ICurve> obstacles, double padding, double loosePadding,
                                     double coneSpannerAngle):this(obstacles, padding, loosePadding, coneSpannerAngle, false) {}

        /// <summary>
        /// The expected number of progress steps this algorithm will take.
        /// </summary>
        public int ExpectedProgressSteps { get; private set; }

        private bool targetIsInsideOfSourceTightPolyline;
        private bool sourceIsInsideOfTargetTightPolyline;
        internal bool UseEdgeLengthMultiplier;
        /// <summary>
        /// if set to true the algorithm will try to shortcut a shortest polyline inner points
        /// </summary>
        internal bool UseInnerPolylingShortcutting=true;

        /// <summary>
        /// if set to true the algorithm will try to shortcut a shortest polyline start and end
        /// </summary>
        internal bool UsePolylineEndShortcutting=true;

        internal bool AllowedShootingStraightLines = true;
  

        /// <summary>
        /// An empty constructor for calling it from inside of MSAGL
        /// </summary>        
        internal InteractiveEdgeRouter() {
            this.ObstacleCalculator = new InteractiveObstacleCalculator(this.Obstacles, this.TightPadding, this.LoosePadding, false);
        }

        private Point StartPointOfEdgeRouting { get; set; }

        private void ExtendVisibilityGraphToLocation(Point location) {
            if (this.VisibilityGraph == null) {
                this.VisibilityGraph = new VisibilityGraph();
            }

            List<Polygon> addedPolygons = null;
            if (!this.activeRectangle.Contains(location)) {
                if (this.activeRectangle.IsEmpty) {
                    this.activeRectangle = new Rectangle(this.SourcePort.Location, location);
                } else {
                    this.activeRectangle.Add(location);
                }

                addedPolygons = this.GetAddedPolygonesAndMaybeExtendActiveRectangle();
                foreach (Polygon polygon in addedPolygons) {
                    this.VisibilityGraph.AddHole(polygon.Polyline);
                }
            }
            if (addedPolygons == null || addedPolygons.Count == 0) {
                if (this.targetVisibilityVertex != null) {
                    this.VisibilityGraph.RemoveVertex(this.targetVisibilityVertex);
                }

                this.CalculateEdgeTargetVisibilityGraph(location);
            }
            else {
                this.RemovePointVisibilityGraphs();
                var visibilityGraphGenerator = new InteractiveTangentVisibilityGraphCalculator(addedPolygons,
                                                                                               this.activePolygons,
                                                                                               this.VisibilityGraph);
                visibilityGraphGenerator.Run();
                this.activePolygons.AddRange(addedPolygons);
                this.CalculateEdgeTargetVisibilityGraph(location);
                this.CalculateSourcePortVisibilityGraph();
            }
        }

        private void RemovePointVisibilityGraphs() {
            if (this.targetVisibilityVertex != null) {
                this.VisibilityGraph.RemoveVertex(this.targetVisibilityVertex);
            }

            if (this._sourceVisibilityVertex != null) {
                this.VisibilityGraph.RemoveVertex(this._sourceVisibilityVertex);
            }
        }

        private void CalculateEdgeTargetVisibilityGraph(Point location) {
            this.targetVisibilityVertex =  PointVisibilityCalculator.CalculatePointVisibilityGraph(this.GetActivePolylines(), this.VisibilityGraph, location,
                                                                    VisibilityKind.Tangent);
        }

        private void CalculateSourcePortVisibilityGraph() {
            this._sourceVisibilityVertex = PointVisibilityCalculator.CalculatePointVisibilityGraph(this.GetActivePolylines(), this.VisibilityGraph,
                                                                    this.StartPointOfEdgeRouting, VisibilityKind.Tangent);
            Debug.Assert(this._sourceVisibilityVertex != null);
        }

        private Point TakeBoundaryPortOutsideOfItsLoosePolyline(ICurve nodeBoundary, double parameter, Polyline loosePolyline) {
            Point location = nodeBoundary[parameter];
            Point tangent =
                (nodeBoundary.LeftDerivative(parameter).Normalize() +
                 nodeBoundary.RightDerivative(parameter).Normalize()).Normalize();
            if (Point.GetTriangleOrientation(PointInsideOfConvexCurve(nodeBoundary), location, location + tangent) ==
                TriangleOrientation.Counterclockwise) {
                tangent = -tangent;
            }

            tangent = tangent.Rotate(Math.PI/2);

            double len = loosePolyline.BoundingBox.Diagonal;
            var ls = new LineSegment(location, location + len*tangent);
            Point p = Curve.GetAllIntersections(ls, loosePolyline, false)[0].IntersectionPoint;

            Point del = tangent*(p - location).Length*0.5;
            //Point del = tangent * this.OffsetForPolylineRelaxing * 2;


            while (true) {
                ls = new LineSegment(location, p + del);
                bool foundIntersectionsOutsideOfSource = false;
                foreach (IntersectionInfo ii in
                    IntersectionsOfLineAndRectangleNodeOverPolyline(ls, this.ObstacleCalculator.RootOfLooseHierarchy)) {
                    if (ii.Segment1 != loosePolyline) {
                        del /= 1.5;
                        foundIntersectionsOutsideOfSource = true;
                        break;
                    }
                }

                if (!foundIntersectionsOutsideOfSource) {
                    break;
                }
            }

            return ls.End;
        }

        private static Point PointInsideOfConvexCurve(ICurve nodeBoundary) {
            return (nodeBoundary[0] + nodeBoundary[1.5])/2; //a hack !!!!!!!!!!!!!!!!!!!!!!
        }

        //Point TakeSourcePortOutsideOfLoosePolyline() {
        //    CurvePort bp = SourcePort as CurvePort;
        //    ICurve nodeBoundary = bp.Node.BoundaryCurve;
        //    Point location = bp.Location;
        //    Point tangent = (nodeBoundary.LeftDerivative(bp.Parameter).Normalize() + nodeBoundary.RightDerivative(bp.Parameter).Normalize()).Normalize();
        //    if (Point.GetTriangleOrientation(bp.Node.Center, location, location + tangent) == TriangleOrientation.Counterclockwise)
        //        tangent = -tangent;

        //    tangent = tangent.Rotate(Math.PI / 2);

        //    double len = this.sourceLoosePolyline.BoundingBox.Diagonal;
        //    Point portLocation = bp.Location;
        //    LineSegment ls = new LineSegment(portLocation, portLocation + len * tangent);
        //    Point p = Curve.GetAllIntersections(ls, this.SourceLoosePolyline, false)[0].IntersectionPoint;
        //    Point del = tangent * this.OffsetForPolylineRelaxing * 2;

        //    while (true) {
        //        ls = new LineSegment(portLocation, p + del);
        //        bool foundIntersectionsOutsideOfSource = false;
        //        foreach (IntersectionInfo ii in IntersectionsOfLineAndRectangleNodeOverPolyline(ls, this.obstacleCalculator.RootOfLooseHierarchy))
        //            if (ii.Segment1 != this.SourceLoosePolyline) {
        //                del /= 1.5;
        //                foundIntersectionsOutsideOfSource = true;
        //                break;
        //            }
        //        if (!foundIntersectionsOutsideOfSource)
        //            break;
        //    }

        //    return ls.End;
        //}

        private IEnumerable<Polyline> GetActivePolylines() {
            foreach (Polygon polygon in this.activePolygons) {
                yield return polygon.Polyline;
            }
        }

        private List<Polygon> GetAddedPolygonesAndMaybeExtendActiveRectangle() {
            Rectangle rect = this.activeRectangle;
            var addedPolygones = new List<Polygon>();
            bool added;
            do {
                added = false;
                foreach (Polyline loosePoly in
                    this.ObstacleCalculator.RootOfLooseHierarchy.GetNodeItemsIntersectingRectangle(this.activeRectangle)) {
                    if (!this.alreadyAddedOrExcludedPolylines.Contains(loosePoly)) {
                        rect.Add(loosePoly.BoundingBox);
                        addedPolygones.Add(new Polygon(loosePoly));
                        this.alreadyAddedOrExcludedPolylines.Insert(loosePoly);
                        //we register the loose polyline in the set to not add it twice
                        added = true;
                    }
                }
                if (added) {
                    this.activeRectangle = rect;
                }
            } while (added);
            return addedPolygones;
        }

        #region commented out code

        // List<Polyline> GetActivePolylines(Rectangle rectangleOfVisibilityGraph) {
        //    return this.activeTightPolylines;
        //}


        //bool LineIntersectsTightObstacles(Point point, Point x) {
        //    return LineIntersectsTightObstacles(new LineSegment(point, x));    
        //}

        #endregion

#if TEST_MSAGL
        // void ShowPolylineAndObstaclesWithGraph() {
        //    List<ICurve> ls = new List<ICurve>();
        //    foreach (Polyline poly in this.obstacleCalculator.TightObstacles)
        //        ls.Add(poly);
        //    foreach (Polyline poly in this.obstacleCalculator.LooseObstacles)
        //        ls.Add(poly);
        //    AddVisibilityGraph(ls);
        //    ls.Add(Polyline);
        //    SugiyamaLayoutSettings.Show(ls.ToArray());
        //}
#endif
        //pull the polyline out from the corners
        private void RelaxPolyline() {
            RelaxedPolylinePoint relaxedPolylinePoint = CreateRelaxedPolylinePoints(this._polyline);
            for (relaxedPolylinePoint = relaxedPolylinePoint.Next;
                 relaxedPolylinePoint.Next != null;
                 relaxedPolylinePoint = relaxedPolylinePoint.Next) {
                this.RelaxPolylinePoint(relaxedPolylinePoint);
            }
        }

        private static RelaxedPolylinePoint CreateRelaxedPolylinePoints(Polyline polyline) {
            PolylinePoint p = polyline.StartPoint;
            var ret = new RelaxedPolylinePoint(p, p.Point);
            RelaxedPolylinePoint currentRelaxed = ret;
            while (p.Next != null) {
                p = p.Next;
                var r = new RelaxedPolylinePoint(p, p.Point) {Prev = currentRelaxed};
                currentRelaxed.Next = r;
                currentRelaxed = r;
            }
            return ret;
        }

        private void RelaxPolylinePoint(RelaxedPolylinePoint relaxedPoint) {
            if (relaxedPoint.PolylinePoint.Prev.Prev == null && this.SourcePort is CurvePort &&
                relaxedPoint.PolylinePoint.Polyline != this.SourceLoosePolyline) {
                return;
            }

            if (relaxedPoint.PolylinePoint.Next.Next == null && this.TargetPort is CurvePort &&
                relaxedPoint.PolylinePoint.Polyline != this.TargetLoosePolyline) {
                return;
            }

            for (double d = this.OffsetForPolylineRelaxing;
                 d > ApproximateComparer.DistanceEpsilon && !this.RelaxWithGivenOffset(d, relaxedPoint);
                 d /= 2) {
            }
        }

        private bool RelaxWithGivenOffset(double offset, RelaxedPolylinePoint relaxedPoint) {
            Debug.Assert(offset > ApproximateComparer.DistanceEpsilon); //otherwise we are cycling infinitely here
            SetRelaxedPointLocation(offset, relaxedPoint);

            if (this.StickingSegmentDoesNotIntersectTightObstacles(relaxedPoint)) {
                return true;
            }
            PullCloserRelaxedPoint(relaxedPoint.Prev);
            return false;
        }

        private static void PullCloserRelaxedPoint(RelaxedPolylinePoint relaxedPolylinePoint) {
            relaxedPolylinePoint.PolylinePoint.Point = 0.2*relaxedPolylinePoint.OriginalPosition +
                                                       0.8*relaxedPolylinePoint.PolylinePoint.Point;
        }

        private bool StickingSegmentDoesNotIntersectTightObstacles(RelaxedPolylinePoint relaxedPoint) {
            return
                !this.PolylineSegmentIntersectsTightHierarchy(relaxedPoint.PolylinePoint.Point,
                                                         relaxedPoint.Prev.PolylinePoint.Point) &&
                (relaxedPoint.Next == null ||
                 !this.PolylineSegmentIntersectsTightHierarchy(relaxedPoint.PolylinePoint.Point,
                                                          relaxedPoint.Next.PolylinePoint.Point));
        }

        private bool PolylineSegmentIntersectsTightHierarchy(Point a, Point b) {
            return this.PolylineIntersectsPolyRectangleNodeOfTightHierarchy(a, b, this.ObstacleCalculator.RootOfTightHierarchy);
        }

        private bool PolylineIntersectsPolyRectangleNodeOfTightHierarchy(Point a, Point b, RectangleNode<Polyline, Point> rect) {
            return this.PolylineIntersectsPolyRectangleNodeOfTightHierarchy(new LineSegment(a, b), rect);
        }

        private bool PolylineIntersectsPolyRectangleNodeOfTightHierarchy(LineSegment ls, RectangleNode<Polyline, Point> rect) {
            if (!ls.BoundingBox.Intersects((Rectangle)rect.Rectangle)) {
                return false;
            }

            if (rect.UserData != null) {
                foreach (IntersectionInfo ii in Curve.GetAllIntersections(ls, rect.UserData, false)) {
                    if (ii.Segment1 != this.SourceTightPolyline && ii.Segment1 != this.TargetTightPolyline) {
                        return true;
                    }

                    if (ii.Segment1 == this.SourceTightPolyline && this.SourcePort is CurvePort) {
                        return true;
                    }

                    if (ii.Segment1 == this.TargetTightPolyline && this.TargetPort is CurvePort) {
                        return true;
                    }
                }
                return false;
            }
            return this.PolylineIntersectsPolyRectangleNodeOfTightHierarchy(ls, rect.Left) ||
                   this.PolylineIntersectsPolyRectangleNodeOfTightHierarchy(ls, rect.Right);
        }

        internal static List<IntersectionInfo> IntersectionsOfLineAndRectangleNodeOverPolyline(LineSegment ls,
                                                                                               RectangleNode<Polyline, Point>
                                                                                                   rectNode) {
            var ret = new List<IntersectionInfo>();
            IntersectionsOfLineAndRectangleNodeOverPolyline(ls, rectNode, ret);
            return ret;
        }

        private static void IntersectionsOfLineAndRectangleNodeOverPolyline(LineSegment ls, RectangleNode<Polyline, Point> rectNode,
                                                                    List<IntersectionInfo> listOfIntersections) {
            if (rectNode == null) {
                return;
            }

            if (!ls.BoundingBox.Intersects((Rectangle)rectNode.Rectangle)) {
                return;
            }

            if (rectNode.UserData != null) {
                listOfIntersections.AddRange(Curve.GetAllIntersections(ls, rectNode.UserData, true));
                return;
            }

            IntersectionsOfLineAndRectangleNodeOverPolyline(ls, rectNode.Left, listOfIntersections);
            IntersectionsOfLineAndRectangleNodeOverPolyline(ls, rectNode.Right, listOfIntersections);
        }

        private bool LineCanBeAcceptedForRouting(LineSegment ls) {
            bool sourceIsFloating = this.SourcePort is FloatingPort;
            bool targetIsFloating = this.TargetPort is FloatingPort;

            if (!sourceIsFloating && !this.targetIsInsideOfSourceTightPolyline) {
                if (!this.InsideOfTheAllowedConeOfBoundaryPort(ls.End, this.SourcePort as CurvePort)) {
                    return false;
                }
            }

            if (!targetIsFloating && this.TargetPort != null && !this.sourceIsInsideOfTargetTightPolyline) {
                if (!this.InsideOfTheAllowedConeOfBoundaryPort(ls.Start, this.TargetPort as CurvePort)) {
                    return false;
                }
            }

            List<IntersectionInfo> xx = IntersectionsOfLineAndRectangleNodeOverPolyline(ls,
                                                                                        this.ObstacleCalculator.
                                                                                            RootOfTightHierarchy);
            foreach (IntersectionInfo ii in xx) {
                if (ii.Segment1 == this.SourceTightPolyline) {
                    continue;
                }

                if (ii.Segment1 == this.targetTightPolyline) {
                    continue;
                }

                return false;
            }
            return true;
        }

        private bool InsideOfTheAllowedConeOfBoundaryPort(Point pointToTest, CurvePort port) {
            ICurve boundaryCurve = port.Curve;
            bool curveIsClockwise = InteractiveObstacleCalculator.CurveIsClockwise(boundaryCurve,
                                                                                   PointInsideOfConvexCurve(
                                                                                       boundaryCurve));
            Point portLocation = port.Location;
            Point pointOnTheRightConeSide = this.GetPointOnTheRightBoundaryPortConeSide(portLocation, boundaryCurve,
                                                                                   curveIsClockwise, port.Parameter);
            Point pointOnTheLeftConeSide = this.GetPointOnTheLeftBoundaryPortConeSide(portLocation, boundaryCurve,
                                                                                 curveIsClockwise, port.Parameter);
            return Point.GetTriangleOrientation(portLocation, pointOnTheRightConeSide, pointToTest) !=
                   TriangleOrientation.Clockwise &&
                   Point.GetTriangleOrientation(portLocation, pointToTest, pointOnTheLeftConeSide) !=
                   TriangleOrientation.Clockwise;
        }

        private Point GetPointOnTheRightBoundaryPortConeSide(Point portLocation, ICurve boundaryCurve, bool curveIsClockwise,
                                                     double portParam) {
            Point tan = curveIsClockwise
                            ? boundaryCurve.RightDerivative(portParam)
                            : -boundaryCurve.LeftDerivative(portParam);

            return portLocation + tan.Rotate(this.EnteringAngleBound);
        }

        private Point GetPointOnTheLeftBoundaryPortConeSide(Point portLocation, ICurve boundaryCurve, bool curveIsClockwise,
                                                    double portParam) {
            Point tan = curveIsClockwise
                            ? -boundaryCurve.LeftDerivative(portParam)
                            : boundaryCurve.RightDerivative(portParam);
            return portLocation + tan.Rotate(-this.EnteringAngleBound);
        }

        private static void SetRelaxedPointLocation(double offset, RelaxedPolylinePoint relaxedPoint) {
            bool leftTurn =
                Point.GetTriangleOrientation(relaxedPoint.Next.OriginalPosition, relaxedPoint.OriginalPosition,
                                             relaxedPoint.Prev.OriginalPosition) == TriangleOrientation.Counterclockwise;
            Point v =
                ((relaxedPoint.Next.OriginalPosition - relaxedPoint.Prev.OriginalPosition).Normalize()*offset).Rotate(
                    Math.PI/2);

            if (!leftTurn) {
                v = -v;
            }

            relaxedPoint.PolylinePoint.Point = relaxedPoint.OriginalPosition + v;
        }

#if TEST_MSAGL
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
// ReSharper disable UnusedMember.Local
        internal void ShowPolylineAndObstacles(params ICurve[] curves)
        {
// ReSharper restore UnusedMember.Local
            IEnumerable<DebugCurve> ls = this.GetDebugCurves(curves);
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(ls);
        }

        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private IEnumerable<DebugCurve> GetDebugCurves(params ICurve[] curves)
        {
            var ls = this.CreateListWithObstaclesAndPolyline(curves);
            //ls.AddRange(this.VisibilityGraph.Edges.Select(e => new DebugCurve(100,0.1, e is TollFreeVisibilityEdge?"red":"green", new LineSegment(e.SourcePoint, e.TargetPoint))));
            if (this._sourceVisibilityVertex != null) {
                ls.Add(new DebugCurve("red", CurveFactory.CreateDiamond(4, 4, this._sourceVisibilityVertex.Point)));
            }

            if (this.targetVisibilityVertex != null) {
                ls.Add(new DebugCurve("purple", new Ellipse(4, 4, this.targetVisibilityVertex.Point)));
            }

            var anywerePort= this.targetPort as HookUpAnywhereFromInsidePort;
            if (anywerePort != null) {
                ls.Add(new DebugCurve("purple", anywerePort.LoosePolyline));
            }

            return ls;
        }


        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private List<DebugCurve> CreateListWithObstaclesAndPolyline(params ICurve[] curves)
        {
            var ls = new List<DebugCurve>(this.ObstacleCalculator.RootOfLooseHierarchy.GetAllLeaves().Select(e => new DebugCurve(100,0.01, "green", e)));
            ls.AddRange(curves.Select(c=>new DebugCurve(100,0.01,"red", c)));
            ls.AddRange(this.ObstacleCalculator.RootOfTightHierarchy.GetAllLeaves().Select(e => new DebugCurve(100, 0.01, "blue", e)));

            // ls.AddRange(visibilityGraph.Edges.Select(e => (ICurve) new LineSegment(e.SourcePoint, e.TargetPoint)));
            if (this._polyline != null) {
                ls.Add(new DebugCurve(100, 0.03, "blue", this._polyline));
            }

            return ls;
        }
#endif

        /// <summary>
        /// smoothing the corners of the polyline
        /// </summary>
        /// <param name="edgePolyline"></param>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public void SmoothCorners(SmoothedPolyline edgePolyline) {
            ValidateArg.IsNotNull(edgePolyline, "edgePolyline");

            CornerSite a = edgePolyline.HeadSite; //the corner start
            CornerSite b; //the corner origin
            CornerSite c; //the corner other end
            while (Curve.FindCorner(a, out b, out c)) {
                a = this.SmoothOneCorner(a, c, b);
            }
        }

        private CornerSite SmoothOneCorner(CornerSite a, CornerSite c, CornerSite b) {
            const double mult = 1.5;
            const double kMin = 0.01;
            
            double k = 0.5;
            CubicBezierSegment seg;
            double u, v;
            if (a.Previous == null) {
                //this will allow to the segment to start from site "a"
                u = 2;
                v = 1;
            } else if (c.Next == null) {
                u = 1;
                v = 2; //this will allow to the segment to end at site "c"
            } else {
                u = v = 1;
            }

            do {
                seg = Curve.CreateBezierSeg(k*u, k*v, a, b, c);
                b.PreviousBezierSegmentFitCoefficient = k*u;
                b.NextBezierSegmentFitCoefficient = k*v;
                k /= mult;
            } while (this.ObstacleCalculator.ObstaclesIntersectICurve(seg) && k > kMin);

            k *= mult; //that was the last k
            if (k < 0.5 && k > kMin) {
                //one time try a smoother seg
                k = 0.5*(k + k*mult);
                seg = Curve.CreateBezierSeg(k*u, k*v, a, b, c);
                if (!this.ObstacleCalculator.ObstaclesIntersectICurve(seg)) {
                    b.PreviousBezierSegmentFitCoefficient = k*u;
                    b.NextBezierSegmentFitCoefficient = k*v;
                }
            }
            
            return b;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="underlyingPolyline"></param>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public void TryToRemoveInflectionsAndCollinearSegments(SmoothedPolyline underlyingPolyline) {
            ValidateArg.IsNotNull(underlyingPolyline, "underlyingPolyline");
            bool progress = true;
            while (progress) {
                progress = false;
                for (CornerSite s = underlyingPolyline.HeadSite; s != null && s.Next != null; s = s.Next) {
                    if (s.Turn*s.Next.Turn < 0) {
                        progress = this.TryToRemoveInflectionEdge(ref s) || progress;
                    }
                }
            }
        }

        private bool TryToRemoveInflectionEdge(ref CornerSite s) {
            if (!this.ObstacleCalculator.ObstaclesIntersectLine(s.Previous.Point, s.Next.Point)) {
                CornerSite a = s.Previous; //forget s
                CornerSite b = s.Next;
                a.Next = b;
                b.Previous = a;
                s = a;
                return true;
            }
            if (!this.ObstacleCalculator.ObstaclesIntersectLine(s.Previous.Point, s.Next.Next.Point)) {
                //forget about s and s.Next
                CornerSite a = s.Previous;
                CornerSite b = s.Next.Next;
                a.Next = b;
                b.Previous = a;
                s = a;
                return true;
            }
            if (!this.ObstacleCalculator.ObstaclesIntersectLine(s.Point, s.Next.Next.Point)) {
                //forget about s.Next
                CornerSite b = s.Next.Next;
                s.Next = b;
                b.Previous = s;
                return true;
            }

            return false;
        }

        //internal Point TargetPoint {
        //    get {
        //        CurvePort tp = this.TargetPort as CurvePort;
        //        if (tp != null)
        //            return this.Target.BoundaryCurve[tp.Parameter];
        //        else
        //            return (this.TargetPort as FloatingPort).Location;
        //    }
        //}

        //internal Point SourcePoint {
        //    get {
        //        CurvePort sp = this.SourcePort as CurvePort;
        //        if (sp != null)
        //            return this.Source.BoundaryCurve[sp.Parameter];
        //        else
        //            return (this.SourcePort as FloatingPort).Location;
        //    }
        //}


        private Polyline GetShortestPolyline(VisibilityVertex sourceVisVertex, VisibilityVertex _targetVisVertex) {
            this.CleanTheGraphForShortestPath();
            var pathCalc = new SingleSourceSingleTargetShortestPathOnVisibilityGraph(this.visibilityGraph, sourceVisVertex, _targetVisVertex);

            IEnumerable<VisibilityVertex> path = pathCalc.GetPath(this.UseEdgeLengthMultiplier);
            if (path == null) {
                //ShowIsPassable(_sourceVisibilityVertex, _targetVisVertex);
                return null;
            }


            Debug.Assert(path.First() == sourceVisVertex && path.Last() == _targetVisVertex);
            var ret = new Polyline();
            foreach (VisibilityVertex v in path) {
                ret.AddPoint(v.Point);
            }

            return RemoveCollinearVertices(ret);
        }

#if TEST_MSAGL
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowIsPassable(VisibilityVertex sourceVisVertex, VisibilityVertex targetVisVertex)
        {
            var dd = new List<DebugCurve>(
                this.visibilityGraph.Edges.Select(
                    e =>
                    new DebugCurve(100, 0.5, e.IsPassable == null || e.IsPassable() ? "green" : "red",
                                   new LineSegment(e.SourcePoint, e.TargetPoint))));
            if(sourceVisVertex!=null) {
                dd.Add(new DebugCurve(CurveFactory.CreateDiamond(3, 3, sourceVisVertex.Point)));
            }

            if (targetVisVertex!=null) {
                dd.Add(new DebugCurve(CurveFactory.CreateEllipse(3, 3, targetVisVertex.Point)));
            }

            if (this.Obstacles != null) {
                dd.AddRange(this.Obstacles.Select(o => new DebugCurve(o)));
            }

            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(dd);
        }
#endif

        private void CleanTheGraphForShortestPath() {
            this.visibilityGraph.ClearPrevEdgesTable();            
        }

        internal static Polyline RemoveCollinearVertices(Polyline ret) {
            for (PolylinePoint pp = ret.StartPoint.Next; pp.Next != null; pp = pp.Next) {
                if (Point.GetTriangleOrientation(pp.Prev.Point, pp.Point, pp.Next.Point) ==
                    TriangleOrientation.Collinear) {
                    pp.Prev.Next = pp.Next;
                    pp.Next.Prev = pp.Prev;
                }
            }
            return ret;
        }


        /// <summary>
        /// returns true if the nodes overlap or just positioned too close
        /// </summary>
        public bool OverlapsDetected {
            get { return this.ObstacleCalculator.OverlapsDetected; }
        }

        ///<summary>
        ///</summary>
        public double ConeSpannerAngle { get; set; }

        internal RectangleNode<Polyline, Point> TightHierarchy {
            get { return this.ObstacleCalculator.RootOfTightHierarchy; }
            set { this.ObstacleCalculator.RootOfTightHierarchy = value; }
        }

        internal RectangleNode<Polyline, Point> LooseHierarchy {
            get { return this.ObstacleCalculator.RootOfLooseHierarchy; }
            set { this.ObstacleCalculator.RootOfLooseHierarchy = value; }
        }

        internal bool UseSpanner { get; set; }

        private void CalculateObstacles() {
            this.ObstacleCalculator = new InteractiveObstacleCalculator(this.Obstacles, this.TightPadding, this.LoosePadding,
                                                                   this.IgnoreTightPadding);
            this.ObstacleCalculator.Calculate();
        }


        //  int count;

        /// <summary>
        ///
        /// </summary>
        /// <param name="targetLocation"></param>
        /// <returns></returns>
        public EdgeGeometry RouteEdgeToLocation(Point targetLocation) {
            this.TargetPort = new FloatingPort((ICurve) null, targetLocation); //otherwise route edge to a port would be called
            this.TargetTightPolyline = null;
            this.TargetLoosePolyline = null;
            var edgeGeometry = new EdgeGeometry();

            var ls = new LineSegment(this.SourcePort.Location, targetLocation);

            if (this.LineCanBeAcceptedForRouting(ls)) {
                this._polyline = new Polyline();
                this._polyline.AddPoint(ls.Start);
                this._polyline.AddPoint(ls.End);
                edgeGeometry.SmoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);
                edgeGeometry.Curve = edgeGeometry.SmoothedPolyline.CreateCurve();
                return edgeGeometry;
            }

            //can we do with just two line segments?
            if (this.SourcePort is CurvePort) {
                ls = new LineSegment(this.StartPointOfEdgeRouting, targetLocation);
                if (
                    IntersectionsOfLineAndRectangleNodeOverPolyline(ls, this.ObstacleCalculator.RootOfTightHierarchy).Count ==
                    0) {
                    this._polyline = new Polyline();
                    this._polyline.AddPoint(this.SourcePort.Location);
                    this._polyline.AddPoint(ls.Start);
                    this._polyline.AddPoint(ls.End);
                    //RelaxPolyline();
                    edgeGeometry.SmoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);
                    edgeGeometry.Curve = edgeGeometry.SmoothedPolyline.CreateCurve();
                    return edgeGeometry;
                }
            }

            this.ExtendVisibilityGraphToLocation(targetLocation);

            this._polyline = this.GetShortestPolyline(this.SourceVisibilityVertex, this.TargetVisibilityVertex);

            this.RelaxPolyline();
            if (this.SourcePort is CurvePort) {
                this._polyline.PrependPoint(this.SourcePort.Location);
            }

            edgeGeometry.SmoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);
            edgeGeometry.Curve = edgeGeometry.SmoothedPolyline.CreateCurve();
            return edgeGeometry;
        }

        /// <summary>
        /// routes the edge to the port
        /// </summary>
        /// <param name="edgeTargetPort"></param>
        /// <param name="portLoosePolyline"></param>
        /// <param name="smooth"> if true will smooth the edge avoiding the obstacles, will take more time</param>
        /// <param name="smoothedPolyline"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1021:AvoidOutParameters", MessageId = "3#"), SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public ICurve RouteEdgeToPort(Port edgeTargetPort, Polyline portLoosePolyline, bool smooth, out SmoothedPolyline smoothedPolyline) {
            ValidateArg.IsNotNull(edgeTargetPort, "edgeTargetToPort");
            if (!this.ObstacleCalculator.IsEmpty()) {
                this.TargetPort = edgeTargetPort;
                this.TargetTightPolyline = GetFirstHitPolyline(edgeTargetPort.Location,
                                                          this.ObstacleCalculator.RootOfTightHierarchy);
                Debug.Assert(this.targetTightPolyline != null);
                var bp = edgeTargetPort as CurvePort;
                if (bp != null) {
                    return this.RouteEdgeToBoundaryPort(portLoosePolyline, smooth, out smoothedPolyline);
                }

                return this.RouteEdgeToFloatingPortOfNode(portLoosePolyline, smooth, out smoothedPolyline);
            }
            if (this.sourcePort != null && this.targetPort != null) {
                smoothedPolyline = this.SmoothedPolylineFromTwoPoints(this.sourcePort.Location, this.targetPort.Location);
                return new LineSegment(this.sourcePort.Location, this.targetPort.Location);
            }
            smoothedPolyline = null;
            return null;
        }

        private SmoothedPolyline SmoothedPolylineFromTwoPoints(Point s, Point e) {
            this._polyline = new Polyline();
            this._polyline.AddPoint(s);
            this._polyline.AddPoint(e);
            return SmoothedPolyline.FromPoints(this._polyline);
        }

        private ICurve RouteEdgeToFloatingPortOfNode(Polyline portLoosePolyline, bool smooth, out SmoothedPolyline smoothedPolyline) {
            if (this.sourcePort is FloatingPort) {
                return this.RouteFromFloatingPortToFloatingPort(portLoosePolyline, smooth, out smoothedPolyline);
            }

            return this.RouteFromBoundaryPortToFloatingPort(portLoosePolyline, smooth, out smoothedPolyline);
        }

        private ICurve RouteFromBoundaryPortToFloatingPort(Polyline targetPortLoosePolyline, bool smooth, out SmoothedPolyline polyline) {
            Point sourcePortLocation = this.SourcePort.Location;
            Point targetPortLocation = this.targetPort.Location;
            var ls = new LineSegment(sourcePortLocation, targetPortLocation);
            if (this.LineCanBeAcceptedForRouting(ls)) {
                polyline = this.SmoothedPolylineFromTwoPoints(ls.Start, ls.End);
                return ls;
            }
            if (!this.targetIsInsideOfSourceTightPolyline) {
                //try a variant with two segments
                Point takenOutPoint = this.TakeBoundaryPortOutsideOfItsLoosePolyline(this.SourcePort.Curve,
                                                                                ((CurvePort)this.SourcePort).Parameter,
                                                                                this.SourceLoosePolyline);
                ls = new LineSegment(takenOutPoint, targetPortLocation);
                if (this.LineAvoidsTightHierarchy(ls, targetPortLoosePolyline)) {
                    polyline = this.SmoothedPolylineFromTwoPoints(ls.Start, ls.End);
                    return ls;
                }
            }
            //we need to route throw the visibility graph
            this.ExtendVisibilityGraphToLocationOfTargetFloatingPort(targetPortLoosePolyline);
            this._polyline = this.GetShortestPolyline(this.SourceVisibilityVertex, this.TargetVisibilityVertex);
            Polyline tmp = this.SourceTightPolyline;
            if (!this.targetIsInsideOfSourceTightPolyline) {
                //this is done to avoid shorcutting through the source tight polyline             
                this.SourceTightPolyline = null;
            }

            this.TryShortcutPolyline();
            this.SourceTightPolyline = tmp;
            this.RelaxPolyline();
            this._polyline.PrependPoint(sourcePortLocation);
            return this.SmoothCornersAndReturnCurve(smooth, out polyline);
        }

        private ICurve SmoothCornersAndReturnCurve(bool smooth, out SmoothedPolyline smoothedPolyline) {
            smoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);
            if (smooth) {
                this.SmoothCorners(smoothedPolyline);
            }

            return smoothedPolyline.CreateCurve();
        }

        private ICurve RouteFromFloatingPortToFloatingPort(Polyline portLoosePolyline, bool smooth, out SmoothedPolyline smoothedPolyline) {
            Point targetPortLocation = this.TargetPort.Location;

            var ls = new LineSegment(this.StartPointOfEdgeRouting, targetPortLocation);
            if (this.AllowedShootingStraightLines && this.LineAvoidsTightHierarchy(ls, this.SourceTightPolyline, this.targetTightPolyline) ) {
                smoothedPolyline = this.SmoothedPolylineFromTwoPoints(ls.Start, ls.End);
                return ls;
            }
            //we need to route through the visibility graph
            this.ExtendVisibilityGraphToLocationOfTargetFloatingPort(portLoosePolyline);
            this._polyline = this.GetShortestPolyline(this.SourceVisibilityVertex, this.TargetVisibilityVertex);
            if (this._polyline == null) {
                smoothedPolyline = null;
                return null;
            }
            if (this.UseSpanner) {
                this.TryShortcutPolyline();
            }

            this.RelaxPolyline();
            smoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);

            return this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);

        }

        private void TryShortcutPolyline() {
            if(this.UseInnerPolylingShortcutting) {
                while (this.ShortcutPolylineOneTime()) {}
            }

            if (this.UsePolylineEndShortcutting) {
                this.TryShortCutThePolylineEnds();
            }
        }

        private void TryShortCutThePolylineEnds() {
            this.TryShortcutPolylineStart();
            this.TryShortcutPolylineEnd();
        }

        private void TryShortcutPolylineEnd() {
            PolylinePoint a = this._polyline.EndPoint;
            PolylinePoint b = a.Prev;
            if (b == null) {
                return;
            }

            PolylinePoint c = b.Prev;
            if (c == null) {
                return;
            }

            Point m = 0.5*(b.Point + c.Point);
            if (this.LineAvoidsTightHierarchy(a.Point, m, this._sourceTightPolyline, this.targetTightPolyline)) {
                var p = new PolylinePoint(m) {Next = a, Prev = c};
                a.Prev = p;
                c.Next = p;
            }
        }

        private void TryShortcutPolylineStart() {
            PolylinePoint a = this._polyline.StartPoint;
            PolylinePoint b = a.Next;
            if (b == null) {
                return;
            }

            PolylinePoint c = b.Next;
            if (c == null) {
                return;
            }

            Point m = 0.5*(b.Point + c.Point);
            if (this.LineAvoidsTightHierarchy(a.Point, m, this._sourceTightPolyline, this.targetTightPolyline)) {
                var p = new PolylinePoint(m) {Prev = a, Next = c};
                a.Next = p;
                c.Prev = p;
            }
        }

        private bool ShortcutPolylineOneTime() {
            bool ret = false;
            for (PolylinePoint pp = this._polyline.StartPoint; pp.Next != null && pp.Next.Next != null; pp = pp.Next) {
                ret |= this.TryShortcutPolyPoint(pp);
            }

            return ret;
        }

        private bool TryShortcutPolyPoint(PolylinePoint pp) {
            if (this.LineAvoidsTightHierarchy(new LineSegment(pp.Point, pp.Next.Next.Point), this.SourceTightPolyline,
                                         this.targetTightPolyline)) {
                //remove pp.Next
                pp.Next = pp.Next.Next;
                pp.Next.Prev = pp;
                return true;
            }
            return false;
        }

        private void ExtendVisibilityGraphToLocationOfTargetFloatingPort(Polyline portLoosePolyline) {
            if (this.VisibilityGraph == null) {
                this.VisibilityGraph = new VisibilityGraph();
            }

            List<Polygon> addedPolygons = null;
            Point targetLocation = this.targetPort.Location;
            if (!this.activeRectangle.Contains(targetLocation)) {
                if (this.activeRectangle.IsEmpty) {
                    this.activeRectangle = new Rectangle(this.SourcePort.Location, targetLocation);
                } else {
                    this.activeRectangle.Add(targetLocation);
                }

                addedPolygons = this.GetAddedPolygonesAndMaybeExtendActiveRectangle();
                foreach (Polygon polygon in addedPolygons) {
                    this.VisibilityGraph.AddHole(polygon.Polyline);
                }
            }

            if (addedPolygons == null) {
                if (this.targetVisibilityVertex != null) {
                    this.VisibilityGraph.RemoveVertex(this.targetVisibilityVertex);
                }

                this.CalculateEdgeTargetVisibilityGraphForFloatingPort(targetLocation, portLoosePolyline);
                if (this.SourceVisibilityVertex == null) {
                    this.CalculateSourcePortVisibilityGraph();
                }
            }
            else {
                this.RemovePointVisibilityGraphs();
                var visibilityGraphGenerator = new InteractiveTangentVisibilityGraphCalculator(addedPolygons,
                                                                                               this.activePolygons,
                                                                                               this.VisibilityGraph);
                visibilityGraphGenerator.Run();
                this.activePolygons.AddRange(addedPolygons);
                this.CalculateEdgeTargetVisibilityGraphForFloatingPort(targetLocation, portLoosePolyline);
                this.CalculateSourcePortVisibilityGraph();
            }
        }

        private void CalculateEdgeTargetVisibilityGraphForFloatingPort(Point targetLocation, Polyline targetLoosePoly) {
            if (this.UseSpanner) {
                this.targetVisibilityVertex = this.AddTransientVisibilityEdgesForPort(targetLocation, targetLoosePoly);
            } else {
                this.targetVisibilityVertex = PointVisibilityCalculator.CalculatePointVisibilityGraph(this.GetActivePolylinesWithException(targetLoosePoly), this.VisibilityGraph, targetLocation, VisibilityKind.Tangent);
            }
        }

        private VisibilityVertex AddTransientVisibilityEdgesForPort(Point point, IEnumerable<Point> loosePoly) {
            VisibilityVertex v = this.GetVertex(point);

            if (v != null) {
                return v;
            }

            v = this.visibilityGraph.AddVertex(point);
            if (loosePoly != null) //if the edges have not been calculated do it in a quick and dirty mode
{
                foreach (Point p in loosePoly) {
                    this.visibilityGraph.AddEdge(point, p, ((a, b) => new TollFreeVisibilityEdge(a, b)));
                }
            } else {
                v = PointVisibilityCalculator.CalculatePointVisibilityGraph(this.GetActivePolylines(),
                                                                        this.VisibilityGraph, point,
                                                                        VisibilityKind.Tangent
                                                                       );
                Debug.Assert(v != null);
            }
            return v;
        }

        private VisibilityVertex GetVertex(Point point) {
            VisibilityVertex v = this.visibilityGraph.FindVertex(point);
            if (v == null && this.LookForRoundedVertices) {
                v = this.visibilityGraph.FindVertex(ApproximateComparer.Round(point));
            }

            return v;
        }

        internal bool LookForRoundedVertices { get; set; }

        internal InteractiveObstacleCalculator ObstacleCalculator { get; set; }

        ///<summary>
        ///</summary>
        ///<param name="obstacles"></param>
        ///<param name="padding"></param>
        ///<param name="loosePadding"></param>
        ///<param name="coneSpannerAngle"></param>
        ///<param name="ignoreTightPadding"></param>
        public InteractiveEdgeRouter(IEnumerable<ICurve> obstacles, double padding, double loosePadding, double coneSpannerAngle, bool ignoreTightPadding) {
            this.IgnoreTightPadding = ignoreTightPadding;
            this.EnteringAngleBound = 80 * Math.PI / 180;
            this.TightPadding = padding;
            this.LoosePadding = loosePadding;
            this.OffsetForPolylineRelaxing = 0.75 * padding;
            if (coneSpannerAngle > 0) {
                Debug.Assert(coneSpannerAngle > Math.PI / 180);
                Debug.Assert(coneSpannerAngle <= 90 * Math.PI / 180);
                this.UseSpanner = true;
                this.ExpectedProgressSteps = ConeSpanner.GetTotalSteps(coneSpannerAngle);
            } else {
                this.ExpectedProgressSteps = obstacles.Count();
            }
            this.ConeSpannerAngle = coneSpannerAngle;
            this.Obstacles = obstacles;
            this.CalculateObstacles();
        }

        internal bool IgnoreTightPadding {get;set;}

        private IEnumerable<Polyline> GetActivePolylinesWithException(Polyline targetLoosePoly) {
            return from polygon in this.activePolygons where polygon.Polyline != targetLoosePoly select polygon.Polyline;
        }

        private ICurve RouteEdgeToBoundaryPort(Polyline portLoosePolyline, bool smooth, out SmoothedPolyline smoothedPolyline) {
            this.TargetLoosePolyline = portLoosePolyline;
            if (this.sourcePort is FloatingPort) {
                return this.RouteFromFloatingPortToBoundaryPort(smooth, out smoothedPolyline);
            }

            return this.RouteFromBoundaryPortToBoundaryPort(smooth, out smoothedPolyline);
        }

        private ICurve RouteFromBoundaryPortToBoundaryPort(bool smooth, out SmoothedPolyline smoothedPolyline) {
            Point sourcePortLocation = this.SourcePort.Location;
            ICurve curve;
            Point targetPortLocation = this.targetPort.Location;
            var ls = new LineSegment(sourcePortLocation, targetPortLocation);
            if (this.LineCanBeAcceptedForRouting(ls)) {
                this._polyline = new Polyline();
                this._polyline.AddPoint(ls.Start);
                this._polyline.AddPoint(ls.End);
                smoothedPolyline = this.SmoothedPolylineFromTwoPoints(ls.Start,ls.End);
                curve = SmoothedPolyline.FromPoints(this._polyline).CreateCurve();
            } else {
                //try three variants with two segments
                Point takenOutPoint = this.TakeBoundaryPortOutsideOfItsLoosePolyline(this.targetPort.Curve,
                                                                                ((CurvePort)this.targetPort).Parameter,
                                                                                this.TargetLoosePolyline);
                ls = new LineSegment(sourcePortLocation, takenOutPoint);
                if (this.InsideOfTheAllowedConeOfBoundaryPort(takenOutPoint, this.SourcePort as CurvePort) &&
                    this.LineAvoidsTightHierarchy(ls, this._sourceTightPolyline)) {
                    this._polyline = new Polyline();
                    this._polyline.AddPoint(ls.Start);
                    this._polyline.AddPoint(ls.End);
                    this._polyline.AddPoint(targetPortLocation);
                    curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                } else {
                    ls = new LineSegment(this.StartPointOfEdgeRouting, targetPortLocation);
                    if (this.InsideOfTheAllowedConeOfBoundaryPort(this.StartPointOfEdgeRouting, this.TargetPort as CurvePort) &&
                        this.LineAvoidsTightHierarchy(ls)) {
                        this._polyline = new Polyline();
                        this._polyline.AddPoint(sourcePortLocation);
                        this._polyline.AddPoint(ls.Start);
                        this._polyline.AddPoint(ls.End);
                        curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                    } else {
                        // we still can make the polyline with two segs when the port sticking segs are intersecting
                        Point x;
                        if (LineSegment.Intersect(sourcePortLocation, this.StartPointOfEdgeRouting, targetPortLocation,
                                                  takenOutPoint, out x)) {
                            this._polyline = new Polyline();
                            this._polyline.AddPoint(sourcePortLocation);
                            this._polyline.AddPoint(x);
                            this._polyline.AddPoint(targetPortLocation);
                            curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                        } else if (ApproximateComparer.Close(this.StartPointOfEdgeRouting, takenOutPoint)) {
                            this._polyline = new Polyline();
                            this._polyline.AddPoint(sourcePortLocation);
                            this._polyline.AddPoint(takenOutPoint);
                            this._polyline.AddPoint(targetPortLocation);
                            curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                        } else if (this.LineAvoidsTightHierarchy(new LineSegment(this.StartPointOfEdgeRouting, takenOutPoint))) {
                            //can we do three segments?
                            this._polyline = new Polyline();
                            this._polyline.AddPoint(sourcePortLocation);
                            this._polyline.AddPoint(this.StartPointOfEdgeRouting);
                            this._polyline.AddPoint(takenOutPoint);
                            this._polyline.AddPoint(targetPortLocation);
                            curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                        } else {
                            this.ExtendVisibilityGraphToTargetBoundaryPort(takenOutPoint);
                            this._polyline = this.GetShortestPolyline(this.SourceVisibilityVertex, this.TargetVisibilityVertex);
                            
                            Polyline tmpTargetTight;
                            Polyline tmpSourceTight = this.HideSourceTargetTightsIfNeeded(out tmpTargetTight);
                            this.TryShortcutPolyline();
                            this.RecoverSourceTargetTights(tmpSourceTight, tmpTargetTight);

                            this.RelaxPolyline();

                            this._polyline.PrependPoint(sourcePortLocation);
                            this._polyline.AddPoint(targetPortLocation);
                            curve = this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
                        }
                    }
                }
            }
            return curve;
        }

        private void RecoverSourceTargetTights(Polyline tmpSourceTight, Polyline tmpTargetTight) {
            this.SourceTightPolyline = tmpSourceTight;
            this.TargetTightPolyline = tmpTargetTight;
        }

        private Polyline HideSourceTargetTightsIfNeeded(out Polyline tmpTargetTight) {
            Polyline tmpSourceTight = this.SourceTightPolyline;
            tmpTargetTight = this.TargetTightPolyline;
            this.SourceTightPolyline = this.TargetTightPolyline = null;
            return tmpSourceTight;
        }

        private bool LineAvoidsTightHierarchy(LineSegment lineSegment) {
            return
                IntersectionsOfLineAndRectangleNodeOverPolyline(lineSegment, this.ObstacleCalculator.RootOfTightHierarchy).
                    Count == 0;
        }

        private ICurve RouteFromFloatingPortToBoundaryPort(bool smooth, out SmoothedPolyline smoothedPolyline) {
            Point targetPortLocation = this.targetPort.Location;
            LineSegment ls;
            if (this.InsideOfTheAllowedConeOfBoundaryPort(this.sourcePort.Location, (CurvePort)this.targetPort)) {
                ls = new LineSegment(this.SourcePort.Location, targetPortLocation);
                if (this.LineCanBeAcceptedForRouting(ls)) {
                    smoothedPolyline = this.SmoothedPolylineFromTwoPoints(ls.Start, ls.End);
                    return ls;
                }
            }
            Point takenOutTargetPortLocation = this.TakeBoundaryPortOutsideOfItsLoosePolyline(this.TargetPort.Curve,
                                                                                         ((CurvePort)this.TargetPort).
                                                                                             Parameter,
                                                                                         this.TargetLoosePolyline);
            //can we do with just two line segments?
            ls = new LineSegment(this.SourcePort.Location, takenOutTargetPortLocation);
            if (this.LineAvoidsTightHierarchy(ls, this._sourceTightPolyline)) {
                this._polyline =new Polyline(ls.Start, ls.End, targetPortLocation);
                smoothedPolyline = SmoothedPolyline.FromPoints(this._polyline);
                return smoothedPolyline.CreateCurve();
            }
            this.ExtendVisibilityGraphToTargetBoundaryPort(takenOutTargetPortLocation);

            this._polyline = this.GetShortestPolyline(this.SourceVisibilityVertex, this.TargetVisibilityVertex);

            this.RelaxPolyline();
            this._polyline.AddPoint(targetPortLocation);
            return this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
        }

        private bool LineAvoidsTightHierarchy(LineSegment ls, Polyline polylineToExclude) {
            bool lineIsGood = true;
            foreach (IntersectionInfo ii in
                IntersectionsOfLineAndRectangleNodeOverPolyline(ls, this.ObstacleCalculator.RootOfTightHierarchy)) {
                if (ii.Segment1 != polylineToExclude) {
                    lineIsGood = false;
                    break;
                }
            }

            return lineIsGood;
        }

        private bool LineAvoidsTightHierarchy(LineSegment ls, Polyline polylineToExclude0, Polyline polylineToExclude1) {
            bool lineIsGood = true;
            foreach (IntersectionInfo ii in
                IntersectionsOfLineAndRectangleNodeOverPolyline(ls, this.ObstacleCalculator.RootOfTightHierarchy)) {
                if (!(ii.Segment1 == polylineToExclude0 || ii.Segment1 == polylineToExclude1)) {
                    lineIsGood = false;
                    break;
                }
            }

            return lineIsGood;
        }

        private bool LineAvoidsTightHierarchy(Point a, Point b, Polyline polylineToExclude0, Polyline polylineToExclude1) {
            return this.LineAvoidsTightHierarchy(new LineSegment(a, b), polylineToExclude0, polylineToExclude1);
        }

        private void ExtendVisibilityGraphToTargetBoundaryPort(Point takenOutTargetPortLocation) {
            List<Polygon> addedPolygons = null;
            if (this.VisibilityGraph == null) {
                this.VisibilityGraph = new VisibilityGraph();
            }

            if (!this.activeRectangle.Contains(takenOutTargetPortLocation) ||
                !this.activeRectangle.Contains(this.TargetLoosePolyline.BoundingBox)) {
                if (this.activeRectangle.IsEmpty) {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=369 there are no structs in js
                    activeRectangle = TargetLoosePolyline.BoundingBox.Clone();
#else
                    this.activeRectangle = this.TargetLoosePolyline.BoundingBox;
#endif
                    this.activeRectangle.Add(this.SourcePort.Location);
                    this.activeRectangle.Add(this.StartPointOfEdgeRouting);
                    this.activeRectangle.Add(takenOutTargetPortLocation);
                } else {
                    this.activeRectangle.Add(takenOutTargetPortLocation);
                    this.activeRectangle.Add(this.TargetLoosePolyline.BoundingBox);
                }
                addedPolygons = this.GetAddedPolygonesAndMaybeExtendActiveRectangle();
                foreach (Polygon polygon in addedPolygons) {
                    this.VisibilityGraph.AddHole(polygon.Polyline);
                }
            }

            if (addedPolygons == null) {
                if (this.targetVisibilityVertex != null) {
                    this.VisibilityGraph.RemoveVertex(this.targetVisibilityVertex);
                }

                this.CalculateEdgeTargetVisibilityGraph(takenOutTargetPortLocation);
            } else {
                this.RemovePointVisibilityGraphs();
                var visibilityGraphGenerator = new InteractiveTangentVisibilityGraphCalculator(addedPolygons,
                                                                                               this.activePolygons,
                                                                                               this.VisibilityGraph);
                visibilityGraphGenerator.Run();
                this.activePolygons.AddRange(addedPolygons);
                this.CalculateEdgeTargetVisibilityGraph(takenOutTargetPortLocation);
                this.CalculateSourcePortVisibilityGraph();
            }
        }

        /// <summary>
        /// returns the hit object
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public Polyline GetHitLoosePolyline(Point point) {
            if (this.ObstacleCalculator.IsEmpty() || this.ObstacleCalculator.RootOfLooseHierarchy == null) {
                return null;
            }

            return GetFirstHitPolyline(point, this.ObstacleCalculator.RootOfLooseHierarchy);
        }

        internal static Polyline GetFirstHitPolyline(Point point, RectangleNode<Polyline, Point> rectangleNode) {
            RectangleNode<Polyline, Point> rectNode = GetFirstHitRectangleNode(point, rectangleNode);
            return rectNode != null ? rectNode.UserData : null;
        }

        private static RectangleNode<Polyline, Point> GetFirstHitRectangleNode(Point point, RectangleNode<Polyline, Point> rectangleNode) {
            if (rectangleNode == null) {
                return null;
            }

            return rectangleNode.FirstHitNode(point,
                                              (pnt, polyline) =>
                                              Curve.PointRelativeToCurveLocation(pnt, polyline) != PointLocation.Outside
                                                  ? HitTestBehavior.Stop
                                                  : HitTestBehavior.Continue);
        }

        /// <summary>
        /// 
        /// </summary>
        public void Clean() {
            this.SourcePort = this.TargetPort = null;
            this.SourceLoosePolyline = this.SourceTightPolyline = null;
            this.targetTightPolyline = this.TargetLoosePolyline = null;

            this.VisibilityGraph = null;
            this._sourceVisibilityVertex = this.targetVisibilityVertex = null;
            this.activePolygons.Clear();
            this.alreadyAddedOrExcludedPolylines.Clear();
            this.activeRectangle.SetToEmpty();
        }

        /// <summary>
        /// setting source port and the loose polyline of the port
        /// </summary>
        /// <param name="port"></param>
        /// <param name="sourceLoosePolylinePar"></param>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "polyline"),
         SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public void SetSourcePortAndSourceLoosePolyline(Port port, Polyline sourceLoosePolylinePar) {
            this.SourceLoosePolyline = sourceLoosePolylinePar;
            this.sourcePort = port;
            if (this.sourcePort != null) {
                this.SourceTightPolyline = GetFirstHitPolyline(this.sourcePort.Location, this.ObstacleCalculator.RootOfTightHierarchy);
                if (this.sourcePort is FloatingPort) {
                    this.alreadyAddedOrExcludedPolylines.Insert(this.SourceLoosePolyline);
                    //we need to exclude the loose polyline around the source port from the tangent visibily graph
                    this.StartPointOfEdgeRouting = this.SourcePort.Location;
                }
                else {
                    this.StartPointOfEdgeRouting = this.TakeBoundaryPortOutsideOfItsLoosePolyline(this.SourcePort.Curve,
                                                                                        ((CurvePort)this.sourcePort).
                                                                                            Parameter,
                                                                                        this.SourceLoosePolyline);
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        protected override void RunInternal() {
            this.CalculateWholeTangentVisibilityGraph();
        }

        internal void CalculateWholeTangentVisibilityGraph() {
            this.VisibilityGraph = new VisibilityGraph();
            this.CalculateWholeVisibilityGraphOnExistingGraph();
        }

        internal void CalculateWholeVisibilityGraphOnExistingGraph() {
            this.activePolygons = new List<Polygon>(this.AllPolygons());
            foreach (Polyline polylineLocal in this.ObstacleCalculator.LooseObstacles) {
                this.VisibilityGraph.AddHole(polylineLocal);
            }

            AlgorithmBase visibilityGraphGenerator;
            if (this.UseSpanner) {
                visibilityGraphGenerator = new ConeSpanner(this.ObstacleCalculator.LooseObstacles, this.VisibilityGraph)
                {ConeAngle = this.ConeSpannerAngle };
            }
            else {
                visibilityGraphGenerator = new InteractiveTangentVisibilityGraphCalculator(new List<Polygon>(),
                                                                                           this.activePolygons,
                                                                                           this.visibilityGraph);
            }
            visibilityGraphGenerator.Run();
        }

        ///<summary>
        ///</summary>
        ///<param name="sourcePortLocal"></param>
        ///<param name="targetPortLocal"></param>
        ///<param name="smooth"></param>
        ///<param name="smoothedPolyline"></param>
        ///<returns></returns>
        [SuppressMessage("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        public ICurve RouteSplineFromPortToPortWhenTheWholeGraphIsReady(Port sourcePortLocal,
                                                                          Port targetPortLocal, bool smooth, out SmoothedPolyline smoothedPolyline) {
            bool reversed = (sourcePortLocal is FloatingPort && targetPortLocal is CurvePort) 
                || sourcePortLocal is HookUpAnywhereFromInsidePort;
            if (reversed) {
                Port tmp = sourcePortLocal;
                sourcePortLocal = targetPortLocal;
                targetPortLocal = tmp;
            }
            this.sourcePort = sourcePortLocal;
            this.targetPort = targetPortLocal;

            this.FigureOutSourceTargetPolylinesAndActiveRectangle();
            ICurve curve = this.GetEdgeGeomByRouting(smooth, out smoothedPolyline);
            if (curve == null) {
                return null;
            }

            this._sourceVisibilityVertex = this.targetVisibilityVertex = null;
            if (reversed) {
                curve = curve.Reverse();
            }

            return curve;
        }

        private ICurve GetEdgeGeomByRouting(bool smooth, out SmoothedPolyline smoothedPolyline) {
            this.targetIsInsideOfSourceTightPolyline = this.SourceTightPolyline == null ||
                                                  Curve.PointRelativeToCurveLocation(this.targetPort.Location,
                                                                                     this.SourceTightPolyline) ==
                                                  PointLocation.Inside;
            this.sourceIsInsideOfTargetTightPolyline = this.TargetTightPolyline == null ||
                                                  Curve.PointRelativeToCurveLocation(this.sourcePort.Location,
                                                                                     this.TargetTightPolyline) ==
                                                  PointLocation.Inside;
            var curvePort = this.sourcePort as CurvePort;
            ICurve curve;
            if (curvePort != null) {
                this.StartPointOfEdgeRouting = !this.targetIsInsideOfSourceTightPolyline
                                              ? this.TakeBoundaryPortOutsideOfItsLoosePolyline(curvePort.Curve,
                                                                                          curvePort.Parameter,
                                                                                          this.SourceLoosePolyline)
                                              : curvePort.Location;
                this.CalculateSourcePortVisibilityGraph();
                if (this.targetPort is CurvePort) {
                    curve = this.RouteFromBoundaryPortToBoundaryPort(smooth, out smoothedPolyline);
                } else {
                    curve = this.RouteFromBoundaryPortToFloatingPort(this.targetLoosePolyline, smooth, out smoothedPolyline);
                }
            }
            else {
                if (this.targetPort is FloatingPort) {
                    this.ExtendVisibilityGraphFromFloatingSourcePort();
                    Debug.Assert(this._sourceVisibilityVertex != null);
                    //the edge has to be reversed to route from CurvePort to FloatingPort
                    curve = this.RouteFromFloatingPortToFloatingPort(this.targetLoosePolyline, smooth, out smoothedPolyline);
                } else {
                    curve = this.RouteFromFloatingPortToAnywherePort(((HookUpAnywhereFromInsidePort)this.targetPort).LoosePolyline,
                                                                smooth, out smoothedPolyline,
                                                                (HookUpAnywhereFromInsidePort)this.targetPort);
                }
            }
            return curve;
        }

        private ICurve RouteFromFloatingPortToAnywherePort(Polyline targetLoosePoly, bool smooth, out SmoothedPolyline smoothedPolyline, HookUpAnywhereFromInsidePort port) {
            if (!port.Curve.BoundingBox.Contains(this.sourcePort.Location)) {
                smoothedPolyline = null;
                return null;
            }

            this._sourceVisibilityVertex = this.GetVertex(this.sourcePort.Location);

            this._polyline = this.GetShortestPolylineToMulitpleTargets(this.SourceVisibilityVertex, this.Targets(targetLoosePoly));
            if (this._polyline == null) {
                smoothedPolyline = null;
                return null;
            }
            if (this.UseSpanner) {
                this.TryShortcutPolyline();
            }

            this.RelaxPolyline();
            this.FixLastPolylinePointForAnywherePort(port);
            if (port.HookSize > 0) {
                this.BuildHook(port);
            }

            return this.SmoothCornersAndReturnCurve(smooth, out smoothedPolyline);
        }

        private void BuildHook(HookUpAnywhereFromInsidePort port) {
            var curve = port.Curve;
            //creating a hook
            var ellipse = new Ellipse(port.HookSize, port.HookSize, this._polyline.End);
            var intersections = Curve.GetAllIntersections(curve, ellipse, true).ToArray();
            Debug.Assert(intersections.Length == 2);
            if (Point.GetTriangleOrientation(intersections[0].IntersectionPoint, this._polyline.End, this._polyline.EndPoint.Prev.Point) == TriangleOrientation.Counterclockwise) {
                intersections.Reverse(); //so the [0] point is to the left of the Polyline
            }

            var polylineTangent = (this._polyline.End - this._polyline.EndPoint.Prev.Point).Normalize();

            var tan0 = curve.Derivative(intersections[0].Par0).Normalize();
            var prj0 = tan0 * polylineTangent;
            if (Math.Abs(prj0) < 0.2) {
                this.ExtendPolyline(tan0, intersections[0], polylineTangent, port);
            } else {
                var tan1 = curve.Derivative(intersections[1].Par0).Normalize();
                var prj1 = tan1 * polylineTangent;
                if (prj1 < prj0) {
                    this.ExtendPolyline(tan1, intersections[1], polylineTangent, port);
                } else {
                    this.ExtendPolyline(tan0, intersections[0], polylineTangent, port);
                }
            }
        }

        private void ExtendPolyline(Point tangentAtIntersection, IntersectionInfo x, Point polylineTangent, HookUpAnywhereFromInsidePort port) {

            var normal=tangentAtIntersection.Rotate(Math.PI/2);
            if(normal*polylineTangent<0) {
                normal =-normal;
            }

            var pointBeforeLast = x.IntersectionPoint + normal * port.HookSize;
            Point pointAfterX;
            if (!Point.LineLineIntersection(pointBeforeLast, pointBeforeLast+tangentAtIntersection, this._polyline.End, this._polyline.End+polylineTangent, out pointAfterX)) {
                return;
            }

            this._polyline.AddPoint(pointAfterX);
            this._polyline.AddPoint(pointBeforeLast);
            this._polyline.AddPoint(x.IntersectionPoint);
        }

        private void FixLastPolylinePointForAnywherePort(HookUpAnywhereFromInsidePort port) {
            while (true) {
                PolylinePoint lastPointInside = this.GetLastPointInsideOfCurveOnPolyline(port.Curve);
                lastPointInside.Next.Next = null;
                this._polyline.EndPoint=lastPointInside.Next;
                var dir = lastPointInside.Next.Point - lastPointInside.Point;
                dir = dir.Normalize()*port.Curve.BoundingBox.Diagonal; //make it a long vector
                var dir0 = dir.Rotate(-port.AdjustmentAngle);
                var dir1 = dir.Rotate(port.AdjustmentAngle);
                var rx=Curve.CurveCurveIntersectionOne(port.Curve, new LineSegment(lastPointInside.Point, lastPointInside.Point+dir0), true);
                var lx=Curve.CurveCurveIntersectionOne(port.Curve, new LineSegment(lastPointInside.Point, lastPointInside.Point+dir1), true);
                if (rx == null || lx == null) {
                    return;
                }
                //this.ShowPolylineAndObstacles(Polyline, new LineSegment(lastPointInside.Point, lastPointInside.Point+dir0), new LineSegment(lastPointInside.Point, rerPoint+dir1), port.Curve);

                var trimmedCurve = GetTrimmedCurveForHookingUpAnywhere(port.Curve, lastPointInside, rx, lx);
                var newLastPoint = trimmedCurve[trimmedCurve.ClosestParameter(lastPointInside.Point)];
                if (!this.LineAvoidsTightHierarchy(new LineSegment(lastPointInside.Point, newLastPoint), this.SourceTightPolyline, null)) {
                    var xx=Curve.CurveCurveIntersectionOne(port.Curve, new LineSegment(lastPointInside.Point, lastPointInside.Next.Point), false);
                    if (xx == null) {
                        return;
                    }
                    //this.ShowPolylineAndObstacles(Polyline, port.Curve);
                    this._polyline.EndPoint.Point = xx.IntersectionPoint;
                    break;
                }

                this._polyline.EndPoint.Point = newLastPoint;
                if (lastPointInside.Prev == null  || !this.TryShortcutPolyPoint(lastPointInside.Prev)) {
                    break;
                }
            }
        }

        private static ICurve GetTrimmedCurveForHookingUpAnywhere(ICurve curve, PolylinePoint lastPointInside, IntersectionInfo x0, IntersectionInfo x1) {
            var clockwise =
                Point.GetTriangleOrientation(x1.IntersectionPoint, x0.IntersectionPoint, lastPointInside.Point) ==
                TriangleOrientation.Clockwise;

            double rightX = x0.Par0;
            double leftX = x1.Par0;
            ICurve tr0, tr1;
            Curve ret;
            if (clockwise) {
                if (rightX < leftX) {
                    return curve.Trim(rightX, leftX);
                }

                tr0 = curve.Trim(rightX, curve.ParEnd);
                tr1 = curve.Trim(curve.ParStart, leftX);
                ret = new Curve();
                return ret.AddSegs(tr0, tr1);
            }

            if (leftX < rightX) {
                return curve.Trim(leftX, rightX);
            }

            tr0 = curve.Trim(leftX, curve.ParEnd);
            tr1 = curve.Trim(curve.ParStart, rightX);
            ret = new Curve();
            return ret.AddSegs(tr0, tr1);
        }

        private PolylinePoint GetLastPointInsideOfCurveOnPolyline(ICurve curve) {
            for (var p = this._polyline.EndPoint.Prev; p != null; p = p.Prev) {
                if (p.Prev == null) {
                    return p;
                }

                if (Curve.PointRelativeToCurveLocation(p.Point, curve) == PointLocation.Inside) {
                    return p;
                }
            }

            throw new InvalidOperationException();

        }

        private Polyline GetShortestPolylineToMulitpleTargets(VisibilityVertex sourceVisVertex, IEnumerable<VisibilityVertex> targets) {
            this.CleanTheGraphForShortestPath();
            //ShowPolylineAndObstacles(targets.Select(t=>new Ellipse(3,3,t.Point)).ToArray());
            var pathCalc = new SingleSourceMultipleTargetsShortestPathOnVisibilityGraph(sourceVisVertex, targets, this.VisibilityGraph);// { dd = ShowPolylineAndObstacles };
            IEnumerable<VisibilityVertex> path = pathCalc.GetPath();
            if (path == null) {
                return null;
            }

            Debug.Assert(path.First() == sourceVisVertex && targets.Contains(path.Last()));
            var ret = new Polyline();
            foreach (VisibilityVertex v in path) {
                ret.AddPoint(v.Point);
            }

            return RemoveCollinearVertices(ret);
            
        }

        private IEnumerable<VisibilityVertex> Targets(Polyline targetLoosePoly) {
            return new List<VisibilityVertex> (targetLoosePoly.Select(p=> this.visibilityGraph.FindVertex(p))); 
        }

        private void ExtendVisibilityGraphFromFloatingSourcePort() {
            var fp = this.sourcePort as FloatingPort;
            Debug.Assert(fp != null);
            this.StartPointOfEdgeRouting = fp.Location;
            if (this.UseSpanner) {
                this._sourceVisibilityVertex = this.AddTransientVisibilityEdgesForPort(this.sourcePort.Location, this.SourceLoosePolyline);
            } else {
                this._sourceVisibilityVertex = PointVisibilityCalculator.CalculatePointVisibilityGraph(
                    from p in this.GetActivePolylines() where p != this.SourceLoosePolyline select p, this.VisibilityGraph,
                    this.StartPointOfEdgeRouting, VisibilityKind.Tangent);
            }
        }

        private void FigureOutSourceTargetPolylinesAndActiveRectangle() {
            this._sourceTightPolyline = GetFirstHitPolyline(this.sourcePort.Location, this.ObstacleCalculator.RootOfTightHierarchy);
            this.SourceLoosePolyline = GetFirstHitPolyline(this.sourcePort.Location, this.ObstacleCalculator.RootOfLooseHierarchy);
            this.targetTightPolyline = GetFirstHitPolyline(this.targetPort.Location, this.ObstacleCalculator.RootOfTightHierarchy);
            this.targetLoosePolyline = GetFirstHitPolyline(this.targetPort.Location, this.ObstacleCalculator.RootOfLooseHierarchy);
            this.activeRectangle = new Rectangle(new Point(double.NegativeInfinity, double.PositiveInfinity),
                                            new Point(double.PositiveInfinity, double.NegativeInfinity));
        }

        private IEnumerable<Polygon> AllPolygons() {
            foreach (Polyline p in this.ObstacleCalculator.LooseObstacles) {
                yield return new Polygon(p);
            }
        }


        /// <summary>
        /// </summary>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1024:UsePropertiesWhereAppropriate")]
        public VisibilityGraph GetVisibilityGraph() {
            return this.VisibilityGraph;
        }

        //        internal void CalculateVisibilityGraph(IEnumerable<EdgeGeometry> edgeGeometries, bool qualityAtPorts)
        //        {
        //            CalculateWholeTangentVisibilityGraph();
        //            if (ConeSpannerAngle > 0 && qualityAtPorts && edgeGeometries != null)
        //                CalculatePortVisibilityGraph(GetPortLocationsPointSet(edgeGeometries));
        //        }

#if DEBUG_MSAGL && !SILVERLIGHT
        internal void ShowObstaclesAndVisGraph()
        {
            var obs = this.obstacleCalculator.LooseObstacles.Select(o => new DebugCurve(100, 1, "blue", o));
            var edges =
                visibilityGraph.Edges.Select(
                    e =>
                    new DebugCurve(70, 1, (e is TransientVisibilityEdge ? "red" : "green"),
                                   new LineSegment(e.SourcePoint, e.TargetPoint)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(obs.Concat(edges));
        }
#endif

        internal void AddActivePolygons(IEnumerable<Polygon> polygons) {
            this.activePolygons.AddRange(polygons);
        }
        internal void ClearActivePolygons() {
            this.activePolygons.Clear();
        }
    }
}