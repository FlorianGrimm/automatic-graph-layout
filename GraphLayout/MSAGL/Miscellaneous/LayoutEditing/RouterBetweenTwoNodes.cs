using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Prototype.LayoutEditing {
    /// <summary>
    /// the router between nodes
    /// </summary>
    public class RouterBetweenTwoNodes {
        private readonly Dictionary<Point, Polyline> pointsToObstacles = new Dictionary<Point, Polyline>();
        private VisibilityGraph _visGraph;
        private ObstacleCalculator obstacleCalculator;

        /// <summary>
        /// the port of the edge start
        /// </summary>
        public Port SourcePort { get; private set; }

        /// <summary>
        /// the port of the edge end
        /// </summary>
        public Port TargetPort { get; private set; }

        private const double enteringAngleBound = 10;

        /// <summary>
        /// the minimum angle between a node boundary curve and and an edge 
        /// curve at the place where the edge curve intersects the node boundary
        /// </summary>
        public double EnteringAngleBound {
            get { return enteringAngleBound; }
        }

        private double minimalPadding = 1;

        /// <summary>
        /// the curve should not come to the nodes closer than MinimalPaddin
        /// </summary>
        public double Padding {
            get { return this.minimalPadding; }
            private set { this.minimalPadding = value; }
        }


        /// <summary>
        /// we pad each node but not more than MaximalPadding
        /// </summary>
        public double LoosePadding { get; set; }


        [SuppressMessage("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        private readonly GeometryGraph graph;

        internal Node Target { get; private set; }

        private VisibilityVertex sourceVisibilityVertex;
        private VisibilityVertex targetVisibilityVertex;

        private VisibilityVertex TargetVisibilityVertex {
            get { return this.targetVisibilityVertex; }
            //            set { targetVisibilityVertex = value; }
        }

        internal Node Source { get; private set; }

        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal GeometryGraph Graph {
            get { return this.graph; }
        }

        private Polyline Polyline { get; set; }

        internal static double DistanceFromPointToPolyline(Point p, Polyline poly) {
            double d = double.PositiveInfinity;
            double u;
            for (PolylinePoint pp = poly.StartPoint; pp.Next != null; pp = pp.Next) {
                d = Math.Min(d, Point.DistToLineSegment(p, pp.Point, pp.Next.Point, out u));
            }

            return d;
        }

        /// <summary>
        /// 
        /// </summary>
        [SuppressMessage("Microsoft.Naming",
            "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        private double OffsetForPolylineRelaxing { get; set; }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="graph"></param>
        /// <param name="minimalPadding"></param>
        /// <param name="maximalPadding"></param>
        /// <param name="offsetForRelaxing"></param>
        public RouterBetweenTwoNodes(GeometryGraph graph, double minimalPadding, double maximalPadding,
                                     double offsetForRelaxing) {
            this.graph = graph;
            this.LoosePadding = maximalPadding;
            this.Padding = minimalPadding;
            this.OffsetForPolylineRelaxing = offsetForRelaxing;
        }


        /// <summary>
        /// Routes a spline between two graph nodes. sourcePort and targetPort define the start and end point of the resulting curve:
        /// startPoint=source.BoundaryCurve[sourcePort] and endPoint=target.BoundaryCurve[target].
        /// </summary>
        /// <param name="edge"></param>
        /// <param name="takeYourTime">if set to true then the method will try to improve the spline</param>
        public void RouteEdge(Edge edge, bool takeYourTime) {
            this.Source = edge.Source;
            this.Target = edge.Target;
            this.SourcePort = edge.SourcePort;
            this.TargetPort = edge.TargetPort;
            this.CalculateObstacles();
            LineSegment lineSeg = this.TryRouteStraightLine();
            if (lineSeg != null) {
                this.Polyline = new Polyline(lineSeg.Start, lineSeg.End);
                edge.UnderlyingPolyline = SmoothedPolyline.FromPoints(this.Polyline);
            } else {
                this.CalculateTangentVisibilityGraph();
                this.Polyline = this.GetShortestPolyline();
                // ShowPolylineAndObstacles();
                this.RelaxPolyline();
                //ShowPolylineAndObstacles();
                //ReducePolyline();
                //ShowPolylineAndObstacles();
                edge.UnderlyingPolyline = SmoothedPolyline.FromPoints(this.Polyline);

                if (takeYourTime) {
                    this.TryToRemoveInflectionsAndCollinearSegs(edge.UnderlyingPolyline);
                    this.SmoothCorners(edge.UnderlyingPolyline);
                }
            }

            edge.Curve = edge.UnderlyingPolyline.CreateCurve();
        }

        //void ReducePolyline() {
        //    for (PolylinePoint pp = this.Polyline.StartPoint.Next; pp.Next != null && pp.Next.Next != null;pp=pp.Next )
        //        pp = TryToRemoveOrDiminishSegment(pp, pp.Next);
        //}

        //PolylinePoint TryToRemoveOrDiminishSegment(PolylinePoint pp, PolylinePoint polylinePoint) {
        //    TriangleOrientation orientation = Point.GetTriangleOrientation(pp.Prev.Point, pp.Point, pp.Next.Point);
        //    if (orientation == Point.GetTriangleOrientation(pp.Point, pp.Next.Point, pp.Next.Next.Point)) {
        //        Point x;
        //        if (Point.LineLineIntersection(pp.Prev.Point, pp.Point, pp.Next.Point, pp.Next.Next.Point, out x)) {
        //            if (orientation == Point.GetTriangleOrientation(pp.Point, x, pp.Next.Point)) {
        //                if (!LineIntersectsTightObstacles(pp.Prev.Point, x) && !LineIntersectsTightObstacles(x, pp.Next.Next.Point)) {
        //                    PolylinePoint px = new PolylinePoint(x);
        //                    //inserting px instead of pp and pp.Next
        //                    px.Prev = pp.Prev;
        //                    pp.Prev.Next = px;
        //                    px.Next = pp.Next.Next;
        //                    pp.Next.Next.Prev = px;
        //                    return px.Prev;
        //                } else {
        //                    for (double k = 0.5; k > 0.01; k /= 2) {
        //                        Point a = pp.Point * (1 - k) + x * k;
        //                        Point b = pp.Next.Point * (1 - k) + x * k;

        //                        if (!LineIntersectsTightObstacles(pp.Point, a) &&
        //                            !LineIntersectsTightObstacles(a, b) &&
        //                            !LineIntersectsTightObstacles(b, pp.Next.Point)) {
        //                            pp.Point = a;
        //                            pp.Next.Point = b;
        //                            break;
        //                        }
        //                    }
        //                }
        //            }
        //        }
        //    }
        //    return pp;
        //}


        //bool LineIntersectsTightObstacles(Point point, Point x) {
        //    return LineIntersectsTightObstacles(new LineSegment(point, x));    
        //}
#if TEST_MSAGL
        //private void ShowPolylineAndObstaclesWithGraph() {
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
            RelaxedPolylinePoint relaxedPolylinePoint = CreateRelaxedPolylinePoints(this.Polyline);
            //ShowPolylineAndObstacles();
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
                var r = new RelaxedPolylinePoint(p, p.Point) { Prev = currentRelaxed };
                currentRelaxed.Next = r;
                currentRelaxed = r;
            }
            return ret;
        }

        private void RelaxPolylinePoint(RelaxedPolylinePoint relaxedPoint) {
            for (double d = this.OffsetForPolylineRelaxing; !this.RelaxWithGivenOffset(d, relaxedPoint); d /= 2) {
            }
        }

        private bool RelaxWithGivenOffset(double offset, RelaxedPolylinePoint relaxedPoint) {
            SetRelaxedPointLocation(offset, relaxedPoint);
#if TEST_MSAGL
            //ShowPolylineAndObstacles();
#endif
            if (this.StickingSegmentDoesNotIntersectTightObstacles(relaxedPoint)) {
                return true;
            }

            PullCloserRelaxedPoint(relaxedPoint.Prev);
            return false;
        }

        private static void PullCloserRelaxedPoint(RelaxedPolylinePoint relaxedPolylinePoint) {
            relaxedPolylinePoint.PolylinePoint.Point = 0.2 * relaxedPolylinePoint.OriginalPosition +
                                                       0.8 * relaxedPolylinePoint.PolylinePoint.Point;
        }

        private bool StickingSegmentDoesNotIntersectTightObstacles(RelaxedPolylinePoint relaxedPoint) {
            return
                !this.LineIntersectsTightObstacles(new LineSegment(relaxedPoint.PolylinePoint.Point,
                                                              relaxedPoint.Prev.PolylinePoint.Point)) && (
                                                                                                             (relaxedPoint
                                                                                                                  .Next ==
                                                                                                              null ||
                                                                                                              !this.LineIntersectsTightObstacles
                                                                                                                   (new LineSegment
                                                                                                                        (relaxedPoint
                                                                                                                             .
                                                                                                                             PolylinePoint
                                                                                                                             .
                                                                                                                             Point,
                                                                                                                         relaxedPoint
                                                                                                                             .
                                                                                                                             Next
                                                                                                                             .
                                                                                                                             PolylinePoint
                                                                                                                             .
                                                                                                                             Point))));
        }

        private bool LineIntersectsTightObstacles(LineSegment ls) {
            return LineIntersectsRectangleNode(ls, this.obstacleCalculator.RootOfTightHierararchy);
        }

        private static bool LineIntersectsRectangleNode(LineSegment ls, RectangleNode<Polyline, Point> rectNode) {
            if (!ls.BoundingBox.Intersects((Rectangle)rectNode.Rectangle)) {
                return false;
            }

            if (rectNode.UserData != null) {
                // SugiyamaLayoutSettings.Show(ls, rectNode.UserData);
                return Curve.GetAllIntersections(rectNode.UserData, ls, false).Count > 0;
            }


            return LineIntersectsRectangleNode(ls, rectNode.Left) || LineIntersectsRectangleNode(ls, rectNode.Right);
        }

        private static void SetRelaxedPointLocation(double offset, RelaxedPolylinePoint relaxedPoint) {
            bool leftTurn = Point.GetTriangleOrientation(relaxedPoint.Next.OriginalPosition,
                                                         relaxedPoint.OriginalPosition,
                                                         relaxedPoint.Prev.OriginalPosition) ==
                            TriangleOrientation.Counterclockwise;
            Point v =
                ((relaxedPoint.Next.OriginalPosition - relaxedPoint.Prev.OriginalPosition).Normalize() * offset).Rotate(
                    Math.PI / 2);

            if (!leftTurn) {
                v = -v;
            }

            relaxedPoint.PolylinePoint.Point = relaxedPoint.OriginalPosition + v;
        }

#if TEST_MSAGL
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowPolylineAndObstacles(){
            List<ICurve> ls = this.CreateListWithObstaclesAndPolyline();
            SugiyamaLayoutSettings.Show(ls.ToArray());
        }


        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private List<ICurve> CreateListWithObstaclesAndPolyline(){
            var ls = new List<ICurve>();
            foreach (Polyline poly in this.obstacleCalculator.TightObstacles) {
                ls.Add(poly);
            }

            foreach (Polyline poly in this.obstacleCalculator.LooseObstacles) {
                ls.Add(poly);
            }

            ls.Add(this.Polyline);
            return ls;
        }
#endif

        private void SmoothCorners(SmoothedPolyline edgePolyline) {
            CornerSite a = edgePolyline.HeadSite; //the corner start
            CornerSite b; //the corner origin
            CornerSite c; //the corner other end
            const double mult = 1.5;

            while (Curve.FindCorner(a, out b, out c)) {
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
                    seg = Curve.CreateBezierSeg(k * u, k * v, a, b, c);
                    b.PreviousBezierSegmentFitCoefficient = k * u;
                    b.NextBezierSegmentFitCoefficient = k * v;
                    k /= mult;
                } while (this.obstacleCalculator.ObstaclesIntersectICurve(seg));

                k *= mult; //that was the last k
                if (k < 0.5) {
                    //one time try a smoother seg
                    k = 0.5 * (k + k * mult);
                    seg = Curve.CreateBezierSeg(k * u, k * v, a, b, c);
                    if (!this.obstacleCalculator.ObstaclesIntersectICurve(seg)) {
                        b.PreviousBezierSegmentFitCoefficient = k * u;
                        b.NextBezierSegmentFitCoefficient = k * v;
                    }
                }
                a = b;
            }
        }

        private void TryToRemoveInflectionsAndCollinearSegs(SmoothedPolyline underlyingPolyline) {
            bool progress = true;
            while (progress) {
                progress = false;
                for (CornerSite s = underlyingPolyline.HeadSite; s != null && s.Next != null; s = s.Next) {
                    if (s.Turn * s.Next.Turn < 0) {
                        progress = this.TryToRemoveInflectionEdge(ref s) || progress;
                    }
                }
            }
        }

        private bool TryToRemoveInflectionEdge(ref CornerSite s) {
            if (!this.obstacleCalculator.ObstaclesIntersectLine(s.Previous.Point, s.Next.Point)) {
                CornerSite a = s.Previous; //forget s
                CornerSite b = s.Next;
                a.Next = b;
                b.Previous = a;
                s = a;
                return true;
            }
            if (!this.obstacleCalculator.ObstaclesIntersectLine(s.Previous.Point, s.Next.Next.Point)) {
                //forget about s and s.Next
                CornerSite a = s.Previous;
                CornerSite b = s.Next.Next;
                a.Next = b;
                b.Previous = a;
                s = a;
                return true;
            }
            if (!this.obstacleCalculator.ObstaclesIntersectLine(s.Point, s.Next.Next.Point)) {
                //forget about s.Next
                CornerSite b = s.Next.Next;
                s.Next = b;
                b.Previous = s;
                return true;
            }

            return false;
        }

        private LineSegment TryRouteStraightLine() {
            var ls = new LineSegment(this.SourcePoint, this.TargetPoint);
            if (this.obstacleCalculator.ObstaclesIntersectICurve(ls)) {
                return null;
            }

            return ls;
        }

        internal Point TargetPoint {
            get {
                var tp = this.TargetPort as CurvePort;
                if (tp != null) {
                    return this.Target.BoundaryCurve[tp.Parameter];
                }

                return this.TargetPort.Location;
            }
        }

        internal Point SourcePoint {
            get {
                var sp = this.SourcePort as CurvePort;
                if (sp != null) {
                    return this.Source.BoundaryCurve[sp.Parameter];
                }

                return this.SourcePort.Location;
            }
        }


        [SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        private Polyline GetShortestPolyline() {
            var pathCalc = new SingleSourceSingleTargetShortestPathOnVisibilityGraph(this._visGraph, this.sourceVisibilityVertex,
                                                                                     this.TargetVisibilityVertex);
            var path = pathCalc.GetPath(false);
            var ret = new Polyline();
            foreach (var v in path) {
                ret.AddPoint(v.Point);
            }

            return RemoveCollinearPoint(ret);
        }

        private static Polyline RemoveCollinearPoint(Polyline ret) {
            for (PolylinePoint pp = ret.StartPoint.Next; pp.Next != null; pp = pp.Next) {
                if (Point.GetTriangleOrientation(pp.Prev.Point, pp.Point, pp.Next.Point) == TriangleOrientation.Collinear) {
                    pp.Prev.Next = pp.Next;
                    pp.Next.Prev = pp.Prev;
                }
            }

            return ret;
        }

        [SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        private void CalculateTangentVisibilityGraph() {
            this._visGraph = VisibilityGraph.GetVisibilityGraphForShortestPath(
                this.SourcePoint, this.TargetPoint, this.obstacleCalculator.LooseObstacles, out this.sourceVisibilityVertex,
                out this.targetVisibilityVertex);
        }

        private void CalculateObstacles() {
            this.obstacleCalculator = new ObstacleCalculator(this);
            this.obstacleCalculator.Calculate();
            foreach (Polyline poly in this.obstacleCalculator.TightObstacles) {
                foreach (Point p in poly) {
                    this.pointsToObstacles[p] = poly;
                }
            }
        }
    }
}