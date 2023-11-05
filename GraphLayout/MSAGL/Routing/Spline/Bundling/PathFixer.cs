using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class PathFixer {
        private MetroGraphData metroGraphData;
        private Func<Metroline, Point, bool> polylineAcceptsPoint;
        private Set<Point> foundCrossings = new Set<Point>();
        private Set<Point> crossingsThatShouldBecomeHubs = new Set<Point>();
        private Set<Point> pointsToDelete;

        public PathFixer(MetroGraphData metroGraphData, Func<Metroline, Point, bool> polylineAcceptsPoint) {
            this.metroGraphData = metroGraphData;
            this.polylineAcceptsPoint = polylineAcceptsPoint;
        }

        private IEnumerable<PolylinePoint> Vertices() {
            return this.metroGraphData.Edges.Select(e => (Polyline)e.Curve).SelectMany(p => p.PolylinePoints);
        }

        private IEnumerable<Polyline> Polylines { get { return this.metroGraphData.Edges.Select(e => (Polyline)e.Curve); } }

        private IEnumerable<PointPair> Edges() {
            var set = new Set<PointPair>();
            foreach (var pp in this.Vertices()) {
                if (pp.Next != null) {
                    set.Insert(FlipCollapser.OrderedPair(pp));
                }
            }

            return set;
        }

        internal bool Run() {
            if (this.metroGraphData.Edges.Count() == 0) {
                return false;
            }

            var splittingPoints = new Dictionary<PointPair, List<Point>>();
            var treeOfVertices = new RTree<Point,Point>();
            foreach (var vertex in this.Vertices()) {
                var r = new Rectangle(vertex.Point);
                r.Pad(ApproximateComparer.IntersectionEpsilon);
                treeOfVertices.Add(r, vertex.Point);
            }

            var treeOfEdges = RectangleNode<PointPair,Point>.CreateRectangleNodeOnData(this.Edges(), e => new Rectangle(e.First, e.Second));
            RectangleNodeUtils.CrossRectangleNodes<PointPair, Point>(treeOfEdges, treeOfEdges, (a, b) => this.IntersectTwoEdges(a, b, splittingPoints, treeOfVertices));

            this.SortInsertedPoints(splittingPoints);
            bool pointsInserted = this.InsertPointsIntoPolylines(splittingPoints);

            bool progress = this.FixPaths();

            bool pointsRemoved = this.RemoveUnimportantCrossings();

            return progress || pointsInserted || pointsRemoved;
        }

        private bool FixPaths() {
            bool progress = false;
            if (this.RemoveSelfCycles()) {
                progress = true;
            }
            //if (CollapseCycles()) progress = true;
            if (this.ReduceEdgeCrossings()) {
                progress = true;
            }

            return progress;
        }

        private void SortInsertedPoints(Dictionary<PointPair, List<Point>> splittingPoints) {
            foreach (var pair in splittingPoints) {
                this.SortInsideSegment(pair.Key, pair.Value);
            }
        }

        private void SortInsideSegment(PointPair edge, List<Point> list) {
            System.Diagnostics.Debug.Assert(list.Count > 0, "an edge should not be present with an empty list");
            list.Sort((a, b) => (a - edge.First).Length.CompareTo((b - edge.First).Length));
        }

        private bool InsertPointsIntoPolylines(Dictionary<PointPair, List<Point>> splittingPoints) {
            bool inserted = false;
            foreach (var metroline in this.metroGraphData.Metrolines) {
                if (this.InsertPointsIntoPolyline(metroline, splittingPoints)) {
                    inserted = true;
                }
            }
            return inserted;
        }

        private bool InsertPointsIntoPolyline(Metroline metroline, Dictionary<PointPair, List<Point>> splittingPoints) {
            bool inserted = false;
            for (var pp = metroline.Polyline.StartPoint; pp.Next != null; pp = pp.Next) {
                if (this.InsertPointsOnPolypoint(pp, splittingPoints, metroline)) {
                    inserted = true;
                }
            }

            return inserted;
        }

        private bool InsertPointsOnPolypoint(PolylinePoint pp, Dictionary<PointPair, List<Point>> splittingPoints, Metroline metroline) {
            var pointPair = FlipCollapser.OrderedPair(pp);
            var reversed = pp.Point != pointPair.First;
            List<Point> list;
            if (!splittingPoints.TryGetValue(pointPair, out list)) {
                return false;
            }

            var endPolyPoint = pp.Next;
            var poly = pp.Polyline;
            if (reversed) {
                for (int i = list.Count - 1; i >= 0; i--) {
                    if (this.polylineAcceptsPoint !=null && !this.polylineAcceptsPoint(metroline, list[i])) {
                        continue;
                    }

                    var p = new PolylinePoint(list[i]) { Prev = pp, Polyline = poly };
                    pp.Next = p;
                    pp = p;
                }
            } else {
                for (int i = 0; i < list.Count; i++) {
                    if (this.polylineAcceptsPoint !=null &&!this.polylineAcceptsPoint(metroline, list[i])) {
                        continue;
                    }

                    var p = new PolylinePoint(list[i]) { Prev = pp, Polyline = poly };
                    pp.Next = p;
                    pp = p;
                }
            }

            pp.Next = endPolyPoint;
            endPolyPoint.Prev = pp;
            return true;
        }

        private bool RemoveSelfCycles() {
            bool progress = false;
            foreach (var poly in this.Polylines) {
                if (RemoveSelfCyclesFromPolyline(poly)) {
                    progress = true;
                }
            }

            return progress;
        }

        //returns removed points
        internal static bool RemoveSelfCyclesFromPolyline(Polyline poly) {
            bool progress = false;
            Dictionary<Point, PolylinePoint> pointsToPp = new Dictionary<Point, PolylinePoint>();
            for (var pp = poly.StartPoint; pp != null; pp = pp.Next) {
                var point = pp.Point;
                PolylinePoint previous;

                if (pointsToPp.TryGetValue(point, out previous)) {//we have a cycle
                    for (var px = previous.Next; px != pp.Next; px = px.Next) {
                        pointsToPp.Remove(px.Point);
                    }
                    previous.Next = pp.Next;
                    pp.Next.Prev = previous;
                    progress = true;
                }
                else {
                    pointsToPp[pp.Point] = pp;
                }
            }
            return progress;
        }

        //bool CollapseCycles() {
        //    var cycleCollapser = new FlipCollapser(metroGraphData, bundlingSettings, cdt);
        //    cycleCollapser.Run();
        //    crossingsThatShouldBecomeHubs.InsertRange(cycleCollapser.GetChangedCrossing());
        //    //TimeMeasurer.DebugOutput("#crossingsThatShouldBecomeHubs = " + crossingsThatShouldBecomeHubs.Count);
        //    return false;
        //}

        private bool ReduceEdgeCrossings() {
            var cycleCollapser = new FlipSwitcher(this.metroGraphData);
            cycleCollapser.Run();
            this.crossingsThatShouldBecomeHubs.InsertRange(cycleCollapser.GetChangedHubs());
            //TimeMeasurer.DebugOutput("#reduced crossings = " + cycleCollapser.NumberOfReducedCrossings());
            return cycleCollapser.NumberOfReducedCrossings() > 0;
        }

        private bool RemoveUnimportantCrossings() {
            bool removed = false;
            this.pointsToDelete = this.foundCrossings - this.crossingsThatShouldBecomeHubs;
            foreach (var polyline in this.Polylines) {
                if (this.RemoveUnimportantCrossingsFromPolyline(polyline)) {
                    removed = true;
                }
            }

            return removed;
        }

        private bool RemoveUnimportantCrossingsFromPolyline(Polyline polyline) {
            bool removed = false;
            for (var p = polyline.StartPoint.Next; p != null && p.Next != null; p = p.Next) {
                if (this.pointsToDelete.Contains(p.Point) && Point.GetTriangleOrientation(p.Prev.Point, p.Point, p.Next.Point) == TriangleOrientation.Collinear) {
                    //forget p
                    var pp = p.Prev;
                    var pn = p.Next;
                    pp.Next = pn;
                    pn.Prev = pp;
                    p = pp;
                    removed = true;
                }
            }

            return removed;
        }

        private void IntersectTwoEdges(PointPair a, PointPair b, Dictionary<PointPair, List<Point>> splittingPoints, RTree<Point,Point> tree) {
            Point x;
            if (LineSegment.Intersect(a.First, a.Second, b.First, b.Second, out x)) {
                Point vertex = this.FindExistingVertexOrCreateNew(tree, x);
                if (this.AddVertexToSplittingList(a, splittingPoints, vertex) |
                this.AddVertexToSplittingList(b, splittingPoints, vertex)) {
                    this.foundCrossings.Insert(vertex);
                }
            }
        }

        private Point FindExistingVertexOrCreateNew(RTree<Point,Point> tree, Point x) {
            var p = tree.RootNode.FirstHitNode(x);
            if (p != null) {
                return p.UserData;
            }

            var rect = new Rectangle(x);
            rect.Pad(ApproximateComparer.IntersectionEpsilon);
            tree.Add(rect, x);
            return x;
        }

        private bool AddVertexToSplittingList(PointPair a, Dictionary<PointPair, List<Point>> splittingPoints, Point intersectionPoint) {
#if TEST_MSAGL
            double t;
            System.Diagnostics.Debug.Assert(
                (!ApproximateComparer.CloseIntersections(intersectionPoint, a.First) &&
                !ApproximateComparer.CloseIntersections(intersectionPoint, a.Second)) ||
                Point.DistToLineSegment(intersectionPoint, a.First, a.Second, out t) < ApproximateComparer.IntersectionEpsilon);
#endif
            if (!ApproximateComparer.CloseIntersections(intersectionPoint, a.First) &&
                !ApproximateComparer.CloseIntersections(intersectionPoint, a.Second)) {
                List<Point> list;
                if (!splittingPoints.TryGetValue(a, out list)) {
                    splittingPoints[a] = list = new List<Point>();
                }

                if (!list.Contains(intersectionPoint)) {
                    list.Add(intersectionPoint);
                    return true;
                }
            }
            return false;
        }
    }
}
