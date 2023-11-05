using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Prototype.LayoutEditing {
    /// <summary>
    /// calculations with obstacles
    /// </summary>
    public class ObstacleCalculator {
        private List<Polyline> looseObstacles = new List<Polyline>();
        private Set<ICurve> portObstacles = new Set<ICurve>();
        private RectangleNode<Polyline, Point> rootOfLooseHierarchy;
        private RectangleNode<Polyline, Point> rootOfTightHierarachy;
        private RouterBetweenTwoNodes router;
        private LineSegment sourceFilterLine;
        private LineSegment targetFilterLine;
        private Set<Polyline> tightObstacles = new Set<Polyline>();

        internal ObstacleCalculator(RouterBetweenTwoNodes router) {
            this.router = router;
        }

        internal Set<Polyline> TightObstacles {
            get { return this.tightObstacles; }
            //            set { tightObstacles = value; }
        }

        internal List<Polyline> LooseObstacles {
            get { return this.looseObstacles; }
            //            set { looseObstacles = value; }
        }

        internal RectangleNode<Polyline, Point> RootOfTightHierararchy {
            get { return this.rootOfTightHierarachy; }
            private set { this.rootOfTightHierarachy = value; }
        }

        private RectangleNode<Polyline, Point> RootOfLooseHierarchy {
            get { return this.rootOfLooseHierarchy; }
            set { this.rootOfLooseHierarchy = value; }
        }

        internal LineSegment SourceFilterLine {
            get { return this.sourceFilterLine; }
        }

        internal LineSegment TargetFilterLine {
            get { return this.targetFilterLine; }
            //            set { targetFilterLine = value; }
        }

        private double EnteringAngle {
            get { return this.router.EnteringAngleBound * Math.PI / 180; }
        }

        /// <summary>
        /// There are two sets of obstacles: tight and loose.
        /// We route the shortest path between loose obstacles, and then beautify it while only taking into account tight obstacles
        /// </summary>
        /// <returns></returns>
        internal void Calculate() {
            this.CreateTightObstacles();
            this.CreateLooseObstacles();
        }

        private void CreateLooseObstacles() {
            this.RootOfLooseHierarchy = this.RootOfTightHierararchy.Clone();

            TraverseHierarchy(this.RootOfLooseHierarchy, delegate(RectangleNode<Polyline, Point> node) {
                if (node.UserData != null) {
                    Polyline tightPolyline = node.UserData;
                    double distance =
                        this.FindMaxPaddingForTightPolyline(tightPolyline);
                    this.LooseObstacles.Add(
                        node.UserData =
                        LoosePolylineWithFewCorners(tightPolyline,
                                                    Math.Min(
                                                        this.router.LoosePadding,
                                                        distance * 0.3)));
                    node.Rectangle = node.UserData.BoundingBox;
                    InteractiveObstacleCalculator.UpdateRectsForParents(node);
                }
            });
        }


        //internal void ShowRectangleNodesHierarchy(RectangleNode<Polyline, Point> node) {
        //    List<ICurve> ls = new List<ICurve>();
        //    FillList(ls, node);
        //    SugiyamaLayoutSettings.Show(ls.ToArray());
        //}

        //internal void FillList(List<ICurve> ls, RectangleNode<Polyline, Point> node) {
        //    if (node == null)
        //        return;
        //    if (node.UserData != null)
        //        ls.Add(node.UserData);
        //    else {
        //        FillList(ls, node.Left);
        //        FillList(ls, node.Right);
        //    }
        //}


        private static void TraverseHierarchy(RectangleNode<Polyline, Point> node, Visitor visitor) {
            visitor(node);
            if (node.Left != null) {
                TraverseHierarchy(node.Left, visitor);
            }

            if (node.Right != null) {
                TraverseHierarchy(node.Right, visitor);
            }
        }

        private void CreateTightObstacles() {
            this.CreateInitialTightObstacles();
            List<Set<Polyline>> overlappingPolylineSets;
            do {
                this.RemoveTightObstaclesOverlappingPortTightObstacles();
                this.CalculateTightHierarchy();
                overlappingPolylineSets = this.GetOverlappingSets();
                foreach (var overlappingSet in overlappingPolylineSets) {
                    this.InsertOverlappingSet(overlappingSet);
                }
            } while (overlappingPolylineSets.Count > 0);
        }

        private void RemoveTightObstaclesOverlappingPortTightObstacles() {
            var toRemove = new List<Polyline>();
            foreach (Polyline poly in this.TightObstaclesMinusPortObstacles()) {
                foreach (ICurve portObstacle in this.portObstacles) {
                    if (poly.BoundingBox.Intersects(portObstacle.BoundingBox)) {
                        if (Curve.GetAllIntersections(poly, portObstacle, false).Count > 0 ||
                            OneCurveLiesInsideOfOther(poly, portObstacle)) {
                            toRemove.Add(poly);
                        }
                    }
                }
            }

            foreach (Polyline poly in toRemove) {
                this.TightObstacles.Remove(poly);
            }
        }

        private IEnumerable<Polyline> TightObstaclesMinusPortObstacles() {
            foreach (Polyline p in this.TightObstacles) {
                if (this.portObstacles.Contains(p) == false) {
                    yield return p;
                }
            }
        }

        private void InsertOverlappingSet(Set<Polyline> overlappingSet) {
            foreach (Polyline p in overlappingSet) {
                this.tightObstacles.Remove(p);
            }

            var hull = new Polyline();
            foreach (Point p in ConvexHull.CalculateConvexHull(this.EnumerateOverSetOfPolylines(overlappingSet))) {
                hull.AddPoint(p);
            }

            hull.Closed = true;

            //debug 
            //List<ICurve> ls=new List<ICurve>();
            //foreach(Polyline p in overlappingSet)
            //    ls.Add(p);

            //ls.Add(hull);
            //SugiyamaLayoutSettings.Show(ls.ToArray());
            //end of debug

            this.tightObstacles.Insert(hull);
        }

        private IEnumerable<Point> EnumerateOverSetOfPolylines(Set<Polyline> pp) {
            foreach (Polyline poly in pp) {
                foreach (Point p in poly) {
                    yield return p;
                }
            }
        }

        private List<Set<Polyline>> GetOverlappingSets() {
            PolylineGraph overlapGraph = this.CalculateOverlapGraph();
            return ConnectedComponents(overlapGraph);
        }

        private static List<Set<Polyline>> ConnectedComponents(PolylineGraph overlapGraph) {
            var list = new List<Set<Polyline>>();
            var processedPolylines = new Set<Polyline>();
            foreach (Polyline poly in overlapGraph.Nodes) {
                if (!processedPolylines.Contains(poly)) {
                    Set<Polyline> component = GetComponent(poly, overlapGraph);
                    if (component.Count > 1) {
                        list.Add(component);
                    }

                    processedPolylines += component;
                }
            }
            return list;
        }

        private static Set<Polyline> GetComponent(Polyline poly, PolylineGraph graph) {
            var ret = new Set<Polyline>();
            ret.Insert(poly);
            var queue = new Queue<Polyline>();
            queue.Enqueue(poly);
            while (queue.Count > 0) {
                foreach (Polyline p in graph.Descendents(queue.Dequeue())) {
                    if (!ret.Contains(p)) {
                        queue.Enqueue(p);
                        ret.Insert(p);
                    }
                }
            }
            return ret;
        }

        private PolylineGraph CalculateOverlapGraph() {
            var graph = new PolylineGraph();
            this.CreateEdgesUnderTwoNodes(this.rootOfTightHierarachy, this.rootOfTightHierarachy, graph);
            return graph;
        }

        private void CalculateTightHierarchy() {
            var rectNodes = new List<RectangleNode<Polyline, Point>>();
            foreach (Polyline polyline in this.TightObstacles) {
                rectNodes.Add(CreateRectNodeOfPolyline(polyline));
            }

            this.RootOfTightHierararchy = RectangleNode<Polyline, Point>.CreateRectangleNodeOnListOfNodes(rectNodes);
        }

        private static RectangleNode<Polyline, Point> CreateRectNodeOfPolyline(Polyline polyline) {
            return new RectangleNode<Polyline, Point>(polyline, (polyline as ICurve).BoundingBox);
        }

        private void CreateEdgesUnderTwoNodes(RectangleNode<Polyline, Point> a, RectangleNode<Polyline, Point> b,
                                      PolylineGraph overlapGraph) {
            //if (a.GetHashCode() < b.GetHashCode())
            //    return;

            Debug.Assert((a.UserData == null && a.Left != null && a.Right != null) ||
                         (a.UserData != null && a.Left == null && a.Right == null));
            Debug.Assert((b.UserData == null && b.Left != null && b.Right != null) ||
                         (b.UserData != null && b.Left == null && b.Right == null));
            if (a.Rectangle.Intersects(b.Rectangle)) {
                if (a.UserData != null) {
                    if (b.UserData != null) {
                        if (a.UserData != b.UserData) {
                            if (Curve.GetAllIntersections(a.UserData, b.UserData, false).Count > 0 ||
                                OneCurveLiesInsideOfOther(a.UserData, b.UserData)) {
                                overlapGraph.AddEdge(a.UserData, b.UserData);
                                overlapGraph.AddEdge(b.UserData, a.UserData);
                            }
                        }
                    } else {
                        this.CreateEdgesUnderTwoNodes(a, b.Left, overlapGraph);
                        this.CreateEdgesUnderTwoNodes(a, b.Right, overlapGraph);
                    }
                } else if (b.UserData != null) {
                    this.CreateEdgesUnderTwoNodes(b, a.Left, overlapGraph);
                    this.CreateEdgesUnderTwoNodes(b, a.Right, overlapGraph);
                } else {
                    this.CreateEdgesUnderTwoNodes(a.Left, b.Left, overlapGraph);
                    this.CreateEdgesUnderTwoNodes(a.Left, b.Right, overlapGraph);
                    this.CreateEdgesUnderTwoNodes(a.Right, b.Left, overlapGraph);
                    this.CreateEdgesUnderTwoNodes(a.Right, b.Right, overlapGraph);
                }
            }
        }

        private static bool OneCurveLiesInsideOfOther(ICurve polyA, ICurve polyB) {
            return (Curve.PointRelativeToCurveLocation(polyA.Start, polyB) != PointLocation.Outside ||
                    Curve.PointRelativeToCurveLocation(polyB.Start, polyA) != PointLocation.Outside);
        }


        /// <summary>
        /// 
        /// </summary>
        private void CreateInitialTightObstacles() {
            this.tightObstacles = new Set<Polyline>();
            foreach (Node node in this.router.Graph.Nodes) {
                if (node == this.router.Source) {
                    this.CreatePortObstacles(this.router.Source, this.router.SourcePort, out this.sourceFilterLine);
                } else if (node == this.router.Target) {
                    this.CreatePortObstacles(this.router.Target, this.router.TargetPort, out this.targetFilterLine);
                } else {
                    this.TightObstacles.Insert(PaddedPolylineBoundaryOfNode(node, this.router.Padding));
                }
            }
        }

        private void CreatePortObstacles(Node node, Port port, out LineSegment filterLine) {
            var bp = port as CurvePort;
            if (bp != null) {
                var padding = this.router.Padding;
                var closedCurve = node.BoundaryCurve;
                Curve paddingCurve = GetPaddedPolyline(closedCurve, padding).ToCurve();
                Point portPoint = node.BoundaryCurve[bp.Parameter];
                double length = node.BoundaryCurve.BoundingBox.Width + node.BoundaryCurve.BoundingBox.Height;
                double leftTipPar = this.GetLeftTipParam(bp.Parameter, portPoint, paddingCurve, node, length);
                double rightTipPar = this.GetRightTipParam(bp.Parameter, portPoint, paddingCurve, node, length);
                paddingCurve = TrimCurve(paddingCurve, rightTipPar, leftTipPar);
                //a simplifying hack here. I know that the parameter start from 0 and advances by 1 on every segment                
                int n = paddingCurve.Segments.Count / 2;
                Debug.Assert(n > 0);
                Curve rightChunk = TrimCurve(paddingCurve, 0, n);
                Curve leftChunk = TrimCurve(paddingCurve, n + 0.8, paddingCurve.ParEnd);
                filterLine = new LineSegment(0.5 * (leftChunk.Start + leftChunk.End),
                                             0.5 * (rightChunk.Start + rightChunk.End));
                Polyline pol = Polyline.PolylineFromCurve(leftChunk);
                pol.Closed = true;
                this.portObstacles.Insert(pol);
                this.TightObstacles.Insert(pol);
                pol = Polyline.PolylineFromCurve(rightChunk);
                pol.Closed = true;
                this.portObstacles.Insert(pol);
                this.TightObstacles.Insert(pol);
            } else {
                filterLine = null;
                this.portObstacles.Insert(node.BoundaryCurve);
            }
        }

        ///<summary>
        ///</summary>
        ///<param name="closedCurve"></param>
        ///<param name="padding"></param>
        ///<returns></returns>
        static public Polyline GetPaddedPolyline(ICurve closedCurve, double padding) {
            return InteractiveObstacleCalculator.CreatePaddedPolyline(Curve.PolylineAroundClosedCurve(closedCurve), padding);
        }

        private static Curve TrimCurve(Curve curve, double u, double v) {
            Debug.Assert(u >= curve.ParStart && u <= curve.ParEnd);
            Debug.Assert(v >= curve.ParStart && v <= curve.ParEnd);
            if (u < v) {
                return curve.Trim(u, v) as Curve;
            }

            var c = new Curve();
            c.AddSegment(curve.Trim(u, curve.ParEnd) as Curve);
            c.AddSegment(curve.Trim(curve.ParStart, v) as Curve);
            return c;
        }

        private double GetRightTipParam(double portParam, Point portPoint, Curve paddingCurve, Node node, double length) {
            bool curveIsClockwise = InteractiveObstacleCalculator.CurveIsClockwise(node.BoundaryCurve, node.Center);
            Point tan = curveIsClockwise
                            ? node.BoundaryCurve.RightDerivative(portParam)
                            : -node.BoundaryCurve.LeftDerivative(portParam);
            tan = (tan.Normalize() * length).Rotate(this.EnteringAngle);
            IList<IntersectionInfo> xs = Curve.GetAllIntersections(paddingCurve,
                                                                   new LineSegment(portPoint, portPoint + tan), true);
            Debug.Assert(xs.Count == 1);
            return xs[0].Par0;
        }

        private double GetLeftTipParam(double portParam, Point portPoint, Curve paddingCurve, Node node, double length) {
            bool curveIsClockwise = InteractiveObstacleCalculator.CurveIsClockwise(node.BoundaryCurve, node.Center);
            Point tan = curveIsClockwise
                            ? node.BoundaryCurve.LeftDerivative(portParam)
                            : -node.BoundaryCurve.RightDerivative(portParam);
            tan = ((-tan.Normalize()) * length).Rotate(-this.EnteringAngle);
            IList<IntersectionInfo> xs = Curve.GetAllIntersections(paddingCurve,
                                                                   new LineSegment(portPoint, portPoint + tan), true);
            Debug.Assert(xs.Count == 1);
            return xs[0].Par0;
        }


        /// <summary>
        /// Creates a padded polyline boundary of the node. The polyline offsets at least as the padding from the node boundary.
        /// </summary>
        /// <param name="node"></param>
        /// <param name="padding"></param>
        /// <returns></returns>
        private static Polyline PaddedPolylineBoundaryOfNode(Node node, double padding) {
            return InteractiveObstacleCalculator.CreatePaddedPolyline(Curve.PolylineAroundClosedCurve(node.BoundaryCurve), padding);
        }

        private static Polyline LoosePolylineWithFewCorners(Polyline tightPolyline, double p) {
            if (p < ApproximateComparer.DistanceEpsilon) {
                return tightPolyline;
            }

            Polyline loosePolyline = CreateLoosePolylineOnBisectors(tightPolyline, p);
            return loosePolyline;
        }

        //private Polyline CutCorners(Polyline loosePolyline, Polyline tightPolyline) {
        //    Polyline ret = new Polyline();
        //    ret.Closed = true;
        //    PolylinePoint pp = loosePolyline.StartPoint;
        //    PolylinePoint tpp=tightPolyline.StartPoint;

        //    do {
        //        PolylinePoint furthestVisible = GetFurthestVisible(pp, ref tpp);
        //        ret.AddPoint(furthestVisible.Point);
        //        pp = furthestVisible;
        //    }
        //    while (pp != loosePolyline.StartPoint);

        //    System.Diagnostics.Debug.Assert(pp == loosePolyline.StartPoint);
        //    //distangle ret.StartPoint and ret.LastPoint
        //    return ret;
        //}

        //static PolylinePoint GetFurthestVisible(PolylinePoint pp, ref PolylinePoint tpp) {
        //    Point pivot = pp.Point;
        //    Point blockingPoint = tpp.NextOnPolyline.Point;
        //    while (Point.GetTriangleOrientation(pivot, blockingPoint, pp.NextOnPolyline.Point) == TriangleOrientation.Counterclockwise) {
        //        pp = pp.NextOnPolyline;
        //        tpp = tpp.NextOnPolyline;
        //    }
        //    return pp;
        //}

        private static Polyline CreateLoosePolylineOnBisectors(Polyline tightPolyline, double p) {
            var ret = new Polyline();

            ret.AddPoint(GetStickingVertexOnBisector(tightPolyline.StartPoint, p));
            var blockingPoint = new Point(); //to silence the compiler
            var candidate = new Point();
            bool justAdded = true;

            for (PolylinePoint pp = tightPolyline.StartPoint.Next; pp != null; pp = pp.Next) {
                Point currentSticking = GetStickingVertexOnBisector(pp, p);
                if (justAdded) {
                    blockingPoint = pp.Point;
                    candidate = currentSticking;
                    justAdded = false;
                } else {
                    if (ret.Count > 1) {
                        // SugiyamaLayoutSettings.Show(tightPolyline, ret, new LineSegment(ret.StartPoint.Point, currentSticking));
                    }

                    //SugiyamaLayoutSettings.Show(new LineSegment(ret.EndPoint.Point, blockingPoint), tightPolyline, new LineSegment(ret.EndPoint.Point, currentSticking));
                    if (Point.GetTriangleOrientation(ret.EndPoint.Point, blockingPoint, currentSticking) !=
                        TriangleOrientation.Counterclockwise) {
                        ret.AddPoint(candidate);
                        // SugiyamaLayoutSettings.Show(ret, tightPolyline);
                        justAdded = true;
                        pp = pp.Prev;
                    } else {
                        candidate = currentSticking;
                        if (Point.GetTriangleOrientation(ret.EndPoint.Point, blockingPoint, pp.Point) ==
                            TriangleOrientation.Counterclockwise) {
                            blockingPoint = pp.Point;
                        }
                    }
                }
            }

            //process the last point
            if (!justAdded) {
                if (Point.GetTriangleOrientation(ret.EndPoint.Point, blockingPoint, ret.StartPoint.Point) ==
                    TriangleOrientation.Counterclockwise) {
                    //the first point is visible, but now can we cut it
                    if (Point.GetTriangleOrientation(ret.EndPoint.Point, blockingPoint, ret.StartPoint.Next.Point) ==
                        TriangleOrientation.Counterclockwise) {
                        ret.RemoveStartPoint();
                    }
                } else {
                    ret.AddPoint(candidate);
                }
            } else {
                //trying to cut away the first point
                if (
                    Point.GetTriangleOrientation(ret.EndPoint.Point, tightPolyline.StartPoint.Point,
                                                 ret.StartPoint.Next.Point) == TriangleOrientation.Counterclockwise) {
                    ret.RemoveStartPoint();
                } else { }
            }

            ret.Closed = true;
            // SugiyamaLayoutSettings.Show(tightPolyline, ret);
            return ret;
        }

        private static Point GetStickingVertexOnBisector(PolylinePoint pp, double p) {
            Point u = pp.Polyline.Prev(pp).Point;
            Point v = pp.Point;
            Point w = pp.Polyline.Next(pp).Point;
            return p * ((v - u).Normalize() + (v - w).Normalize()).Normalize() + v;
        }

        private double FindMaxPaddingForTightPolyline(Polyline polyline) {
            var dist = double.MaxValue;
            var polygon = new Polygon(polyline);

            foreach (var poly in this.RootOfLooseHierarchy.GetAllLeaves().Where(p => p != polyline)) {
                dist = Math.Min(dist, Polygon.Distance(polygon, new Polygon(poly)));
            }

            //            TraverseHierarchy(RootOfLooseHierarchy, delegate(RectangleNode<Polyline, Point> node) {
            //                                                        if (node.UserData != null)
            //                                                            if (node.UserData != polyline)
            //                                                                dist = Math.Min(dist,
            //                                                                                Polygon.Distance(polygon,
            //                                                                                                 new Polygon(
            //                                                                                                     node.UserData)));
            //                                                    });
            dist = Math.Min(dist, RouterBetweenTwoNodes.DistanceFromPointToPolyline(this.router.SourcePoint, polyline));
            dist = Math.Min(dist, RouterBetweenTwoNodes.DistanceFromPointToPolyline(this.router.TargetPoint, polyline));
            return dist;
        }

        private static bool CurvesIntersect(ICurve a, ICurve b) {
            return Curve.GetAllIntersections(a, b, false).Count > 0;
        }

        internal bool ObstaclesIntersectLine(Point a, Point b) {
            return this.ObstaclesIntersectICurve(new LineSegment(a, b));
        }

        internal bool ObstaclesIntersectICurve(ICurve curve) {
            return CurveIntersectsRectangleNode(curve, this.RootOfTightHierararchy)
                   ||
                   (this.SourceFilterLine != null && CurvesIntersect(curve, this.SourceFilterLine))
                   ||
                   (this.TargetFilterLine != null && CurvesIntersect(curve, this.TargetFilterLine));
        }

        internal static bool CurveIntersectsRectangleNode(ICurve curve, RectangleNode<Polyline, Point> rectNode) {
            Rectangle boundingBox = curve.BoundingBox;
            return CurveIntersectsRectangleNode(curve, ref boundingBox, rectNode);
        }

        private static bool CurveIntersectsRectangleNode(ICurve curve, ref Rectangle curveBox, RectangleNode<Polyline, Point> rectNode) {
            if (!rectNode.Rectangle.Intersects(curveBox)) {
                return false;
            }

            if (rectNode.UserData != null) {
                return Curve.CurveCurveIntersectionOne(rectNode.UserData, curve, false) != null ||
                       Inside(rectNode.UserData, curve);
            }

            Debug.Assert(rectNode.Left != null && rectNode.Right != null);

            return CurveIntersectsRectangleNode(curve, ref curveBox, rectNode.Left) ||
                   CurveIntersectsRectangleNode(curve, ref curveBox, rectNode.Right);
        }

        /// <summary>
        /// we know here that there are no intersection between "curveUnderTest" and "curve",
        /// We are testing that curve is inside of "curveUnderTest"
        /// </summary>
        /// <param name="curveUnderTest"></param>
        /// <param name="curve"></param>
        /// <returns></returns>
        private static bool Inside(ICurve curveUnderTest, ICurve curve) {
            return Curve.PointRelativeToCurveLocation(curve.Start, curveUnderTest) == PointLocation.Inside;
        }

        #region Nested type: Visitor

        private delegate void Visitor(RectangleNode<Polyline, Point> node);

        #endregion
        /// <summary>
        /// 
        /// </summary>
        /// <param name="quadrilateral"></param>
        /// <param name="rectangleNode"></param>
        /// <param name="polylineToIgnore"></param>
        /// <returns></returns>
        public static bool CurveIntersectsRectangleNode(Polyline quadrilateral, RectangleNode<Polyline, Point> rectangleNode, Polyline polylineToIgnore) {
            Rectangle boundingBox = quadrilateral.BoundingBox;
            return CurveIntersectsRectangleNode(quadrilateral, ref boundingBox, rectangleNode, polylineToIgnore);

        }

        private static bool CurveIntersectsRectangleNode(ICurve curve, ref Rectangle curveBox, RectangleNode<Polyline, Point> rectNode, Polyline polylineToIgnore) {
            if (!rectNode.Rectangle.Intersects(curveBox)) {
                return false;
            }

            if (rectNode.UserData != null) {
                return rectNode.UserData != polylineToIgnore &&
                       (Curve.CurveCurveIntersectionOne(rectNode.UserData, curve, false) != null ||
                        Inside(rectNode.UserData, curve));
            }

            Debug.Assert(rectNode.Left != null && rectNode.Right != null);

            return CurveIntersectsRectangleNode(curve, ref curveBox, rectNode.Left, polylineToIgnore) ||
                   CurveIntersectsRectangleNode(curve, ref curveBox, rectNode.Right, polylineToIgnore);
        }

        private class PolylineGraph
        {
            private Dictionary<Polyline, List<Polyline>> sourceToTargets = new Dictionary<Polyline, List<Polyline>>();

            internal IEnumerable<Polyline> Nodes { get { return this.sourceToTargets.Keys; } }

            internal void AddEdge(Polyline source, Polyline target)
            {
                List<Polyline> listOfEdges;
                if (!this.sourceToTargets.TryGetValue(source, out listOfEdges))
                {
                    this.sourceToTargets[source] = listOfEdges = new List<Polyline>();
                }

                listOfEdges.Add(target);
            }

            internal IEnumerable<Polyline> Descendents(Polyline Polyline)
            {
                return this.sourceToTargets[Polyline];
            }
        }
    }
}