using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
#if TEST_MSAGL
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
#endif
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    /// <summary>
    /// sweeps a given direction of cones and adds discovered edges to the graph
    /// </summary>
    internal class LineSweeper : LineSweeperBase, IConeSweeper {
        public Point ConeRightSideDirection { get; set; }
        public Point ConeLeftSideDirection { get; set; }

        private readonly ConeSideComparer coneSideComparer;
        private readonly VisibilityGraph visibilityGraph;
        private readonly RbTree<ConeSide> rightConeSides;
        private readonly RbTree<ConeSide> leftConeSides;
        private VisibilityGraph portEdgesGraph;
        internal Func<VisibilityVertex, VisibilityVertex, VisibilityEdge> PortEdgesCreator { get; set; }

        private LineSweeper(IEnumerable<Polyline> obstacles, Point direction, Point coneRsDir, Point coneLsDir,
                    VisibilityGraph visibilityGraph, Set<Point> ports, Polyline borderPolyline)
            : base(obstacles, direction) {
            this.visibilityGraph = visibilityGraph;
            this.ConeRightSideDirection = coneRsDir;
            this.ConeLeftSideDirection = coneLsDir;
            this.coneSideComparer = new ConeSideComparer(this);
            this.leftConeSides = new RbTree<ConeSide>(this.coneSideComparer);
            this.rightConeSides = new RbTree<ConeSide>(this.coneSideComparer);
            this.Ports = ports;
            this.BorderPolyline = borderPolyline;
            this.PortEdgesCreator = (a, b) => new TollFreeVisibilityEdge(a, b);
        }

        private Polyline BorderPolyline { get; set; }

       
        internal static void Sweep(
            IEnumerable<Polyline> obstacles,
            Point direction,
            double coneAngle,
            VisibilityGraph visibilityGraph,
            Set<Point> ports,
            Polyline borderPolyline) {
         
            var cs = new LineSweeper(obstacles, direction, direction.Rotate(-coneAngle/2),
                                     direction.Rotate(coneAngle/2), visibilityGraph, ports,
                                     borderPolyline);
            cs.Calculate();
        }

        private void Calculate() {
            this.InitQueueOfEvents();
            while (this.EventQueue.Count > 0) {
                this.ProcessEvent(this.EventQueue.Dequeue());
            }

            if (this.BorderPolyline != null) {
                this.CloseRemainingCones();
            }

            this.CreatePortEdges();

        }

        private void CreatePortEdges() {
            if (this.portEdgesGraph != null) {
                foreach (var edge in this.portEdgesGraph.Edges) {
                    this.visibilityGraph.AddEdge(edge.SourcePoint, edge.TargetPoint, this.PortEdgesCreator);
                }
            }
        }

        private void CloseRemainingCones() {
            if (this.leftConeSides.Count == 0) {
                return;
            }

            Debug.Assert(this.leftConeSides.Count == this.rightConeSides.Count);

            PolylinePoint p = this.BorderPolyline.StartPoint;
            var steps= this.leftConeSides.Count; //we cannot make more than leftConeSides.Count if the data is correct
            //because at each step we remove at least one cone
            do {
                var cone = this.leftConeSides.TreeMinimum().Item.Cone;
                p = this.FindPolylineSideIntersectingConeRightSide(p, cone);
                p = this.GetPolylinePointInsideOfConeAndRemoveCones(p, cone);
                steps--;
            } while (this.leftConeSides.Count > 0 && steps>0);
        }

        private PolylinePoint GetPolylinePointInsideOfConeAndRemoveCones(PolylinePoint p, Cone cone) {
            var pn = p.NextOnPolyline;
            Point insidePoint = FindInsidePoint(p.Point, pn.Point, cone);

            if (ApproximateComparer.Close(insidePoint, p.Point)) {
                this.AddEdgeAndRemoveCone(cone, p.Point);
                this.AddEdgesAndRemoveRemainingConesByPoint(p.Point);
                //we don't move p forward here. In the next iteration we just cross [p,pn] with the new leftmost cone right side
            } else if (ApproximateComparer.Close(insidePoint, pn.Point)) {
                this.AddEdgeAndRemoveCone(cone, pn.Point);
                this.AddEdgesAndRemoveRemainingConesByPoint(pn.Point);
                p = pn;
            } else {
                p = InsertPointIntoPolylineAfter(this.BorderPolyline, p, insidePoint);
                this.AddEdgeAndRemoveCone(cone, p.Point);
                this.AddEdgesAndRemoveRemainingConesByPoint(p.Point);
            }
            return p;
        }

        private static Point FindInsidePoint(Point leftPoint, Point rightPoint, Cone cone) {
            //            if (debug)
            //                LayoutAlgorithmSettings.Show(CurveFactory.CreateCircle(3, leftPoint),
            //                                             CurveFactory.CreateDiamond(3, 3, rightPoint),
            //                                             BorderPolyline, ExtendSegmentToZ(cone.LeftSide),
            //                                             ExtendSegmentToZ(cone.RightSide));
            return FindInsidePointBool(leftPoint, rightPoint, cone.Apex, cone.Apex + cone.LeftSideDirection,
                                       cone.Apex + cone.RightSideDirection);
        }

        private static Point FindInsidePointBool(Point leftPoint, Point rightPoint, Point apex, Point leftSideConePoint,
                                         Point rightSideConePoint) {
            if (ApproximateComparer.Close(leftPoint, rightPoint)) {
                return leftPoint; //does not matter which one to return
            }

            if (Point.PointIsInsideCone(leftPoint, apex, leftSideConePoint, rightSideConePoint)) {
                return leftPoint;
            }

            if (Point.PointIsInsideCone(rightPoint, apex, leftSideConePoint, rightSideConePoint)) {
                return rightPoint;
            }

            var m = 0.5 * (leftPoint + rightPoint);

            if (Point.PointToTheLeftOfLine(m, apex, leftSideConePoint)) {
                return FindInsidePointBool(m, rightPoint, apex, leftSideConePoint, rightSideConePoint);
            }

            return FindInsidePointBool(leftPoint, m, apex, leftSideConePoint, rightSideConePoint);
        }

        private void AddEdgesAndRemoveRemainingConesByPoint(Point point) {
            var conesToRemove = new List<Cone>();
            foreach (var leftConeSide in this.leftConeSides) {
                if (Point.PointToTheRightOfLineOrOnLine(point, leftConeSide.Start,
                                                        leftConeSide.Start + leftConeSide.Direction)) {
                    conesToRemove.Add(leftConeSide.Cone);
                } else {
                    break;
                }
            }
            foreach (var cone in conesToRemove) {
                this.AddEdgeAndRemoveCone(cone, point);
            }
        }

        private PolylinePoint FindPolylineSideIntersectingConeRightSide(PolylinePoint p, Cone cone) {
            var startPoint = p;
            var a = cone.Apex;
            var b = cone.Apex + this.ConeRightSideDirection;
            var pSign = GetSign(p, a, b);
            do {
                var pn = p.NextOnPolyline;
                var pnSigh = GetSign(pn, a, b);
                if (pnSigh - pSign > 0) {
                    return p;
                }

                p = pn;
                pSign = pnSigh;
                if (p == startPoint) {
                    throw new InvalidOperationException();
                }
            } while (true);
            /*
                        throw new InvalidOleVariantTypeException();
            */
        }

        private static int GetSign(PolylinePoint p, Point a, Point b) {
            var d = Point.SignedDoubledTriangleArea(a, b, p.Point);
            if (d < 0) {
                return 1;
            }

            return d > 0 ? -1 : 0;
        }

#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void Showside(PolylinePoint p, Point a, Point b, PolylinePoint pn) {
            this.ShowBothTrees(new DebugCurve(100, 1, "brown", this.BorderPolyline), new DebugCurve(100, 2, "blue",
                                                                                          new LineSegment(a, b)),
                          new DebugCurve(100, 2, "green",
                                         new LineSegment(
                                             pn.Point, p.Point)
                              ));
        }
#endif

        //        void CheckThatPolylineIsLegal()
        //        {
        //            var p = BorderPolyline.StartPoint;
        //            do
        //            {
        //                var pn = p.NextOnPolyline;
        //                Debug.Assert(!ApproximateComparer.Close(p.Point, pn.Point));
        //                Debug.Assert((pn.Point - p.Point)*(pn.NextOnPolyline.Point - pn.Point) > -ApproximateComparer.Tolerance);
        //                p = pn;
        //            } while (p != BorderPolyline.StartPoint);
        //        }

#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowBoundaryPolyline() {
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(this.CreateBoundaryPolyDebugCurves());
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private IEnumerable<DebugCurve> CreateBoundaryPolyDebugCurves() {
            int i = 0;
            for (var p = this.BorderPolyline.StartPoint; p != null; p = p.Next) {
                yield return new DebugCurve(new Ellipse(1, 1, p.Point), i++);
            }
        }
#endif

        private void AddEdgeAndRemoveCone(Cone cone, Point p) {
            if (this.Ports != null && this.Ports.Contains(cone.Apex)) {
                this.CreatePortEdge(cone, p);
            } else {
                this.visibilityGraph.AddEdge(cone.Apex, p);
            }

            this.RemoveCone(cone);
        }

        /*********************
            A complication arises when we have overlaps. Loose obstacles become large enough to contain several
            ports. We need to avoid a situation when a port has degree more than one. 
            To avoid this situation we redirect to p every edge incoming into cone.Apex. 
            Notice that we create a new graph to keep port edges for ever 
            direction of the sweep and the above procedure just alignes the edges better.
            In the resulting graph, which contains the sum of the graphs passed to AddDirection, of course
            a port can have an incoming and outcoming edge at the same time
            *******************/

        private void CreatePortEdge(Cone cone, Point p) {
            if (this.portEdgesGraph == null) {
                this.portEdgesGraph = new VisibilityGraph();
            }

            var coneApexVert = this.portEdgesGraph.FindVertex(cone.Apex);
            //all previous edges adjacent to cone.Apex 
            var edgesToFix = (coneApexVert != null)
                                 ? coneApexVert.InEdges.Concat(coneApexVert.OutEdges).ToArray()
                                 : null;
            if (edgesToFix != null) {
                foreach (var edge in edgesToFix) {
                    var otherPort = (edge.Target == coneApexVert ? edge.Source : edge.Target).Point;
                    VisibilityGraph.RemoveEdge(edge);
                    this.portEdgesGraph.AddEdge(otherPort, p);
                }
            }

            this.portEdgesGraph.AddEdge(cone.Apex, p);
        }


        internal static PolylinePoint InsertPointIntoPolylineAfter(Polyline borderPolyline, PolylinePoint insertAfter,
                                                                   Point pointToInsert) {
            PolylinePoint np;
            if (insertAfter.Next != null) {
                np = new PolylinePoint(pointToInsert) { Prev = insertAfter, Next = insertAfter.Next, Polyline = borderPolyline };
                insertAfter.Next.Prev = np;
                insertAfter.Next = np;
            } else {
                np = new PolylinePoint(pointToInsert) { Prev = insertAfter, Polyline = borderPolyline };
                insertAfter.Next = np;
                borderPolyline.EndPoint = np;
            }

            Debug.Assert(
                !(ApproximateComparer.Close(np.Point, np.PrevOnPolyline.Point) ||
                  ApproximateComparer.Close(np.Point, np.NextOnPolyline.Point)));

            borderPolyline.RequireInit();
            return np;
        }

        private void ProcessEvent(SweepEvent p) {
            var vertexEvent = p as VertexEvent;
           

            if (vertexEvent != null) {
                this.ProcessVertexEvent(vertexEvent);
            } else {
                var rightIntersectionEvent = p as RightIntersectionEvent;
                if (rightIntersectionEvent != null) {
                    this.ProcessRightIntersectionEvent(rightIntersectionEvent);
                } else {
                    var leftIntersectionEvent = p as LeftIntersectionEvent;
                    if (leftIntersectionEvent != null) {
                        this.ProcessLeftIntersectionEvent(leftIntersectionEvent);
                    } else {
                        var coneClosure = p as ConeClosureEvent;
                        if (coneClosure != null) {
                            if (!coneClosure.ConeToClose.Removed) {
                                this.RemoveCone(coneClosure.ConeToClose);
                            }
                        } else {
                            this.ProcessPortObstacleEvent((PortObstacleEvent)p);
                        }

                        this.Z = this.GetZ(p);
                    }
                }
            }
            //Debug.Assert(TreesAreCorrect());
        }
#if TEST_MSAGL
        //        protected override bool TreesAreCorrect() {
        //            return TreeIsCorrect(leftConeSides) && TreeIsCorrect(rightConeSides);
        //        }
        //
        //        bool TreeIsCorrect(RbTree<ConeSide> tree) {
        //            var y = double.NegativeInfinity;
        //            foreach (var t in tree) {
        //                var x = coneSideComparer.IntersectionOfSegmentAndSweepLine(t);
        //                var yp = x*DirectionPerp;
        //                if (yp < y - ApproximateComparer.DistanceEpsilon)
        //                    return false;
        //                y = yp;
        //            }
        //            return true;
        //        }
#endif
        private void ProcessPortObstacleEvent(PortObstacleEvent portObstacleEvent) {
            this.Z = this.GetZ(portObstacleEvent);
            this.GoOverConesSeeingVertexEvent(portObstacleEvent);
            this.CreateConeOnVertex(portObstacleEvent);
        }

        private void ProcessLeftIntersectionEvent(LeftIntersectionEvent leftIntersectionEvent) {
            if (leftIntersectionEvent.coneLeftSide.Removed == false) {
                if (Math.Abs((leftIntersectionEvent.EndVertex.Point - leftIntersectionEvent.Site) * this.SweepDirection) <
                    ApproximateComparer.DistanceEpsilon) {
                    //the cone is totally covered by a horizontal segment
                    this.RemoveCone(leftIntersectionEvent.coneLeftSide.Cone);
                } else {
                    this.RemoveSegFromLeftTree(leftIntersectionEvent.coneLeftSide);
                    this.Z = this.SweepDirection * leftIntersectionEvent.Site; //it is safe now to restore the order
                    var leftSide = new BrokenConeSide(
                        leftIntersectionEvent.Site,
                        leftIntersectionEvent.EndVertex, leftIntersectionEvent.coneLeftSide);
                    this.InsertToTree(this.leftConeSides, leftSide);
                    leftIntersectionEvent.coneLeftSide.Cone.LeftSide = leftSide;
                    this.LookForIntersectionOfObstacleSideAndLeftConeSide(leftIntersectionEvent.Site,
                                                                     leftIntersectionEvent.EndVertex);
                    this.TryCreateConeClosureForLeftSide(leftSide);
                }
            } else {
                this.Z = this.SweepDirection * leftIntersectionEvent.Site;
            }
        }

        private void TryCreateConeClosureForLeftSide(BrokenConeSide leftSide) {
            var coneRightSide = leftSide.Cone.RightSide as ConeRightSide;
            if (coneRightSide != null) {
                if (
                    Point.GetTriangleOrientation(coneRightSide.Start, coneRightSide.Start + coneRightSide.Direction,
                                                 leftSide.EndVertex.Point) == TriangleOrientation.Clockwise) {
                    this.CreateConeClosureEvent(leftSide, coneRightSide);
                }
            }
        }

        private void CreateConeClosureEvent(BrokenConeSide brokenConeSide, ConeSide otherSide) {
            Point x;
#if TEST_MSAGL
            var r =
#endif
 Point.RayIntersectsRayInteriors(brokenConeSide.start, brokenConeSide.Direction, otherSide.Start,
                                                otherSide.Direction, out x);
#if TEST_MSAGL
            if (!r) {
                LayoutAlgorithmSettings.ShowDebugCurves(
                    new DebugCurve(100, 0.1, "red",new LineSegment(brokenConeSide.Start, brokenConeSide.start + brokenConeSide.Direction)),
                    new DebugCurve(100,0.1, "black", new Ellipse(0.1,0.1, brokenConeSide.Start)),
                    new DebugCurve(100, 0.1, "blue",new LineSegment(otherSide.Start, otherSide.Start + otherSide.Direction)));
            }

            Debug.Assert(r);
#endif
            this.EnqueueEvent(new ConeClosureEvent(x, brokenConeSide.Cone));
        }

        private void ProcessRightIntersectionEvent(RightIntersectionEvent rightIntersectionEvent) {
            //restore Z for the time being
            // Z = PreviousZ;
            if (rightIntersectionEvent.coneRightSide.Removed == false) {
                //it can happen that the cone side participating in the intersection is gone;
                //obstracted by another obstacle or because of a vertex found inside of the cone
                //PrintOutRightSegTree();
                this.RemoveSegFromRightTree(rightIntersectionEvent.coneRightSide);
                this.Z = this.SweepDirection * rightIntersectionEvent.Site;
                var rightSide = new BrokenConeSide(
                    rightIntersectionEvent.Site,
                    rightIntersectionEvent.EndVertex, rightIntersectionEvent.coneRightSide);
                this.InsertToTree(this.rightConeSides, rightSide);
                rightIntersectionEvent.coneRightSide.Cone.RightSide = rightSide;
                this.LookForIntersectionOfObstacleSideAndRightConeSide(rightIntersectionEvent.Site,
                                                                  rightIntersectionEvent.EndVertex);

                this.TryCreateConeClosureForRightSide(rightSide);
            } else {
                this.Z = this.SweepDirection * rightIntersectionEvent.Site;
            }
        }

        private void TryCreateConeClosureForRightSide(BrokenConeSide rightSide) {
            var coneLeftSide = rightSide.Cone.LeftSide as ConeLeftSide;
            if (coneLeftSide != null) {
                if (
                    Point.GetTriangleOrientation(coneLeftSide.Start, coneLeftSide.Start + coneLeftSide.Direction,
                                                 rightSide.EndVertex.Point) == TriangleOrientation.Counterclockwise) {
                    this.CreateConeClosureEvent(rightSide, coneLeftSide);
                }
            }
        }

        private void RemoveConesClosedBySegment(Point leftPoint, Point rightPoint) {
            this.CloseConesCoveredBySegment(leftPoint, rightPoint,
                                       this.SweepDirection * leftPoint > this.SweepDirection * rightPoint
                                           ? this.leftConeSides
                                           : this.rightConeSides);
        }

        private void CloseConesCoveredBySegment(Point leftPoint, Point rightPoint, RbTree<ConeSide> tree) {
            var node = tree.FindFirst(
                s => Point.GetTriangleOrientation(s.Start, s.Start + s.Direction, leftPoint) ==
                     TriangleOrientation.Counterclockwise);

            Point x;
            if (node == null || !Point.IntervalIntersectsRay(leftPoint, rightPoint,
                                                             node.Item.Start, node.Item.Direction, out x)) {
                return;
            }

            var conesToRemove = new List<Cone>();
            do {
                conesToRemove.Add(node.Item.Cone);
                node = tree.Next(node);
            } while (node != null && Point.IntervalIntersectsRay(leftPoint, rightPoint,
                                                                 node.Item.Start, node.Item.Direction, out x));


            foreach (var cone in conesToRemove) {
                this.RemoveCone(cone);
            }
        }

        private void ProcessVertexEvent(VertexEvent vertexEvent) {
            this.Z = this.GetZ(vertexEvent);
            this.GoOverConesSeeingVertexEvent(vertexEvent);
            this.AddConeAndEnqueueEvents(vertexEvent);
        }

#if TEST_MSAGL
        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static Ellipse EllipseOnVert(SweepEvent vertexEvent) {
            // ReSharper restore UnusedMember.Local
            return new Ellipse(5, 5, vertexEvent.Site);
        }

        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static Ellipse EllipseOnPolylinePoint(PolylinePoint pp) {
            // ReSharper restore UnusedMember.Local
            return EllipseOnPolylinePoint(pp, 5);
        }
        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static Ellipse EllipseOnPolylinePoint(PolylinePoint pp, double i)
            // ReSharper restore UnusedMember.Local
        {
            return new Ellipse(i, i, pp.Point);
        }

        private static ICurve Diamond(Point p) {
            return CurveFactory.CreateDiamond(2, 2, p);
        }

        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization",
            "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)"
            ),
         System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void CheckConsistency() {
            // ReSharper restore UnusedMember.Local
            foreach (var s in this.rightConeSides) {
                this.coneSideComparer. SetOperand(s);
            }
            foreach (var s in this.leftConeSides) {
                this.coneSideComparer.SetOperand(s);
                if (!this.rightConeSides.Contains(s.Cone.RightSide)) {
                    this.PrintOutRightSegTree();
                    this.PrintOutLeftSegTree();

                    this.ShowLeftTree();
                    this.ShowRightTree();
                }
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowRightTree(params ICurve[] curves) {
            var l = this.Obstacles.Select(p => new DebugCurve(100, 5, "green", p)).ToList();
            l.AddRange(this.rightConeSides.Select(s => new DebugCurve(100, 5, "blue", this.ExtendSegmentToZ(s))));

            //            foreach (VisibilityEdge edge in visibilityGraph.Edges)
            //                l.Add(BezierOnEdge(edge));

            l.AddRange(curves.Select(c => new DebugCurve(100, 5, "brown", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }


        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA1801:ReviewUnusedParameters", MessageId = "curves"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowBothTrees(params DebugCurve[] curves) {
            var l = this.Obstacles.Select(p => new DebugCurve(100, 5, "green", p)).ToList();
            l.AddRange(this.leftConeSides.Select(s => new DebugCurve(this.ExtendSegmentToZ(s))));
            l .AddRange(this.rightConeSides.Select(s => new DebugCurve(this.ExtendSegmentToZ(s))));

            //            foreach (VisibilityEdge edge in visibilityGraph.Edges)
            //                l.Add(BezierOnEdge(edge));

            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }

        private void ShowLeftTree(params ICurve[] curves) {
            var l = this.Obstacles.Select(p => new DebugCurve(100, 0.01,"green", p)).ToList();
            var range = new RealNumberSpan();
            var ellipseSize = 0.01;

            foreach (var s in this.leftConeSides) {
                var curve = this.ExtendSegmentToZ(s);
                range.AddValue(curve.Start* this.DirectionPerp);
                range.AddValue(curve.End * this.DirectionPerp);
                l.Add(new DebugCurve(100, 0.1, "red", curve));
                l.Add(new DebugCurve(200,0.1, "black", new Ellipse(ellipseSize,ellipseSize, curve.End)));
                ellipseSize += 2;
            }
            l.Add(this.DebugSweepLine(range));

            //            foreach (VisibilityEdge edge in visibilityGraph.Edges)
            //                l.Add(BezierOnEdge(edge));

            l.AddRange(curves.Select(c => new DebugCurve(100, 0.5, "brown", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }

        private DebugCurve DebugSweepLine(RealNumberSpan range) {
            var ls = new LineSegment(this.Z * this.SweepDirection + this.DirectionPerp * range.Min, this.Z * this.SweepDirection + this.DirectionPerp * range.Max);
            return new DebugCurve(100,0.1,"magenta", ls);
        }
#endif

        private void AddConeAndEnqueueEvents(VertexEvent vertexEvent) {
            var leftVertexEvent = vertexEvent as LeftVertexEvent;
            if (leftVertexEvent != null) {
                PolylinePoint nextPoint = vertexEvent.Vertex.NextOnPolyline;
                this.CloseConesAddConeAtLeftVertex(leftVertexEvent, nextPoint);
            } else {
                var rightVertexEvent = vertexEvent as RightVertexEvent;
                if (rightVertexEvent != null) {
                    PolylinePoint nextPoint = vertexEvent.Vertex.PrevOnPolyline;
                    this.CloseConesAddConeAtRightVertex(rightVertexEvent, nextPoint);
                } else {
                    this.CloseConesAddConeAtLeftVertex(vertexEvent, vertexEvent.Vertex.NextOnPolyline);
                    this.CloseConesAddConeAtRightVertex(vertexEvent, vertexEvent.Vertex.PrevOnPolyline);
                }
            }
        }

        private void CloseConesAddConeAtRightVertex(VertexEvent rightVertexEvent,
                                            PolylinePoint nextVertex) {
            var prevSite = rightVertexEvent.Vertex.NextOnPolyline.Point;
            var prevZ = prevSite* this.SweepDirection;
            if (ApproximateComparer.Close(prevZ, this.Z)) {
                this.RemoveConesClosedBySegment(prevSite, rightVertexEvent.Vertex.Point);
            }

            var site = rightVertexEvent.Site;
            var coneLp = site + this.ConeLeftSideDirection;
            var coneRp = site + this.ConeRightSideDirection;
            var nextSite = nextVertex.Point;
            //SugiyamaLayoutSettings.Show(new LineSegment(site, coneLP), new LineSegment(site, coneRP), new LineSegment(site, nextSite));
            //try to remove the right side
            if ((site - prevSite)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.RemoveRightSide(new RightObstacleSide(rightVertexEvent.Vertex.NextOnPolyline));
            }

            if ((site - nextVertex.Point) * this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.RemoveLeftSide(new LeftObstacleSide(nextVertex));
            }

            if (this.GetZ(nextSite) + ApproximateComparer.DistanceEpsilon < this.GetZ(rightVertexEvent)) {
                this.CreateConeOnVertex(rightVertexEvent);
            }

            if (!Point.PointToTheRightOfLineOrOnLine(nextSite, site, coneLp)) {
                //if (angle <= -coneAngle / 2) {
                this.CreateConeOnVertex(rightVertexEvent);
                if (Point.PointToTheLeftOfLineOrOnLine(nextSite + this.DirectionPerp, nextSite, site)) {
                    this.EnqueueEvent(new RightVertexEvent(nextVertex));
                }
                //  TryEnqueueRighVertexEvent(nextVertex);
            } else if (Point.PointToTheLeftOfLineOrOnLine(nextSite, site, coneRp)) {
                //if (angle < coneAngle / 2) {
                this.CaseToTheLeftOfLineOrOnLineConeRp(rightVertexEvent, nextVertex);
            } else {
                if ((nextSite - site)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                    this.LookForIntersectionOfObstacleSideAndLeftConeSide(rightVertexEvent.Site, nextVertex);
                    this.InsertRightSide(new RightObstacleSide(rightVertexEvent.Vertex));
                }
                this.EnqueueEvent(new RightVertexEvent(nextVertex));
            }
        }

        private void CaseToTheLeftOfLineOrOnLineConeRp(VertexEvent rightVertexEvent, PolylinePoint nextVertex) {
            this.EnqueueEvent(new RightVertexEvent(nextVertex));
            //the obstacle side is inside of the cone
            //we need to create an obstacle left side segment instead of the left cone side
            var cone = new Cone(rightVertexEvent.Vertex.Point, this);
            var obstacleSideSeg = new BrokenConeSide(cone.Apex, nextVertex, new ConeLeftSide(cone));
            cone.LeftSide = obstacleSideSeg;
            cone.RightSide = new ConeRightSide(cone);
            var rnode = this.InsertToTree(this.rightConeSides, cone.RightSide);
            this.LookForIntersectionWithConeRightSide(rnode);
            var lnode = this.InsertToTree(this.leftConeSides, cone.LeftSide);
            this.FixConeLeftSideIntersections(obstacleSideSeg, lnode);
            if ((nextVertex.Point - rightVertexEvent.Site) * this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.InsertRightSide(new RightObstacleSide(rightVertexEvent.Vertex));
            }
        }

        private void LookForIntersectionOfObstacleSideAndRightConeSide(Point obstacleSideStart,
                                                               PolylinePoint obstacleSideVertex) {
            RBNode<ConeSide> node = this.GetLastNodeToTheLeftOfPointInRightSegmentTree(obstacleSideStart);

            if (node != null) {
                var coneRightSide = node.Item as ConeRightSide;
                if (coneRightSide != null) {
                    Point intersection;
                    if (Point.IntervalIntersectsRay(obstacleSideStart, obstacleSideVertex.Point,
                                                    coneRightSide.Start, this.ConeRightSideDirection, out intersection) &&
                        this.SegmentIsNotHorizontal(intersection, obstacleSideVertex.Point)) {
                        this.EnqueueEvent(this.CreateRightIntersectionEvent(coneRightSide, intersection, obstacleSideVertex));
                    }
                }
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        private RightIntersectionEvent CreateRightIntersectionEvent(ConeRightSide coneRightSide, Point intersection,
                                                            PolylinePoint obstacleSideVertex) {
            Debug.Assert(Math.Abs((obstacleSideVertex.Point - intersection) * this.SweepDirection) > 0);
            return new RightIntersectionEvent(coneRightSide,
                                              intersection, obstacleSideVertex);
        }

        private RBNode<ConeSide> GetLastNodeToTheLeftOfPointInRightSegmentTree(Point obstacleSideStart) {
            return this.rightConeSides.FindLast(
                s => PointIsToTheRightOfSegment(obstacleSideStart, s));
        }

        private void LookForIntersectionOfObstacleSideAndLeftConeSide(Point obstacleSideStart,
                                                              PolylinePoint obstacleSideVertex) {
            var node = this.GetFirstNodeToTheRightOfPoint(obstacleSideStart);
            //          ShowLeftTree(Box(obstacleSideStart));
            if (node == null) {
                return;
            }

            var coneLeftSide = node.Item as ConeLeftSide;
            if (coneLeftSide == null) {
                return;
            }

            Point intersection;
            if (Point.IntervalIntersectsRay(obstacleSideStart, obstacleSideVertex.Point, coneLeftSide.Start,
                                            this.ConeLeftSideDirection, out intersection)) {
                this.EnqueueEvent(new LeftIntersectionEvent(coneLeftSide, intersection, obstacleSideVertex));
            }
        }

        private RBNode<ConeSide> GetFirstNodeToTheRightOfPoint(Point p) {
            return this.leftConeSides.FindFirst(s => PointIsToTheLeftOfSegment(p, s));
        }

#if TEST_MSAGL
        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static ICurve Box(Point p) {
            // ReSharper restore UnusedMember.Local
            return CurveFactory.CreateRectangle(2, 2, p);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization",
            "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)"
            )]
        private void PrintOutRightSegTree() {
            System.Diagnostics.Debug.WriteLine("right segment tree");
            foreach (var t in this.rightConeSides) {
                System.Diagnostics.Debug.WriteLine(t);
            }

            System.Diagnostics.Debug.WriteLine("end of right segments");
        }
#endif

        private static bool PointIsToTheLeftOfSegment(Point p, ConeSide seg) {
            return (Point.GetTriangleOrientation(seg.Start, seg.Start + seg.Direction, p) ==
                    TriangleOrientation.Counterclockwise);
        }

        private static bool PointIsToTheRightOfSegment(Point p, ConeSide seg) {
            return (Point.GetTriangleOrientation(seg.Start, seg.Start + seg.Direction, p) ==
                    TriangleOrientation.Clockwise);
        }

        private void FixConeLeftSideIntersections(BrokenConeSide leftSide, RBNode<ConeSide> rbNode) {
            //the first intersection can happen only with succesors of leftSide
            Debug.Assert(rbNode != null);
            do { //this loop usually works only once
                rbNode = this.leftConeSides.Next(rbNode);
            } while (rbNode != null &&  Point.PointToTheRightOfLineOrOnLine(leftSide.Start, rbNode.Item.Start, rbNode.Item.Start+rbNode.Item.Direction)); 

            if (rbNode != null) {
                Point intersection;
                var seg = rbNode.Item as ConeLeftSide;
                if (seg != null &&
                    Point.IntervalIntersectsRay(leftSide.Start, leftSide.End, seg.Start, seg.Direction, out intersection)) {
                    this.EnqueueEvent(new LeftIntersectionEvent(seg, intersection, leftSide.EndVertex));
                    //Show(CurveFactory.CreateDiamond(3, 3, intersection));
                }
            }
        }

        private RBNode<ConeSide> InsertToTree(RbTree<ConeSide> tree, ConeSide coneSide) {
            Debug.Assert(coneSide.Direction * this.SweepDirection > 0);
            this.coneSideComparer.SetOperand(coneSide);
            return tree.Insert(coneSide);
        }

        private void CloseConesAddConeAtLeftVertex(VertexEvent leftVertexEvent, PolylinePoint nextVertex) {
            //close segments first
            Point prevSite = leftVertexEvent.Vertex.PrevOnPolyline.Point;
            double prevZ = prevSite * this.SweepDirection;
            if (ApproximateComparer.Close(prevZ, this.Z) && (prevSite - leftVertexEvent.Site) * this.DirectionPerp > 0) {
                //Show(
                //    new Ellipse(1, 1, prevSite),
                //    CurveFactory.CreateBox(2, 2, leftVertexEvent.Vertex.Point));
                this.RemoveConesClosedBySegment(leftVertexEvent.Vertex.Point, prevSite);
            }

            var site = leftVertexEvent.Site;
            var coneLp = site + this.ConeLeftSideDirection;
            var coneRp = site + this.ConeRightSideDirection;
            var nextSite = nextVertex.Point;
            // SugiyamaLayoutSettings.Show(new LineSegment(site, coneLP), new LineSegment(site, coneRP), new LineSegment(site, nextSite));

            if ((site - prevSite) * this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.RemoveLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex.PrevOnPolyline));
            }

            var nextDelZ = this.GetZ(nextSite) - this.Z;
            if(nextDelZ<-ApproximateComparer.DistanceEpsilon) {
                this.RemoveRightSide(new RightObstacleSide(nextVertex));
            }

            if (nextDelZ < -ApproximateComparer.DistanceEpsilon ||
                ApproximateComparer.Close(nextDelZ, 0) && (nextSite - leftVertexEvent.Site) * this.DirectionPerp > 0) {
                //if (angle > Math.PI / 2)
                this.CreateConeOnVertex(leftVertexEvent); //it is the last left vertex on this obstacle
                
            } else if (!Point.PointToTheLeftOfLineOrOnLine(nextSite, site, coneRp)) {
                //if (angle >= coneAngle / 2) {
                this.CreateConeOnVertex(leftVertexEvent);
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                //we schedule LeftVertexEvent for a vertex with horizontal segment to the left on the top of the obstace
            } else if (!Point.PointToTheLeftOfLineOrOnLine(nextSite, site, coneLp)) {
                //if (angle >= -coneAngle / 2) {
                //we cannot completely obscure the cone here
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                //the obstacle side is inside of the cone
                //we need to create an obstacle right side segment instead of the cone side
                var cone = new Cone(leftVertexEvent.Vertex.Point, this);
                var rightSide = new BrokenConeSide(leftVertexEvent.Vertex.Point, nextVertex,
                                                   new ConeRightSide(cone));
                cone.RightSide = rightSide;
                cone.LeftSide = new ConeLeftSide(cone);
                this.LookForIntersectionWithConeLeftSide(this.InsertToTree(this.leftConeSides, cone.LeftSide));
                var rbNode = this.InsertToTree(this.rightConeSides, rightSide);
                this.FixConeRightSideIntersections(rightSide, rbNode);
                if ((nextVertex.Point - leftVertexEvent.Site) * this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                    this.InsertLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex));
                }
            } else {
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                if ((nextVertex.Point - leftVertexEvent.Site) * this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                    //if( angle >- Pi/2
                    // Debug.Assert(angle > -Math.PI / 2);
                    this.LookForIntersectionOfObstacleSideAndRightConeSide(leftVertexEvent.Site, nextVertex);
                    this.InsertLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex));
                }
            }
        }

        private void RemoveCone(Cone cone)
        {
            // the following should not happen if the containment hierarchy is correct.  
            // If containment is not correct it still should not result in a fatal error, just a funny looking route.
            // Debug.Assert(cone.Removed == false);
            cone.Removed = true;
            this.RemoveSegFromLeftTree(cone.LeftSide);
            this.RemoveSegFromRightTree(cone.RightSide);
        }

        private void RemoveSegFromRightTree(ConeSide coneSide) {
            //   ShowRightTree();
            Debug.Assert(coneSide.Removed == false);
            this.coneSideComparer.SetOperand(coneSide);
            var b = this.rightConeSides.Remove(coneSide);
            coneSide.Removed = true;
            if (b == null) {
                var tmpZ = this.Z;
                this.Z = Math.Max(this.GetZ(coneSide.Start), this.Z - 0.01);
                //we need to return to the past a little bit when the order was still correc
                this.coneSideComparer.SetOperand(coneSide);
                b = this.rightConeSides.Remove(coneSide);
                this.Z = tmpZ;

#if TEST_MSAGL
                if (b == null) {
                    this.PrintOutRightSegTree();
                }
#endif
            }
        }

        private void RemoveSegFromLeftTree(ConeSide coneSide) {
            Debug.Assert(coneSide.Removed == false);
            coneSide.Removed = true;
            this.coneSideComparer.SetOperand(coneSide);
            var b = this.leftConeSides.Remove(coneSide);

            if (b == null) {
                var tmpZ = this.Z;
                this.Z = Math.Max(this.GetZ(coneSide.Start), this.Z - 0.01);
                this.coneSideComparer.SetOperand(coneSide);
                b = this.leftConeSides.Remove(coneSide);
                this.Z = tmpZ;
#if TEST_MSAGL
                if (b == null) {
                    this.PrintOutLeftSegTree();
                    this.ShowLeftTree(new Ellipse(2, 2, coneSide.Start));
                }
#endif
            }

            Debug.Assert(b != null);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rightSide"></param>
        /// <param name="rbNode">represents a node of the right cone side</param>
        private void FixConeRightSideIntersections(BrokenConeSide rightSide, RBNode<ConeSide> rbNode) {
            //the first intersection can happen only with predecessors of rightSide
            Debug.Assert(rbNode != null);
            do { //this loop usually works only once
                rbNode = this.rightConeSides.Previous(rbNode);
            } while (rbNode != null && Point.PointToTheLeftOfLineOrOnLine(rightSide.Start, rbNode.Item.Start, rbNode.Item.Start + rbNode.Item.Direction));  
            if (rbNode != null) {
                Point intersection;
                var seg = rbNode.Item as ConeRightSide;
                if (seg != null &&
                    Point.IntervalIntersectsRay(rightSide.Start, rightSide.End, seg.Start, seg.Direction,
                                                out intersection)) {
                    this.EnqueueEvent(this.CreateRightIntersectionEvent(seg, intersection, rightSide.EndVertex));
                    // Show(CurveFactory.CreateDiamond(3, 3, intersection));
                }
            }
        }

        private void CreateConeOnVertex(SweepEvent sweepEvent) {
            var cone = new Cone(sweepEvent.Site, this);
#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=368
            //SharpKit/Colin - property assignment values not retained
            cone.LeftSide = new ConeLeftSide(cone);
            cone.RightSide = new ConeRightSide(cone);

            var leftNode = InsertToTree(leftConeSides, cone.LeftSide);
            var rightNode = InsertToTree(rightConeSides, cone.RightSide);
#else
            var leftNode = this.InsertToTree(this.leftConeSides, cone.LeftSide = new ConeLeftSide(cone));
            var rightNode = this.InsertToTree(this.rightConeSides, cone.RightSide = new ConeRightSide(cone));
#endif       
            this.LookForIntersectionWithConeRightSide(rightNode);
            this.LookForIntersectionWithConeLeftSide(leftNode);
        }

        private void LookForIntersectionWithConeLeftSide(RBNode<ConeSide> leftNode) {
            //Show(new Ellipse(1, 1, leftNode.item.Start));


            var coneLeftSide = leftNode.Item as ConeLeftSide;
            if (coneLeftSide != null) {
                //leftNode = leftSegmentTree.TreePredecessor(leftNode);
                //if (leftNode != null) {
                //    var seg = leftNode.item as ObstacleSideSegment;
                //    if (seg != null)
                //        TryIntersectionOfConeLeftSideAndObstacleConeSide(coneLeftSide, seg);
                //}

                RightObstacleSide rightObstacleSide = this.FindFirstObstacleSideToTheLeftOfPoint(coneLeftSide.Start);
                if (rightObstacleSide != null) {
                    this.TryIntersectionOfConeLeftSideAndObstacleSide(coneLeftSide, rightObstacleSide);
                }
            } else {
                var seg = (BrokenConeSide)leftNode.Item;
                leftNode = this.leftConeSides.Next(leftNode);
                if (leftNode != null) {
                    coneLeftSide = leftNode.Item as ConeLeftSide;
                    if (coneLeftSide != null) {
                        this.TryIntersectionOfConeLeftSideAndObstacleConeSide(coneLeftSide, seg);
                    }
                }
            }
        }

        private void LookForIntersectionWithConeRightSide(RBNode<ConeSide> rightNode) {
            //Show(new Ellipse(10, 5, rightNode.item.Start));
            var coneRightSide = rightNode.Item as ConeRightSide;
            if (coneRightSide != null) {
                //rightNode = rightSegmentTree.TreeSuccessor(rightNode);
                //if (rightNode != null) {
                //    var seg = rightNode.item as ObstacleSideSegment;
                //    if (seg != null)
                //        TryIntersectionOfConeRightSideAndObstacleConeSide(coneRightSide, seg);
                //}

                LeftObstacleSide leftObstacleSide = this.FindFirstObstacleSideToToTheRightOfPoint(coneRightSide.Start);
                if (leftObstacleSide != null) {
                    this.TryIntersectionOfConeRightSideAndObstacleSide(coneRightSide, leftObstacleSide);
                }
            } else {
                var seg = (BrokenConeSide)rightNode.Item;
                rightNode = this.rightConeSides.Previous(rightNode);
                if (rightNode != null) {
                    coneRightSide = rightNode.Item as ConeRightSide;
                    if (coneRightSide != null) {
                        this.TryIntersectionOfConeRightSideAndObstacleConeSide(coneRightSide, seg);
                    }
                }
            }
        }

        private void TryIntersectionOfConeRightSideAndObstacleConeSide(ConeRightSide coneRightSide,
                                                               BrokenConeSide seg) {
            Point x;
            if (Point.IntervalIntersectsRay(seg.Start, seg.End, coneRightSide.Start,
                                            coneRightSide.Direction, out x)) {
                this.EnqueueEvent(this.CreateRightIntersectionEvent(coneRightSide, x, seg.EndVertex));
                //Show(CurveFactory.CreateDiamond(3, 3, x));
            }
        }

        private void TryIntersectionOfConeRightSideAndObstacleSide(ConeRightSide coneRightSide, ObstacleSide side) {
            Point x;
            if (Point.IntervalIntersectsRay(side.Start, side.End, coneRightSide.Start,
                                            coneRightSide.Direction, out x)) {
                this.EnqueueEvent(this.CreateRightIntersectionEvent(coneRightSide, x, side.EndVertex));
                //Show(CurveFactory.CreateDiamond(3, 3, x));
            }
        }

        private void TryIntersectionOfConeLeftSideAndObstacleConeSide(ConeLeftSide coneLeftSide, BrokenConeSide seg) {
            Point x;
            if (Point.IntervalIntersectsRay(seg.Start, seg.End, coneLeftSide.Start, coneLeftSide.Direction, out x)) {
                this.EnqueueEvent(new LeftIntersectionEvent(coneLeftSide, x, seg.EndVertex));
                //Show(CurveFactory.CreateDiamond(3, 3, x));
            }
        }

        private void TryIntersectionOfConeLeftSideAndObstacleSide(ConeLeftSide coneLeftSide, ObstacleSide side) {
            Point x;
            if (Point.IntervalIntersectsRay(side.Start, side.End, coneLeftSide.Start, coneLeftSide.Direction, out x)) {
                this.EnqueueEvent(new LeftIntersectionEvent(coneLeftSide, x, side.EndVertex));
                //    Show(CurveFactory.CreateDiamond(3, 3, x));
            }
        }


#if TEST_MSAGL
        internal void Show(params ICurve[] curves) {
            var l = this.Obstacles.Select(o => new DebugCurve(100, 0.1, "blue", o)).ToList();

            foreach (var s in this.rightConeSides) {
                l.Add(new DebugCurve(0.5, "brown", this.ExtendSegmentToZ(s)));
                if (s is BrokenConeSide) {
                    l.Add(new DebugCurve("brown", Diamond(s.Start)));
                }

                l.Add(new DebugCurve(0.5, "green",
                                     this.ExtendSegmentToZ(s.Cone.LeftSide)));
                if (s.Cone.LeftSide is BrokenConeSide) {
                    l.Add(new DebugCurve("green", Diamond(s.Cone.LeftSide.Start)));
                }
            }

//            l.AddRange(
//                visibilityGraph.Edges.Select(
//                    edge => new DebugCurve(0.2, "maroon", new LineSegment(edge.SourcePoint, edge.TargetPoint))));

            l.AddRange(curves.Select(c => new DebugCurve("red", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static CubicBezierSegment BezierOnEdge(VisibilityEdge edge) {
            return new CubicBezierSegment(edge.SourcePoint, 2.0 / 3.0 * edge.SourcePoint + 1.0 / 3.0 * edge.TargetPoint,
                                          1.0 / 3.0 * edge.SourcePoint + 2.0 / 3.0 * edge.TargetPoint, edge.TargetPoint);
        }

        internal ICurve ExtendSegmentToZ(ConeSide segment) {
            double den = segment.Direction * this.SweepDirection;
            Debug.Assert(Math.Abs(den) > ApproximateComparer.DistanceEpsilon);
            double t = (this.Z + 40 - segment.Start * this.SweepDirection) / den;
            return new LineSegment(segment.Start, segment.Start + segment.Direction * t);
        }

        internal ICurve ExtendSegmentToZPlus1(ConeSide segment) {
            double den = segment.Direction * this.SweepDirection;
            Debug.Assert(Math.Abs(den) > ApproximateComparer.DistanceEpsilon);
            double t = (this.Z + 1 - segment.Start * this.SweepDirection) / den;

            return new LineSegment(segment.Start, segment.Start + segment.Direction * t);
        }
#endif

        //        static int count;
        private void GoOverConesSeeingVertexEvent(SweepEvent vertexEvent) {
            var rbNode = this.FindFirstSegmentInTheRightTreeNotToTheLeftOfVertex(vertexEvent);

            if (rbNode == null) {
                return;
            }

            var coneRightSide = rbNode.Item;
            var cone = coneRightSide.Cone;
            var leftConeSide = cone.LeftSide;
            if (VertexIsToTheLeftOfSegment(vertexEvent, leftConeSide)) {
                return;
            }

            var visibleCones = new List<Cone> { cone };
            this.coneSideComparer.SetOperand(leftConeSide);
            rbNode = this.leftConeSides.Find(leftConeSide);

            if (rbNode == null) {
                var tmpZ = this.Z;
                this.Z = Math.Max(this.GetZ(leftConeSide.Start), this.PreviousZ);
                //we need to return to the past a little bit when the order was still correct
                this.coneSideComparer.SetOperand(leftConeSide);
                rbNode = this.leftConeSides.Find(leftConeSide);
                this.Z = tmpZ;
#if TEST_MSAGL
//                if (rbNode == null) {
                    //GeometryGraph gg = CreateGraphFromObstacles();
                    //gg.Save("c:\\tmp\\bug");
//                    PrintOutLeftSegTree();
//                    System.Diagnostics.Debug.WriteLine(leftConeSide);
//                    ShowLeftTree(new Ellipse(3, 3, vertexEvent.Site));
//                    ShowRightTree(new Ellipse(3, 3, vertexEvent.Site));
//                }
#endif
            }
            // the following should not happen if the containment hierarchy is correct.  
            // If containment is not correct it still should not result in a fatal error, just a funny looking route.
            // Debug.Assert(rbNode!=null);

            if (rbNode == null) {//it is an emergency measure and should not happen                
                rbNode = this.GetRbNodeEmergency(rbNode, leftConeSide);
                if (rbNode == null) {
                    return; // the cone is not there! and it is a bug
                }
            }

            rbNode = this.leftConeSides.Next(rbNode);
            while (rbNode != null && !VertexIsToTheLeftOfSegment(vertexEvent, rbNode.Item)) {
                visibleCones.Add(rbNode.Item.Cone);
                rbNode = this.leftConeSides.Next(rbNode);
            }

            //Show(new Ellipse(1, 1, vertexEvent.Site));
            foreach (var visCone in visibleCones) {
                this.AddEdgeAndRemoveCone(visCone, vertexEvent.Site);
            }
        }

        private RBNode<ConeSide> GetRbNodeEmergency(RBNode<ConeSide> rbNode, ConeSide leftConeSide) {
            for (var node = this.leftConeSides.TreeMinimum(); node != null; node = this.leftConeSides.Next(node)) {
                if (node.Item == leftConeSide) {
                    rbNode = node;
                    break;
                }
            }

            return rbNode;
        }

#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider",
            MessageId = "System.Int32.ToString")]
        internal static GeometryGraph CreateGraphFromObstacles(IEnumerable<Polyline> obstacles) {
            var gg = new GeometryGraph();
            foreach (var ob in obstacles) {
                gg.Nodes.Add(new Node(ob.ToCurve()));
            }
            return gg;
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String,System.Object,System.Object)"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization",
            "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)"
            )]
        private void PrintOutLeftSegTree() {
            System.Diagnostics.Debug.WriteLine("Left cone segments########");
            foreach (var t in this.leftConeSides) {
                var x = this.coneSideComparer.IntersectionOfSegmentAndSweepLine(t);

                System.Diagnostics.Debug.WriteLine("{0} x={1}", t, x* this.DirectionPerp );                
            }
            System.Diagnostics.Debug.WriteLine("##########end of left cone segments");
        }
#endif

        private static bool VertexIsToTheLeftOfSegment(SweepEvent vertexEvent, ConeSide seg) {
            return (Point.GetTriangleOrientation(seg.Start, seg.Start + seg.Direction,
                                                 vertexEvent.Site) == TriangleOrientation.Counterclockwise);
        }

        private static bool VertexIsToTheRightOfSegment(SweepEvent vertexEvent, ConeSide seg) {
            return (Point.GetTriangleOrientation(seg.Start, seg.Start + seg.Direction,
                                                 vertexEvent.Site) == TriangleOrientation.Clockwise);
        }

        private RBNode<ConeSide> FindFirstSegmentInTheRightTreeNotToTheLeftOfVertex(SweepEvent vertexEvent) {
            return this.rightConeSides.FindFirst(
                s => !VertexIsToTheRightOfSegment(vertexEvent, s)
                );
        }

        private void EnqueueEvent(RightVertexEvent vertexEvent) {
            if (this.SweepDirection * (vertexEvent.Site - vertexEvent.Vertex.PrevOnPolyline.Point) > ApproximateComparer.Tolerance) {
                return;//otherwise we enqueue the vertex twice; once as a LeftVertexEvent and once as a RightVertexEvent
            }

            base.EnqueueEvent(vertexEvent);
        }

    }
}