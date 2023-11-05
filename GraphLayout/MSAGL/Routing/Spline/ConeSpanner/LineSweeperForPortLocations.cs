using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Routing.Visibility;
#if TEST_MSAGL
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.DebugHelpers.Persistence;
#endif

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    /// <summary>
    /// Sweeps a given direction of cones and adds discovered edges to the graph.
    /// The cones can only start at ports here.
    /// </summary>
    internal class LineSweeperForPortLocations : LineSweeperBase, IConeSweeper {
        public Point ConeRightSideDirection {
            get;
            set;
        }

        public Point ConeLeftSideDirection {
            get;
            set;
        }

        private readonly ConeSideComparer coneSideComparer;
        private readonly VisibilityGraph visibilityGraph;
        private readonly RbTree<ConeSide> rightConeSides;
        private readonly RbTree<ConeSide> leftConeSides;

        private LineSweeperForPortLocations(IEnumerable<Polyline> obstacles, Point direction, Point coneRsDir, Point coneLsDir,
                                    VisibilityGraph visibilityGraph, IEnumerable<Point> portLocations)
            : base(obstacles, direction) {
            this.visibilityGraph = visibilityGraph;
            this.ConeRightSideDirection = coneRsDir;
            this.ConeLeftSideDirection = coneLsDir;
            this.coneSideComparer = new ConeSideComparer(this);
            this.leftConeSides = new RbTree<ConeSide>(this.coneSideComparer);
            this.rightConeSides = new RbTree<ConeSide>(this.coneSideComparer);
            this.PortLocations = portLocations;
        }

        private IEnumerable<Point> PortLocations {
            get;
            set;
        }

        internal static void Sweep(IEnumerable<Polyline> obstacles,
                                   Point direction, double coneAngle, VisibilityGraph visibilityGraph,
                                   IEnumerable<Point> portLocations) {
            var cs = new LineSweeperForPortLocations(obstacles, direction, direction.Rotate(-coneAngle/2),
                                                     direction.Rotate(coneAngle/2), visibilityGraph, portLocations);
            cs.Calculate();
        }

        private void Calculate() {
            this.InitQueueOfEvents();
            foreach (Point portLocation in this.PortLocations) {
                this.EnqueueEvent(new PortLocationEvent(portLocation));
            }

            while (this.EventQueue.Count > 0) {
                this.ProcessEvent(this.EventQueue.Dequeue());
            }
        }

        private void ProcessEvent(SweepEvent p) {
            var vertexEvent = p as VertexEvent;
            // ShowTrees(CurveFactory.CreateDiamond(3, 3, p.Site));
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
                            var portLocationEvent = p as PortLocationEvent;
                            if (portLocationEvent != null) {
                                this.ProcessPortLocationEvent(portLocationEvent);
                            } else {
                                this.ProcessPointObstacleEvent((PortObstacleEvent) p);
                            }
                        }
                        this.Z = this.GetZ(p);
                    }
                }
            }
            //     ShowTrees(CurveFactory.CreateEllipse(3,3,p.Site));
        }

        private void ProcessPointObstacleEvent(PortObstacleEvent portObstacleEvent) {
            this.Z = this.GetZ(portObstacleEvent);
            this.GoOverConesSeeingVertexEvent(portObstacleEvent);
        }

        private void CreateConeOnPortLocation(SweepEvent sweepEvent) {
            var cone = new Cone(sweepEvent.Site, this);
            RBNode<ConeSide> leftNode = this.InsertToTree(this.leftConeSides, cone.LeftSide = new ConeLeftSide(cone));
            RBNode<ConeSide> rightNode = this.InsertToTree(this.rightConeSides, cone.RightSide = new ConeRightSide(cone));
            this.LookForIntersectionWithConeRightSide(rightNode);
            this.LookForIntersectionWithConeLeftSide(leftNode);
        }

        private void ProcessPortLocationEvent(PortLocationEvent portEvent) {
            this.Z = this.GetZ(portEvent);
            this.GoOverConesSeeingVertexEvent(portEvent);
            this.CreateConeOnPortLocation(portEvent);
        }

        private void ProcessLeftIntersectionEvent(LeftIntersectionEvent leftIntersectionEvent) {
            if (leftIntersectionEvent.coneLeftSide.Removed == false) {
                if (Math.Abs((leftIntersectionEvent.EndVertex.Point - leftIntersectionEvent.Site)* this.SweepDirection) <
                    ApproximateComparer.DistanceEpsilon) {
                    //the cone is totally covered by a horizontal segment
                    this.RemoveCone(leftIntersectionEvent.coneLeftSide.Cone);
                } else {
                    this.RemoveSegFromLeftTree(leftIntersectionEvent.coneLeftSide);
                    this.Z = this.SweepDirection *leftIntersectionEvent.Site; //it is safe now to restore the order
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
                this.Z = this.SweepDirection *leftIntersectionEvent.Site;
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
            bool r = Point.RayIntersectsRayInteriors(brokenConeSide.start, brokenConeSide.Direction, otherSide.Start,
                                                     otherSide.Direction, out x);
            Debug.Assert(r);
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
                this.Z = this.SweepDirection *rightIntersectionEvent.Site;
                var rightSide = new BrokenConeSide(
                    rightIntersectionEvent.Site,
                    rightIntersectionEvent.EndVertex, rightIntersectionEvent.coneRightSide);
                this.InsertToTree(this.rightConeSides, rightSide);
                rightIntersectionEvent.coneRightSide.Cone.RightSide = rightSide;
                this.LookForIntersectionOfObstacleSideAndRightConeSide(rightIntersectionEvent.Site,
                                                                  rightIntersectionEvent.EndVertex);

                this.TryCreateConeClosureForRightSide(rightSide);
            } else {
                this.Z = this.SweepDirection *rightIntersectionEvent.Site;
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
                                       this.SweepDirection *leftPoint > this.SweepDirection *rightPoint
                                           ? this.leftConeSides
                                           : this.rightConeSides);
        }

        private void CloseConesCoveredBySegment(Point leftPoint, Point rightPoint, RbTree<ConeSide> tree) {
            RBNode<ConeSide> node = tree.FindFirst(
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


            foreach (Cone cone in conesToRemove) {
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
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static Ellipse EllipseOnVert(SweepEvent vertexEvent) {
            // ReSharper restore UnusedMember.Local
            return new Ellipse(2, 2, vertexEvent.Site);
        }

        // ReSharper disable UnusedMember.Local
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static Ellipse EllipseOnPolylinePoint(PolylinePoint pp) {
            // ReSharper restore UnusedMember.Local
            return new Ellipse(2, 2, pp.Point);
        }

#endif


#if TEST_MSAGL
    // ReSharper disable UnusedMember.Local
        [SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void CheckConsistency() {
            // ReSharper restore UnusedMember.Local
            foreach (var s in this.rightConeSides) {
                this.coneSideComparer.SetOperand(s);
            }
            foreach (var s in this.leftConeSides) {
                this.coneSideComparer.SetOperand(s);
                if (!this.rightConeSides.Contains(s.Cone.RightSide)) {
                    this.PrintOutRightSegTree();
                    this.PrintOutLeftSegTree();
                }
            }
        }

        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowTrees(params ICurve[] curves) {
            // ReSharper restore UnusedMember.Local
            var l = this.Obstacles.Select(c => new DebugCurve(100, 1, "blue", c));
            l = l.Concat(this.rightConeSides.Select(s => new DebugCurve(200, 1, "brown", this.ExtendSegmentToZ(s))));
            l = l.Concat(this.leftConeSides.Select(s => new DebugCurve(200, 1, "gree", this.ExtendSegmentToZ(s))));
            l = l.Concat(curves.Select(c => new DebugCurve("red", c)));
            l =
                l.Concat(
                    this.visibilityGraph.Edges.Select(e => new LineSegment(e.SourcePoint, e.TargetPoint)).Select(
                        c => new DebugCurve("marine", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }

        private void ShowLeftTree(params ICurve[] curves) {
            var l = this.Obstacles.Select(c => new DebugCurve(c));
            l = l.Concat(this.leftConeSides.Select(s => new DebugCurve("brown", this.ExtendSegmentToZ(s))));
            l = l.Concat(curves.Select(c => new DebugCurve("red", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);

        }

        private void ShowRightTree(params ICurve[] curves) {
            var l = this.Obstacles.Select(c => new DebugCurve(c));
            l = l.Concat(this.rightConeSides.Select(s => new DebugCurve("brown", this.ExtendSegmentToZ(s))));
            l = l.Concat(curves.Select(c => new DebugCurve("red", c)));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }

        // ReSharper disable UnusedMember.Global
        internal void Show(params ICurve[] curves) {
            // ReSharper restore UnusedMember.Global
            var l = this.Obstacles.Select(c => new DebugCurve(100, 1, "black", c));

            l = l.Concat(curves.Select(c => new DebugCurve(200, 1, "red", c)));
            //            foreach (var s in rightConeSides){
            //                l.Add(ExtendSegmentToZ(s));
            //                if (s is BrokenConeSide)
            //                    l.Add(Diamond(s.Start));
            //                l.Add(ExtendSegmentToZ(s.Cone.LeftSide));
            //            }

            l =
                l.Concat(
                    this.visibilityGraph.Edges.Select(edge => new LineSegment(edge.SourcePoint, edge.TargetPoint)).Select(
                        c => new DebugCurve(100, 1, "blue", c)));


            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);

        }

        private ICurve ExtendSegmentToZ(ConeSide segment) {
            double den = segment.Direction * this.SweepDirection;
            Debug.Assert(Math.Abs(den) > ApproximateComparer.DistanceEpsilon);
            double t = (this.Z - segment.Start * this.SweepDirection) / den;

            return new LineSegment(segment.Start, segment.Start + segment.Direction * t);
        }



#endif
        private void AddConeAndEnqueueEvents(VertexEvent vertexEvent) {
            var leftVertexEvent = vertexEvent as LeftVertexEvent;
            if (leftVertexEvent != null) {
                PolylinePoint nextPoint = vertexEvent.Vertex.NextOnPolyline;
                this.CloseConesAtLeftVertex(leftVertexEvent, nextPoint);
            } else {
                var rightVertexEvent = vertexEvent as RightVertexEvent;
                if (rightVertexEvent != null) {
                    PolylinePoint nextPoint = vertexEvent.Vertex.PrevOnPolyline;
                    this.CloseConesAtRightVertex(rightVertexEvent, nextPoint);
                } else {
                    this.CloseConesAtLeftVertex(vertexEvent, vertexEvent.Vertex.NextOnPolyline);
                    this.CloseConesAtRightVertex(vertexEvent, vertexEvent.Vertex.PrevOnPolyline);
                }
            }
        }

        private void CloseConesAtRightVertex(VertexEvent rightVertexEvent,
                                     PolylinePoint nextVertex) {
            Point prevSite = rightVertexEvent.Vertex.NextOnPolyline.Point;
            double prevZ = prevSite* this.SweepDirection;
            if (prevZ <= this.Z && this.Z - prevZ < ApproximateComparer.DistanceEpsilon) {
                this.RemoveConesClosedBySegment(prevSite, rightVertexEvent.Vertex.Point);
            }

            Point site = rightVertexEvent.Site;
            Point coneLp = site + this.ConeLeftSideDirection;
            Point coneRp = site + this.ConeRightSideDirection;
            Point nextSite = nextVertex.Point;
            //SugiyamaLayoutSettings.Show(new LineSegment(site, coneLP), new LineSegment(site, coneRP), new LineSegment(site, nextSite));
            //try to remove the right side
            if ((site - prevSite)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.RemoveRightSide(new RightObstacleSide(rightVertexEvent.Vertex.NextOnPolyline));
            }

            if (this.GetZ(nextSite) + ApproximateComparer.DistanceEpsilon < this.GetZ(rightVertexEvent)) {
                return;
            }

            if (!Point.PointToTheRightOfLineOrOnLine(nextSite, site, coneLp)) {
                //if (angle <= -coneAngle / 2) {
                //   CreateConeOnVertex(rightVertexEvent);
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
            //                var cone = new Cone(rightVertexEvent.Vertex.Point, this);
            //                var obstacleSideSeg = new BrokenConeSide(cone.Apex, nextVertex, new ConeLeftSide(cone));
            //                cone.LeftSide = obstacleSideSeg;
            //                cone.RightSide = new ConeRightSide(cone);
            //                var rnode = InsertToTree(rightConeSides, cone.RightSide);
            //                LookForIntersectionWithConeRightSide(rnode);
            RBNode<ConeSide> lnode =
                this.leftConeSides.FindFirst(side => PointIsToTheLeftOfSegment(rightVertexEvent.Site, side));
            this.FixConeLeftSideIntersections(rightVertexEvent.Vertex, nextVertex, lnode);
            if ((nextVertex.Point - rightVertexEvent.Site)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
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

        [SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        private RightIntersectionEvent CreateRightIntersectionEvent(ConeRightSide coneRightSide, Point intersection,
                                                            PolylinePoint obstacleSideVertex) {
            Debug.Assert(Math.Abs((obstacleSideVertex.Point - intersection)* this.SweepDirection) >
                         ApproximateComparer.DistanceEpsilon);
            return new RightIntersectionEvent(coneRightSide,
                                              intersection, obstacleSideVertex);
        }

        private RBNode<ConeSide> GetLastNodeToTheLeftOfPointInRightSegmentTree(Point obstacleSideStart) {
            return this.rightConeSides.FindLast(
                s => PointIsToTheRightOfSegment(obstacleSideStart, s));
        }

        private void LookForIntersectionOfObstacleSideAndLeftConeSide(Point obstacleSideStart,
                                                              PolylinePoint obstacleSideVertex) {
            RBNode<ConeSide> node = this.GetFirstNodeToTheRightOfPoint(obstacleSideStart);
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
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)")]
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

        private void FixConeLeftSideIntersections(PolylinePoint obstSideStart, PolylinePoint obstSideEnd,
                                          RBNode<ConeSide> rbNode) {
            if (rbNode != null) {
                Point intersection;
                var seg = rbNode.Item as ConeLeftSide;
                if (seg != null &&
                    Point.IntervalIntersectsRay(obstSideStart.Point, obstSideEnd.Point, seg.Start, seg.Direction,
                                                out intersection)) {
                    this.EnqueueEvent(new LeftIntersectionEvent(seg, intersection, obstSideEnd));
                }
            }
        }

        private RBNode<ConeSide> InsertToTree(RbTree<ConeSide> tree, ConeSide coneSide) {
            Debug.Assert(coneSide.Direction* this.SweepDirection > ApproximateComparer.DistanceEpsilon);
            this.coneSideComparer.SetOperand(coneSide);
            return tree.Insert(coneSide);
        }

        private void CloseConesAtLeftVertex(VertexEvent leftVertexEvent, PolylinePoint nextVertex) {
            //close segments first
            Point prevSite = leftVertexEvent.Vertex.PrevOnPolyline.Point;
            double prevZ = prevSite* this.SweepDirection;
            if (prevZ <= this.Z && this.Z - prevZ < ApproximateComparer.DistanceEpsilon) {
                //Show(
                //    new Ellipse(1, 1, prevSite),
                //    CurveFactory.CreateBox(2, 2, leftVertexEvent.Vertex.Point));

                this.RemoveConesClosedBySegment(leftVertexEvent.Vertex.Point, prevSite);
            }

            Point site = leftVertexEvent.Site;
            Point coneLp = site + this.ConeLeftSideDirection;
            Point coneRp = site + this.ConeRightSideDirection;
            Point nextSite = nextVertex.Point;
            // SugiyamaLayoutSettings.Show(new LineSegment(site, coneLP), new LineSegment(site, coneRP), new LineSegment(site, nextSite));

            if ((site - prevSite)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                this.RemoveLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex.PrevOnPolyline));
            }

            if (Point.PointToTheRightOfLineOrOnLine(nextSite, site, site + this.DirectionPerp)) {
                //if (angle > Math.PI / 2)
                //   CreateConeOnVertex(leftVertexEvent); //it is the last left vertex on this obstacle
            } else if (!Point.PointToTheLeftOfLineOrOnLine(nextSite, site, coneRp)) {
                //if (angle >= coneAngle / 2) {
                // CreateConeOnVertex(leftVertexEvent);
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                //we schedule LeftVertexEvent for a vertex with horizontal segment to the left on the top of the obstace
            } else if (!Point.PointToTheLeftOfLineOrOnLine(nextSite, site, coneLp)) {
                //if (angle >= -coneAngle / 2) {
                //we cannot completely obscure the cone here
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                //the obstacle side is inside of the cone
                //we need to create an obstacle right side segment instead of the cone side
                //                var cone = new Cone(leftVertexEvent.Vertex.Point, this);
                //                var rightSide = new BrokenConeSide(leftVertexEvent.Vertex.Point, nextVertex,
                //                                                        new ConeRightSide(cone));
                //                cone.RightSide = rightSide;
                //                cone.LeftSide = new ConeLeftSide(cone);
                //                LookForIntersectionWithConeLeftSide(InsertToTree(leftConeSides, cone.LeftSide));
                RBNode<ConeSide> rbNode = this.rightConeSides.FindLast(s => PointIsToTheRightOfSegment(site, s));
                this.FixConeRightSideIntersections(leftVertexEvent.Vertex, nextVertex, rbNode);
                if ((nextVertex.Point - leftVertexEvent.Site)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                    this.InsertLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex));
                }
            } else {
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                if ((nextVertex.Point - leftVertexEvent.Site)* this.SweepDirection > ApproximateComparer.DistanceEpsilon) {
                    //if( angle >- Pi/2
                    // Debug.Assert(angle > -Math.PI / 2);
                    this.LookForIntersectionOfObstacleSideAndRightConeSide(leftVertexEvent.Site, nextVertex);
                    this.InsertLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex));
                }
            }
        }

        private void RemoveCone(Cone cone) {
            Debug.Assert(cone.Removed == false);
            cone.Removed = true;
            this.RemoveSegFromLeftTree(cone.LeftSide);
            this.RemoveSegFromRightTree(cone.RightSide);
        }

        private void RemoveSegFromRightTree(ConeSide coneSide) {
            //   ShowRightTree();
            Debug.Assert(coneSide.Removed == false);
            this.coneSideComparer.SetOperand(coneSide);
            RBNode<ConeSide> b = this.rightConeSides.Remove(coneSide);
            coneSide.Removed = true;
            if (b == null) {
                double tmpZ = this.Z;
                this.Z = Math.Max(this.GetZ(coneSide.Start), this.Z - 0.01);
                //we need to return to the past a little bit when the order was still correc
                this.coneSideComparer.SetOperand(coneSide);
                b = this.rightConeSides.Remove(coneSide);
                this.Z = tmpZ;

#if TEST_MSAGL
                if (b == null) {
                    this.PrintOutRightSegTree();
                    this.ShowRightTree(CurveFactory.CreateDiamond(3, 4, coneSide.Start));
                    GeometryGraph gg = CreateGraphFromObstacles(this.Obstacles);
                    GeometryGraphWriter.Write(gg, "c:\\tmp\\bug1");
                }
#endif
            }
            Debug.Assert(b != null);
        }

        private void RemoveSegFromLeftTree(ConeSide coneSide) {
            Debug.Assert(coneSide.Removed == false);
            coneSide.Removed = true;
            this.coneSideComparer.SetOperand(coneSide);
            RBNode<ConeSide> b = this.leftConeSides.Remove(coneSide);

            if (b == null) {
                double tmpZ = this.Z;
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
        /// <param name="obstSideEndVertex"></param>
        /// <param name="rbNode">represents a node of the right cone side</param>
        /// <param name="obstSideStartVertex"></param>
        private void FixConeRightSideIntersections(PolylinePoint obstSideStartVertex, PolylinePoint obstSideEndVertex,
                                           RBNode<ConeSide> rbNode) {
            if (rbNode != null) {
                Point intersection;
                var seg = rbNode.Item as ConeRightSide;
                if (seg != null &&
                    Point.IntervalIntersectsRay(obstSideStartVertex.Point, obstSideEndVertex.Point, seg.Start,
                                                seg.Direction,
                                                out intersection)) {
                    this.EnqueueEvent(this.CreateRightIntersectionEvent(seg, intersection, obstSideEndVertex));
                }
            }
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
                var seg = (BrokenConeSide) leftNode.Item;
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
                var seg = (BrokenConeSide) rightNode.Item;
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


        //        static int count;
        private void GoOverConesSeeingVertexEvent(SweepEvent vertexEvent) {
            RBNode<ConeSide> rbNode = this.FindFirstSegmentInTheRightTreeNotToTheLeftOfVertex(vertexEvent);

            if (rbNode == null) {
                return;
            }

            ConeSide coneRightSide = rbNode.Item;
            Cone cone = coneRightSide.Cone;
            ConeSide leftConeSide = cone.LeftSide;
            if (VertexIsToTheLeftOfSegment(vertexEvent, leftConeSide)) {
                return;
            }

            var visibleCones = new List<Cone> {cone};
            this.coneSideComparer.SetOperand(leftConeSide);
            rbNode = this.leftConeSides.Find(leftConeSide);

            if (rbNode == null) {
                double tmpZ = this.Z;

                this.Z = Math.Max(this.GetZ(leftConeSide.Start), this.PreviousZ);
                //we need to return to the past when the order was still correct
                this.coneSideComparer.SetOperand(leftConeSide);
                rbNode = this.leftConeSides.Find(leftConeSide);
                this.Z = tmpZ;


#if TEST_MSAGL
                if (rbNode == null) {
                    //GeometryGraph gg = CreateGraphFromObstacles();
                    //gg.Save("c:\\tmp\\bug");


                    this.PrintOutLeftSegTree();
                    System.Diagnostics.Debug.WriteLine(leftConeSide);
                    this.ShowLeftTree(new Ellipse(3, 3, vertexEvent.Site));
                    this.ShowRightTree(new Ellipse(3, 3, vertexEvent.Site));
                }
#endif
            }

            rbNode = this.leftConeSides.Next(rbNode);
            while (rbNode != null && !VertexIsToTheLeftOfSegment(vertexEvent, rbNode.Item)) {
                visibleCones.Add(rbNode.Item.Cone);
                rbNode = this.leftConeSides.Next(rbNode);
            }

            //Show(new Ellipse(1, 1, vertexEvent.Site));

            foreach (Cone c in visibleCones) {
                this.AddEdge(c.Apex, vertexEvent.Site);
                this.RemoveCone(c);
            }
        }

        private void AddEdge(Point a, Point b) {
            Debug.Assert(this.PortLocations.Contains(a));
            /*********************
            A complication arises when we have overlaps. Loose obstacles become large enough to contain several
            ports. We need to avoid a situation when a port has degree more than one. 
            To avoid this situation we redirect to b every edge incoming into a. 
            Notice that we create a new graph for each AddDiriction call, so all this edges point roughly to the 
            direction of the sweep and the above procedure just alignes the edges better.
            In the resulting graph, which contains the sum of the graphs passed to AddDirection, of course
            a port can have an incoming and outcoming edge at the same time
            *******************/


            VisibilityEdge ab = this.visibilityGraph.AddEdge(a, b);
            VisibilityVertex av = ab.Source;
            Debug.Assert(av.Point == a && ab.TargetPoint == b);
            //all edges adjacent to a which are different from ab
            VisibilityEdge[] edgesToFix =
                av.InEdges.Where(e => e != ab).Concat(av.OutEdges.Where(e => e != ab)).ToArray();
            foreach (VisibilityEdge edge in edgesToFix) {
                Point c = (edge.Target == av ? edge.Source : edge.Target).Point;
                VisibilityGraph.RemoveEdge(edge);
                this.visibilityGraph.AddEdge(c, b);
            }
        }


#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.Int32.ToString")]
        private static GeometryGraph CreateGraphFromObstacles(IEnumerable<Polyline> obstacles) {
            var gg = new GeometryGraph();
            foreach (var ob in obstacles) {
                gg.Nodes.Add(new Node(ob.ToCurve()));
            }
            return gg;
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.WriteLine(System.String)")]
        private void PrintOutLeftSegTree() {
            System.Diagnostics.Debug.WriteLine("Left cone segments");
            foreach (var t in this.leftConeSides) {
                System.Diagnostics.Debug.WriteLine(t);
            }

            System.Diagnostics.Debug.WriteLine("end of left cone segments");
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
            if (this.SweepDirection *(vertexEvent.Site - vertexEvent.Vertex.PrevOnPolyline.Point) >
                ApproximateComparer.Tolerance) {
                return;
            }
            //otherwise we enqueue the vertex twice; once as a LeftVertexEvent and once as a RightVertexEvent
            base.EnqueueEvent(vertexEvent);
        }
    }
}