using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// The class is looking for the free space around AxisEdges
    /// </summary>
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=301
    internal class FreeSpaceFinder : LineSweeperBase {
#else
    internal class FreeSpaceFinder : LineSweeperBase, IComparer<AxisEdgesContainer> {
#endif
        private static double AreaComparisonEpsilon = ApproximateComparer.IntersectionEpsilon;
        private readonly PointProjection xProjection;
        
        internal static double X(Point p){return p.X;}
        internal static double MinusY(Point p) { return -p.Y; }

        private readonly RbTree<AxisEdgesContainer> edgeContainersTree;
        internal Dictionary<AxisEdge, List<PathEdge>> PathOrders { get; set; }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="direction"></param>
        /// <param name="obstacles"></param>
        /// <param name="axisEdgesToObstaclesTheyOriginatedFrom"></param>
        /// <param name="pathOrders"></param>
        /// <param name="axisEdges">edges to find the empty space around</param>
        internal FreeSpaceFinder(Direction direction, IEnumerable<Polyline> obstacles, Dictionary<AxisEdge,Polyline> axisEdgesToObstaclesTheyOriginatedFrom, Dictionary<AxisEdge, List<PathEdge>> pathOrders, 
            IEnumerable<AxisEdge> axisEdges): base(obstacles, new CompassVector(direction).ToPoint()) {
            this.DirectionPerp = new CompassVector(direction).Right.ToPoint();
            this.PathOrders = pathOrders;
            this.xProjection = direction == Direction.North ? (PointProjection)X : MinusY;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=301
            edgeContainersTree = new RbTree<AxisEdgesContainer>(new FreeSpaceFinderComparer(this));
#else
            this.edgeContainersTree = new RbTree<AxisEdgesContainer>(this);
#endif
            this.SweepPole = CompassVector.VectorDirection(this.SweepDirection);
            Debug.Assert(CompassVector.IsPureDirection(this.SweepPole));
            this.AxisEdges = axisEdges;
            this.AxisEdgesToObstaclesTheyOriginatedFrom = axisEdgesToObstaclesTheyOriginatedFrom;
            
        }

        private Dictionary<AxisEdge, Polyline> AxisEdgesToObstaclesTheyOriginatedFrom { get; set; }

        protected Direction SweepPole { get; set; }

     //   List<Path> EdgePaths { get; set; }

        //VisibilityGraph PathVisibilityGraph { get; set; }

        /// <summary>
        /// calculates the right offsets
        /// </summary>
        internal void FindFreeSpace() {
            this.InitTheQueueOfEvents();
            this.ProcessEvents();
        //    ShowAxisEdges();            
        }

        private void ProcessEvents() {
            while (this.EventQueue.Count > 0) {
                this.ProcessEvent(this.EventQueue.Dequeue());
            }
        }

        private void ProcessEvent(SweepEvent sweepEvent) {
//            if (SweepDirection.Y == 1 && (sweepEvent.Site - new Point(75.45611, 15.21524)).Length < 0.1)
               // ShowAtPoint(sweepEvent.Site);
          
            var vertexEvent = sweepEvent as VertexEvent;
            if (vertexEvent != null) {
                this.ProcessVertexEvent(vertexEvent);
            } else {
                var lowEdgeEvent = sweepEvent as AxisEdgeLowPointEvent;
                this.Z = this.GetZ(sweepEvent.Site);
                if (lowEdgeEvent != null) {
                    this.ProcessLowEdgeEvent(lowEdgeEvent);
                } else {
                    this.ProcessHighEdgeEvent((AxisEdgeHighPointEvent)sweepEvent);
                }
            }
        }

        private void ProcessHighEdgeEvent(AxisEdgeHighPointEvent edgeForNudgingHighPointEvent) {
            var edge = edgeForNudgingHighPointEvent.AxisEdge;
            this.RemoveEdge(edge);
            this.ConstraintEdgeWithObstaclesAtZ(edge, edge.Target.Point);
        }

        private void ProcessLowEdgeEvent(AxisEdgeLowPointEvent lowEdgeEvent) {
            
            var edge = lowEdgeEvent.AxisEdge;

            var containerNode = this.GetOrCreateAxisEdgesContainer(edge);
            containerNode.Item.AddEdge(edge);
            var prev = this.edgeContainersTree.Previous(containerNode);
            if (prev != null) {
                foreach (var prevEdge in prev.Item.Edges) {
                    foreach (var ed in containerNode.Item.Edges) {
                        this.TryToAddRightNeighbor(prevEdge, ed);
                    }
                }
            }

            var next = this.edgeContainersTree.Next(containerNode);
            if (next != null) {
                foreach (var ed in containerNode.Item.Edges) {
                    foreach (var neEdge in next.Item.Edges) {
                        this.TryToAddRightNeighbor(ed, neEdge);
                    }
                }
            }

            this.ConstraintEdgeWithObstaclesAtZ(edge, edge.Source.Point);
        }

        private void TryToAddRightNeighbor(AxisEdge leftEdge, AxisEdge rightEdge) {
            if (this.ProjectionsOfEdgesOverlap(leftEdge, rightEdge)) {
                leftEdge.AddRightNeighbor(rightEdge);
            }
        }

        private bool ProjectionsOfEdgesOverlap(AxisEdge leftEdge, AxisEdge rightEdge) {
            return this.SweepPole == Direction.North
                       ? !(leftEdge.TargetPoint.Y < rightEdge.SourcePoint.Y - ApproximateComparer.DistanceEpsilon ||
                           rightEdge.TargetPoint.Y < leftEdge.SourcePoint.Y - ApproximateComparer.DistanceEpsilon)
                       : !(leftEdge.TargetPoint.X < rightEdge.SourcePoint.X - ApproximateComparer.DistanceEpsilon ||
                           rightEdge.TargetPoint.X < leftEdge.SourcePoint.X - ApproximateComparer.DistanceEpsilon);
        }

#if TEST_MSAGL
// ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void DebShowEdge(AxisEdge edge, Point point){
            // ReSharper restore UnusedMember.Local
            // if (InterestingEdge(edge))
            this.ShowEdge(edge,point);
        }


// ReSharper disable SuggestBaseTypeForParameter
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowEdge(AxisEdge edge, Point point){
// ReSharper restore SuggestBaseTypeForParameter

            var dd = this.GetObstacleBoundaries("black");
            var seg = new DebugCurve( 1, "red", new LineSegment(edge.Source.Point, edge.Target.Point));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(dd.Concat(
                new[]{seg ,new DebugCurve("blue",CurveFactory.CreateEllipse(3, 3, point))}));
  

        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private IEnumerable<DebugCurve> GetObstacleBoundaries(string color){
            return this.Obstacles.Select(p => new DebugCurve(1, color, p));
        }
#endif


        /// <summary>
        /// 
        /// </summary>
        /// <param name="edge"></param>
        /// <param name="point">a point on the edge on Z level</param>
        private void ConstraintEdgeWithObstaclesAtZ(AxisEdge edge, Point point) {
            Debug.Assert(point==edge.Source.Point || point == edge.Target.Point);
            this.ConstraintEdgeWithObstaclesAtZFromLeft(edge, point);
            this.ConstraintEdgeWithObstaclesAtZFromRight(edge, point);
        }

        private void ConstraintEdgeWithObstaclesAtZFromRight(AxisEdge edge, Point point) {
            var node = this.GetActiveSideFromRight(point);
            if (node == null) {
                return;
            }

            if (this.NotRestricting(edge, ((LeftObstacleSide) node.Item).Polyline)) {
                return;
            }

            var x = this.ObstacleSideComparer.IntersectionOfSideAndSweepLine(node.Item);
            edge.BoundFromRight(x* this.DirectionPerp);
        }

        private RBNode<SegmentBase> GetActiveSideFromRight(Point point){
            return this.LeftObstacleSideTree.FindFirst(side =>
                                           PointToTheLeftOfLineOrOnLineLocal(point, side.Start, side.End));
        }

        private void ConstraintEdgeWithObstaclesAtZFromLeft(AxisEdge edge, Point point){
            //    ShowNudgedSegAndPoint(point, nudgedSegment);
            var node = this.GetActiveSideFromLeft(point);
            if (node == null) {
                return;
            }

            if (this.NotRestricting(edge, ((RightObstacleSide) node.Item).Polyline)) {
                return;
            }

            var x = this.ObstacleSideComparer.IntersectionOfSideAndSweepLine(node.Item);
            edge.BoundFromLeft(x * this.DirectionPerp);
        }

        private static bool PointToTheLeftOfLineOrOnLineLocal(Point a, Point linePoint0, Point linePoint1) {
            return Point.SignedDoubledTriangleArea(a, linePoint0, linePoint1) > -AreaComparisonEpsilon;
        }

        private static bool PointToTheRightOfLineOrOnLineLocal(Point a, Point linePoint0, Point linePoint1) {
            return Point.SignedDoubledTriangleArea(linePoint0, linePoint1, a) < AreaComparisonEpsilon;
        }

        private RBNode<SegmentBase> GetActiveSideFromLeft(Point point){
            return this.RightObstacleSideTree.FindLast(side =>
                                                  PointToTheRightOfLineOrOnLineLocal(point, side.Start, side.End));
        }
        #region debug
#if TEST_MSAGL
        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowPointAndEdge(Point point, AxisEdge edge) {
// ReSharper restore UnusedMember.Local
            List<ICurve> curves = this.GetCurves(point, edge);

            LayoutAlgorithmSettings.Show(curves.ToArray());
        }

// ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowPointAndEdgeWithSweepline(Point point, AxisEdge edge) {
// ReSharper restore UnusedMember.Local
            List<ICurve> curves = this.GetCurves(point, edge);

            curves.Add(new LineSegment(this.SweepDirection * this.Z + 10 * this.DirectionPerp, this.SweepDirection * this.Z - 10 * this.DirectionPerp));

            LayoutAlgorithmSettings.Show(curves.ToArray());
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private List<ICurve> GetCurves(Point point, AxisEdge edge) {
            var ellipse = CurveFactory.CreateEllipse(3, 3, point);
            var curves = new List<ICurve>(this.Obstacles.Select(o => o as ICurve)){ellipse,
                                                                            new LineSegment(edge.Source.Point, edge.Target.Point
                                                                                )};

            if (edge.RightBound < double.PositiveInfinity) {
                double rightOffset = edge.RightBound;
                var del = this.DirectionPerp * rightOffset;
                curves.Add(new LineSegment(edge.Source.Point + del, edge.Target.Point + del));
            }
            if (edge.LeftBound > double.NegativeInfinity) {
                double leftOffset = edge.LeftBound;
                var del = this.DirectionPerp * leftOffset;
                curves.Add(new LineSegment(edge.Source.Point + del, edge.Target.Point  + del));
            }

            curves.AddRange((from e in this.PathOrders.Keys
                             let a = e.SourcePoint
                             let b = e.TargetPoint
                             select new CubicBezierSegment(a, a*0.8 + b*0.2, a*0.2 + b*0.8, b)).Cast<ICurve>());

            return curves;
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private List<DebugCurve> GetCurvesTest(Point point){
            var ellipse = CurveFactory.CreateEllipse(3, 3, point);
            var curves = new List<DebugCurve>(this.Obstacles.Select(o => new DebugCurve(100, 1, "black", o)))
                         {new DebugCurve(100, 1, "red", ellipse)};
            curves.AddRange(from e in this.edgeContainersTree
                             from axisEdge in e.Edges
                             let a = axisEdge.Source.Point
                             let b = axisEdge.Target.Point
                             select new DebugCurve(100, 1, "green", new LineSegment(a, b)));


            curves.AddRange(RightNeighborsCurvesTest(this.edgeContainersTree));

            return curves;
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static IEnumerable<DebugCurve> RightNeighborsCurvesTest(IEnumerable<AxisEdgesContainer> rbTree) {
            foreach (var container in rbTree) {
                foreach (var edge  in container.Edges) {
                    foreach (var rn in edge.RightNeighbors) {
                        yield return new DebugCurve(100,1,"brown",new LineSegment(EdgeMidPoint(edge), EdgeMidPoint(rn)));
                    }
                }
            }
        }

        private static Point EdgeMidPoint(AxisEdge edge) {
            return 0.5*(edge.SourcePoint + edge.TargetPoint);
        }

        // ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowAxisEdges() {
            // ReSharper restore UnusedMember.Local
            var dd = new List<DebugCurve>(this.GetObstacleBoundaries("black"));
            int i = 0;
            foreach (var axisEdge in this.AxisEdges) {
                var color = DebugCurve.Colors[i];
                dd.Add(new DebugCurve(200, 1, color,
                                       new LineSegment(axisEdge.Source.Point, axisEdge.Target.Point)));
                Point perp = axisEdge.Direction == Direction.East ? new Point(0, 1) : new Point(-1, 0);
                if (axisEdge.LeftBound != double.NegativeInfinity) {
                    dd.Add(new DebugCurve(200, 0.5, color,
                        new LineSegment(axisEdge.Source.Point + axisEdge.LeftBound * perp, axisEdge.Target.Point + axisEdge.LeftBound * perp)));
                }
                if (axisEdge.RightBound != double.PositiveInfinity) {
                    dd.Add(new DebugCurve(200, 0.5, color,
                        new LineSegment(axisEdge.Source.Point - axisEdge.RightBound * perp, axisEdge.Target.Point - axisEdge.RightBound * perp)));
                }
                i = (i + 1) % DebugCurve.Colors.Length;
            }
            DebugCurveCollection.WriteToFile(dd, "c:/tmp/ae");
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(dd);
        }

// ReSharper disable UnusedMember.Local
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private void ShowAtPoint(Point point) {
// ReSharper restore UnusedMember.Local
            var curves = this.GetCurvesTest(point);
            LayoutAlgorithmSettings.ShowDebugCurves(curves.ToArray());
        }
#endif
        #endregion
        private RBNode<AxisEdgesContainer> GetOrCreateAxisEdgesContainer(AxisEdge edge) {
            var source = edge.Source.Point;

            var ret = this.GetAxisEdgesContainerNode(source);
      
            if(ret!=null) {
                return ret;
            }

            return this.edgeContainersTree.Insert(new AxisEdgesContainer(source));
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point">the point has to be on the same line as the container</param>
        /// <returns></returns>
        private RBNode<AxisEdgesContainer> GetAxisEdgesContainerNode(Point point){
            var prj = this.xProjection( point);
            var ret =
                this.edgeContainersTree.FindFirst(cont => this.xProjection(cont.Source) >= prj-ApproximateComparer.DistanceEpsilon/2);
            if(ret != null) {
                if (this.xProjection(ret.Item.Source) <= prj+ApproximateComparer.DistanceEpsilon/2) {
                    return ret;
                }
            }

            return null;
        }

        private void ProcessVertexEvent(VertexEvent vertexEvent) {
            this.Z = this.GetZ(vertexEvent);
            var leftVertexEvent = vertexEvent as LeftVertexEvent;
            if (leftVertexEvent != null) {
                this.ProcessLeftVertex(leftVertexEvent, vertexEvent.Vertex.NextOnPolyline);
            } else {
                var rightVertexEvent = vertexEvent as RightVertexEvent;
                if (rightVertexEvent != null) {
                    this.ProcessRightVertex(rightVertexEvent, vertexEvent.Vertex.PrevOnPolyline);
                } else {
                    this.ProcessLeftVertex(vertexEvent, vertexEvent.Vertex.NextOnPolyline);
                    this.ProcessRightVertex(vertexEvent, vertexEvent.Vertex.PrevOnPolyline);
                }
            }
        }

        private void ProcessRightVertex(VertexEvent rightVertexEvent, PolylinePoint nextVertex){
            Debug.Assert(this.Z == rightVertexEvent.Site* this.SweepDirection);

            var site = rightVertexEvent.Site;
            this.ProcessPrevSegmentForRightVertex(rightVertexEvent, site);

            var delta = nextVertex.Point - rightVertexEvent.Site;
            var deltaX = delta* this.DirectionPerp;
            var deltaZ = delta* this.SweepDirection;
            if (deltaZ <= ApproximateComparer.DistanceEpsilon){
                if (deltaX > 0 && deltaZ >= 0) {
                    this.EnqueueEvent(new RightVertexEvent(nextVertex));
                } else {
                    this.RestrictEdgeContainerToTheRightOfEvent(rightVertexEvent.Vertex);
                }
            }
            else{
                //deltaZ>epsilon
                this.InsertRightSide(new RightObstacleSide(rightVertexEvent.Vertex));
                this.EnqueueEvent(new RightVertexEvent(nextVertex));
                this.RestrictEdgeContainerToTheRightOfEvent(rightVertexEvent.Vertex);

            }
        }

        private void RestrictEdgeContainerToTheRightOfEvent(PolylinePoint polylinePoint) {
            var site = polylinePoint.Point;
            var siteX = this.xProjection(site);
            var containerNode =
                this.edgeContainersTree.FindFirst
                    (container => siteX <= this.xProjection(container.Source));

            if (containerNode != null) {
                foreach (var edge in containerNode.Item.Edges) {
                    if (!this.NotRestricting(edge, polylinePoint.Polyline)) {
                        edge.BoundFromLeft(this.DirectionPerp *site);
                    }
                }
            }
        }

        private bool NotRestricting(AxisEdge edge, Polyline polyline) {
            Polyline p;
            return this.AxisEdgesToObstaclesTheyOriginatedFrom.TryGetValue(edge, out p) && p == polyline;
        }

        private void ProcessPrevSegmentForRightVertex(VertexEvent rightVertexEvent, Point site) {
            var prevSite = rightVertexEvent.Vertex.NextOnPolyline.Point;
            var delta = site - prevSite;
            double deltaZ = delta * this.SweepDirection;
            if (deltaZ > ApproximateComparer.DistanceEpsilon) {
                this.RemoveRightSide(new RightObstacleSide(rightVertexEvent.Vertex.NextOnPolyline));
            }
        }

        private void RemoveEdge(AxisEdge edge){
            var containerNode = this.GetAxisEdgesContainerNode(edge.Source.Point);
            containerNode.Item.RemoveAxis(edge);
            if(containerNode.Item.IsEmpty()) {
                this.edgeContainersTree.DeleteNodeInternal(containerNode);
            }
        }

        private void ProcessLeftVertex(VertexEvent leftVertexEvent, PolylinePoint nextVertex){
            Debug.Assert(this.Z == leftVertexEvent.Site* this.SweepDirection);

            var site = leftVertexEvent.Site;
            this.ProcessPrevSegmentForLeftVertex(leftVertexEvent, site);

            Point delta = nextVertex.Point - leftVertexEvent.Site;
            double deltaX = delta* this.DirectionPerp;
            double deltaZ = delta* this.SweepDirection;
            if (deltaZ <= ApproximateComparer.DistanceEpsilon ){
                if (deltaX < 0 && deltaZ >= 0) {
                    this.EnqueueEvent(new LeftVertexEvent(nextVertex));
                }
            }
            else{
                //deltaZ>epsilon
                this.InsertLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex));
                this.EnqueueEvent(new LeftVertexEvent(nextVertex));
            }
            //ShowAtPoint(leftVertexEvent.Site);
            this.RestrictEdgeFromTheLeftOfEvent(leftVertexEvent.Vertex);
        }

        private void RestrictEdgeFromTheLeftOfEvent(PolylinePoint polylinePoint) {
            //ShowAtPoint(site);
            Point site = polylinePoint.Point;
            RBNode<AxisEdgesContainer> containerNode = this.GetContainerNodeToTheLeftOfEvent(site);

            if (containerNode != null) {
                foreach (var edge in containerNode.Item.Edges) {
                    if (!this.NotRestricting(edge, polylinePoint.Polyline)) {
                        edge.BoundFromRight(site* this.DirectionPerp);
                    }
                }
            }
        }

        private RBNode<AxisEdgesContainer> GetContainerNodeToTheLeftOfEvent(Point site) {
            double siteX = this.xProjection(site);
            return
                this.edgeContainersTree.FindLast(
                    container => this.xProjection(container.Source)<= siteX);
            //                Point.PointToTheRightOfLineOrOnLine(site, container.Source,
            //                                                                                                container.UpPoint));
        }


        private void ProcessPrevSegmentForLeftVertex(VertexEvent leftVertexEvent, Point site) {
            var prevSite = leftVertexEvent.Vertex.PrevOnPolyline.Point;
            var delta = site - prevSite;
            double deltaZ = delta * this.SweepDirection;
            if (deltaZ > ApproximateComparer.DistanceEpsilon) {
                this.RemoveLeftSide(new LeftObstacleSide(leftVertexEvent.Vertex.PrevOnPolyline));
            }
        }

        private void InitTheQueueOfEvents() {
            this.InitQueueOfEvents();
            foreach (var axisEdge in this.AxisEdges) {
                this.EnqueueEventsForEdge(axisEdge);
            }
        }

        protected IEnumerable<AxisEdge> AxisEdges { get; set; }

        private void EnqueueEventsForEdge(AxisEdge edge) {
            if (this.EdgeIsParallelToSweepDir(edge)) {
                this.EnqueueEvent(EdgeLowPointEvent(edge, edge.Source.Point));
                this.EnqueueEvent(EdgeHighPointEvent(edge, edge.Target.Point));
            }
        }

        private bool EdgeIsParallelToSweepDir(AxisEdge edge) {
            return edge.Direction == this.SweepPole || edge.Direction == CompassVector.OppositeDir(this.SweepPole);
        }

        private static SweepEvent EdgeHighPointEvent(AxisEdge edge, Point point) {
            return new AxisEdgeHighPointEvent(edge, point);
        }

        private static SweepEvent EdgeLowPointEvent(AxisEdge edge, Point point) {
            return new AxisEdgeLowPointEvent(edge, point);

        }

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=301
        public class FreeSpaceFinderComparer : IComparer<AxisEdgesContainer>
        {
            private FreeSpaceFinder m_Owner;
            public FreeSpaceFinderComparer(FreeSpaceFinder owner)
            {
                m_Owner = owner;
            }
            public int Compare(AxisEdgesContainer x, AxisEdgesContainer y)
            {
                ValidateArg.IsNotNull(x, "x");
                ValidateArg.IsNotNull(y, "y");
                return (x.Source * m_Owner.DirectionPerp).CompareTo(y.Source * m_Owner.DirectionPerp);
            }
        }
#else
        public int Compare(AxisEdgesContainer x, AxisEdgesContainer y) {
            ValidateArg.IsNotNull(x, "x");
            ValidateArg.IsNotNull(y, "y");
            return (x.Source * this.DirectionPerp).CompareTo(y.Source * this.DirectionPerp);
        }
#endif
    }
}
