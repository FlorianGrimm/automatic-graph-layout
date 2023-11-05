using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;
using Microsoft.Msagl.Core;

namespace Microsoft.Msagl.Routing.Visibility {
    internal class LineSweeperBase : IComparer<SweepEvent> {
        private Point directionPerp; // sweep direction rotated 90 degrees clockwse
        private BinaryHeapWithComparer<SweepEvent> eventQueue;

        protected RbTree<SegmentBase> LeftObstacleSideTree { get; set; }

        protected ObstacleSideComparer ObstacleSideComparer { get; set; }

        protected RbTree<SegmentBase> RightObstacleSideTree { get; set; }
        protected Set<Point> Ports;
        public LineSweeperBase(IEnumerable<Polyline> obstacles, Point sweepDirection) {
            this.Obstacles = obstacles;
            this.SweepDirection = sweepDirection;
            this.DirectionPerp = sweepDirection.Rotate(-Math.PI / 2);
            this.EventQueue = new BinaryHeapWithComparer<SweepEvent>(this);
            this.ObstacleSideComparer = new ObstacleSideComparer(this);
            this.LeftObstacleSideTree = new RbTree<SegmentBase>(this.ObstacleSideComparer);
            this.RightObstacleSideTree = new RbTree<SegmentBase>(this.ObstacleSideComparer);
        }

        protected internal BinaryHeapWithComparer<SweepEvent> EventQueue {
            get { return this.eventQueue; }
            set { this.eventQueue = value; }
        }
        public Point SweepDirection { get; set; }
        /// <summary>
        /// sweep direction rotated by 90 degrees clockwise
        /// </summary>
        protected Point DirectionPerp {
            get { return this.directionPerp; }
            set { this.directionPerp = value; }
        }

        protected double PreviousZ=double.NegativeInfinity;
        private double z;
        public double Z {
            get { return this.z; }
            set {
                if (value > this.z + ApproximateComparer.Tolerance) {
                    this.PreviousZ = this.z;
                }
#if TEST_MSAGL
                Debug.Assert(this.PreviousZ <=value);
#endif
                this.z = value;
         //       Debug.Assert(TreesAreCorrect());
            }
        }

       // protected virtual bool TreesAreCorrect() { return true; }

        protected internal IEnumerable<Polyline> Obstacles { get; set; }


        protected double GetZ(SweepEvent eve) {
            return this.SweepDirection * eve.Site;
        }

        protected double GetZ(Point point) {
            return this.SweepDirection * point;
        }

        protected bool SegmentIsNotHorizontal(Point a, Point b) {
            return Math.Abs((a - b) * this.SweepDirection) > ApproximateComparer.DistanceEpsilon;
        }

        protected void RemoveLeftSide(LeftObstacleSide side) {
            this.ObstacleSideComparer.SetOperand(side);
            this.LeftObstacleSideTree.Remove(side);
        }

        protected void RemoveRightSide(RightObstacleSide side) {
            this.ObstacleSideComparer.SetOperand(side);
            this.RightObstacleSideTree.Remove(side);
        }

        protected void InsertLeftSide(LeftObstacleSide side) {
            this.ObstacleSideComparer.SetOperand(side);
            this.LeftObstacleSideTree.Insert((side));
        }

        protected void InsertRightSide(RightObstacleSide side) {
            this.ObstacleSideComparer.SetOperand(side);
            this.RightObstacleSideTree.Insert(side);
        }

        protected RightObstacleSide FindFirstObstacleSideToTheLeftOfPoint(Point point) {
            var node =
                this.RightObstacleSideTree.FindLast(
                    s => Point.PointToTheRightOfLineOrOnLine(point, s.Start, s.End));
            return node == null ? null : (RightObstacleSide)(node.Item);
        }

        protected LeftObstacleSide FindFirstObstacleSideToToTheRightOfPoint(Point point) {
            var node =
                this.LeftObstacleSideTree.FindFirst(
                    s => !Point.PointToTheRightOfLineOrOnLine(point, s.Start, s.End));
            return node == null ? null : (LeftObstacleSide)node.Item;
        }

        protected void EnqueueEvent(SweepEvent eve) {
            Debug.Assert(this.GetZ(eve.Site)>= this.PreviousZ);
            this.eventQueue.Enqueue(eve);
        }

        protected void InitQueueOfEvents() {
            foreach (var obstacle in this.Obstacles) {
                this.EnqueueLowestPointsOnObstacles(obstacle);
            }

            if (this.Ports != null) {
                foreach (var point in this.Ports) {
                    this.EnqueueEvent(new PortObstacleEvent(point));
                }
            }
        }

        private void EnqueueLowestPointsOnObstacles(Polyline poly) {
            PolylinePoint candidate = this.GetLowestPoint(poly);
            this.EnqueueEvent(new LowestVertexEvent(candidate));
        }

        private PolylinePoint GetLowestPoint(Polyline poly) {
            PolylinePoint candidate = poly.StartPoint;
            PolylinePoint pp = poly.StartPoint.Next;

            for (; pp != null; pp = pp.Next) {
                if (this.Less(pp.Point, candidate.Point)) {
                    candidate = pp;
                }
            }

            return candidate;
        }

        /// <summary>
        /// imagine that direction points up,
        /// lower events have higher priorities,
        /// for events at the same level events to the left have higher priority
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public int Compare(SweepEvent a, SweepEvent b) {
            ValidateArg.IsNotNull(a, "a");
            ValidateArg.IsNotNull(b, "b");
            Point aSite = a.Site;
            Point bSite = b.Site;
            return this.ComparePoints(ref aSite, ref bSite);
        }

        private bool Less(Point a, Point b) {
            return this.ComparePoints(ref a, ref b) < 0;
        }

        private int ComparePoints(ref Point aSite, ref Point bSite) {
            var aProjection = this.SweepDirection * aSite;
            var bProjection = this.SweepDirection * bSite;
            if (aProjection < bProjection) {
                return -1;
            }

            if (aProjection > bProjection) {
                return 1;
            }

            aProjection = this.directionPerp * aSite;
            bProjection = this.directionPerp * bSite;

            if (aProjection < bProjection) {
                return -1;
            }

            return aProjection > bProjection ? 1 : 0;
        }
    }
}
