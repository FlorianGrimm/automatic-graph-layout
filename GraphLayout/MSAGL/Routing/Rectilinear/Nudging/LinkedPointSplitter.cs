using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// intersects a set of horizontal LinkedPoints with a set of vertical LinkedPoints
    /// </summary>
    internal class LinkedPointSplitter {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="horizontalPoints">no two horizontal segs overlap, but they can share an end point</param>
        /// <param name="verticalPoints">no two vertical segs overlap, but they can share an end point</param>
        internal LinkedPointSplitter(List<LinkedPoint> horizontalPoints, List<LinkedPoint> verticalPoints) {
            this.VerticalPoints = verticalPoints;
            this.HorizontalPoints =horizontalPoints;
        }

        private List<LinkedPoint> HorizontalPoints { get; set; }

        private List<LinkedPoint> VerticalPoints { get; set; }

        internal void SplitPoints() {
            if(this.VerticalPoints.Count==0 || this.HorizontalPoints.Count==0) {
                return; //there will be no intersections
            }

            this.InitEventQueue();
            this.ProcessEvents();
        }

        private void ProcessEvents() {
            while (!this.Queue.IsEmpty()) {
                double z;
                var linkedPoint = this.Queue.Dequeue(out z);
                this.ProcessEvent(linkedPoint, z);
            }
        }

        private void ProcessEvent(LinkedPoint linkedPoint, double z){
            if(ApproximateComparer.Close(linkedPoint.Next.Point.X, linkedPoint.Point.X)) {
                if (z==Low(linkedPoint)) {
                    this.ProcessLowLinkedPointEvent(linkedPoint);
                } else {
                    this.ProcessHighLinkedPointEvent(linkedPoint);
                }
            } else {
                this.IntersectWithTree(linkedPoint);
            }
        }

        private void IntersectWithTree(LinkedPoint horizontalPoint) {
            double left, right;
            bool xAligned;
            Debug.Assert(ApproximateComparer.Close(horizontalPoint.Y,horizontalPoint.Next.Y));
            var y = horizontalPoint.Y;
            if(horizontalPoint.Point.X<horizontalPoint.Next.Point.X) {
                left = horizontalPoint.Point.X;
                right = horizontalPoint.Next.Point.X;
                xAligned=true;
            }else {
                right= horizontalPoint.Point.X;
                left = horizontalPoint.Next.Point.X;
                xAligned=false;
            }
            if(xAligned) {
                for ( var node = this.tree.FindFirst(p => left<= p.Point.X); 
                node!=null &&  node.Item.Point.X <= right ;
                node= this.tree.Next(node)) {
                var p = new Point(node.Item.Point.X, y );
                horizontalPoint = TrySplitHorizontalPoint(horizontalPoint, p, true);
                TrySplitVerticalPoint(node.Item,p);
            }
            } else //xAligned==false
{
                for (var node = this.tree.FindLast(p => p.Point.X <= right);
                node != null && node.Item.Point.X >= left;
                node = this.tree.Previous(node)) {
                    var p = new Point(node.Item.Point.X, y);
                    horizontalPoint = TrySplitHorizontalPoint(horizontalPoint, p, false);
                    TrySplitVerticalPoint(node.Item, p);
                }
            }
        }

        private static void TrySplitVerticalPoint(LinkedPoint linkedPoint, Point point) {
            Debug.Assert(ApproximateComparer.Close(linkedPoint.X, linkedPoint.Next.X));
            if (Low(linkedPoint) + ApproximateComparer.DistanceEpsilon < point.Y && point.Y + ApproximateComparer.DistanceEpsilon < High(linkedPoint)) {
                linkedPoint.SetNewNext(point);
            }
        }

        private static LinkedPoint TrySplitHorizontalPoint(LinkedPoint horizontalPoint, Point point, bool xAligned) {
            Debug.Assert(ApproximateComparer.Close(horizontalPoint.Y, horizontalPoint.Next.Y));
            if (xAligned && horizontalPoint.X + ApproximateComparer.DistanceEpsilon < point.X &&
                point.X + ApproximateComparer.DistanceEpsilon < horizontalPoint.Next.X ||
                !xAligned && horizontalPoint.Next.X + ApproximateComparer.DistanceEpsilon < point.X &&
                point.X + ApproximateComparer.DistanceEpsilon < horizontalPoint.X) {
                horizontalPoint.SetNewNext(point);
                return horizontalPoint.Next;
            }
            return horizontalPoint;
        }

        private void ProcessHighLinkedPointEvent(LinkedPoint linkedPoint) {
            this.tree.Remove(linkedPoint);
        }

        private readonly RbTree<LinkedPoint> tree = new RbTree<LinkedPoint>((a, b) => a.Point.X.CompareTo(b.Point.X));

        private void ProcessLowLinkedPointEvent(LinkedPoint linkedPoint) {
            this.tree.Insert(linkedPoint);
        }

        private void InitEventQueue() {
            this.Queue = new GenericBinaryHeapPriorityQueue<LinkedPoint>();
            foreach (var vertPoint in this.VerticalPoints) {
                this.Queue.Enqueue(vertPoint, Low(vertPoint));
            }
            //a horizontal point will appear in the queue after a vertical point 
            // with the same coordinate low coorinate
            foreach (var horizPoint in this.HorizontalPoints) {
                this.Queue.Enqueue(horizPoint, horizPoint.Point.Y);
            }
        }

        private static double Low(LinkedPoint vertPoint) {
            return Math.Min(vertPoint.Point.Y, vertPoint.Next.Point.Y);
        }

        private static double High(LinkedPoint vertPoint) {
            return Math.Max(vertPoint.Point.Y, vertPoint.Next.Point.Y);
        }

        private GenericBinaryHeapPriorityQueue<LinkedPoint> Queue { get; set; }
    }
}