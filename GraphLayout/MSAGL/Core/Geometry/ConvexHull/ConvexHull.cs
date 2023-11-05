using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing;

namespace Microsoft.Msagl.Core.Geometry {
    /// <summary>
    /// Creates the convex hull of a set of points following "Computational Geometry, second edition" of O'Rourke
    /// </summary>
    public sealed class ConvexHull {
        private HullPoint[] hullPoints;
        private Point pivot;
        private HullStack stack;
        private HullPointComparer comparer;

        private ConvexHull(IEnumerable<Point> bodyPoints) {
            this.SetPivotAndAllocateHullPointsArray(bodyPoints);
        }

        private void SetPivotAndAllocateHullPointsArray(IEnumerable<Point> bodyPoints) {
            this.pivot = new Point(0, Double.MaxValue); //set Y to a very big value
            int pivotIndex = -1;
            int n = 0;
            foreach (Point point in bodyPoints) {
                if (point.Y < this.pivot.Y) {
                    this.pivot = point;
                    pivotIndex = n;
                }
                else if (point.Y == this.pivot.Y) {
                    if (point.X > this.pivot.X) {
                        this.pivot = point;
                        pivotIndex = n;
                    }
                }

                n++;
            }
            if (n >= 1) {
                this.hullPoints = new HullPoint[n - 1]; //we will not copy the pivot into the hull points
                n = 0;
                foreach (Point point in bodyPoints) {
                    if (n != pivotIndex) {
                        this.hullPoints[n++] = new HullPoint(point);
                    } else {
                        pivotIndex = -1; //forget where the pivot was
                    }
                }
            }
        }

        private Point StackTopPoint {
            get { return this.stack.Point; }
        }

        private Point StackSecondPoint {
            get { return this.stack.Next.Point; }
        }

        /// <summary>
        /// calculates the convex hull of the given set of points
        /// </summary>
        /// <param name="pointsOfTheBody">Point of the convex hull.</param>
        /// <returns>The list of extreme points of the hull boundaries in the clockwise order</returns>
        public static IEnumerable<Point> CalculateConvexHull(IEnumerable<Point> pointsOfTheBody) {
            var convexHull = new ConvexHull(pointsOfTheBody);
            return convexHull.Calculate();
        }

        private IEnumerable<Point> Calculate() {
            if (this.pivot.Y == Double.MaxValue) {
                return new Point[0];
            }

            if (this.hullPoints.Length == 0) {
                return new[] { this.pivot };
            }

            this.SortAllPointsWithoutPivot();
            this.Scan();
            return this.EnumerateStack();
        }

        private IEnumerable<Point> EnumerateStack() {
            HullStack stackCell = this.stack;
            while (stackCell != null) {
                yield return stackCell.Point;
                stackCell = stackCell.Next;
            }
        }

        private void Scan() {
            int i = 0;
            while (this.hullPoints[i].Deleted) {
                i++;
            }

            this.stack = new HullStack(this.pivot);
            this.Push(i++);
            if (i < this.hullPoints.Length) {
                if (!this.hullPoints[i].Deleted) {
                    this.Push(i++);
                } else {
                    i++;
                }
            }

            while (i < this.hullPoints.Length) {
                if (!this.hullPoints[i].Deleted) {
                    if (this.LeftTurn(i)) {
                        this.Push(i++);
                    } else {
                        this.Pop();
                    }
                }
                else {
                    i++;
                }
            }

            //cleanup the end
            while (this.StackHasMoreThanTwoPoints() && !this.LeftTurnToPivot()) {
                this.Pop();
            }
        }

        private bool LeftTurnToPivot() {
            return Point.GetTriangleOrientation(this.StackSecondPoint, this.StackTopPoint, this.pivot) ==
                   TriangleOrientation.Counterclockwise;
        }

        private bool StackHasMoreThanTwoPoints() {
            return this.stack.Next != null && this.stack.Next.Next != null;
        }

        private void Pop() {
            this.stack = this.stack.Next;
        }

        private bool LeftTurn(int i) {
            if (this.stack.Next == null) {
                return true; //there is only one point in the stack
            }

            var orientation = Point.GetTriangleOrientationWithIntersectionEpsilon(this.StackSecondPoint, this.StackTopPoint, this.hullPoints[i].Point);
            if (orientation == TriangleOrientation.Counterclockwise) {
                return true;
            }

            if (orientation == TriangleOrientation.Clockwise) {
                return false;
            }

            return this.BackSwitchOverPivot(this.hullPoints[i].Point);
        }

        private bool BackSwitchOverPivot(Point point) {
            //we know here that there at least two points in the stack but it has to be exaclty two 
            if (this.stack.Next.Next != null) {
                return false;
            }

            Debug.Assert(this.StackSecondPoint == this.pivot);
            return this.StackTopPoint.X > this.pivot.X + ApproximateComparer.DistanceEpsilon &&
                   point.X < this.pivot.X - ApproximateComparer.DistanceEpsilon;
        }

        private void Push(int p) {
            var t = new HullStack(this.hullPoints[p].Point) {Next = this.stack };
            this.stack = t;
        }

        private void SortAllPointsWithoutPivot() {
            this.comparer = new HullPointComparer(this.pivot);
            Array.Sort(this.hullPoints, this.comparer);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.String.Format(System.String,System.Object,System.Object)")]
        internal static Polyline CreateConvexHullAsClosedPolyline(IEnumerable<Point> points) {
            var convexHull = new Polyline(CalculateConvexHull(points)) { Closed = true };
#if TEST_MSAGL
            foreach (var point in points) {
                if (Curve.PointRelativeToCurveLocation(point, convexHull) == PointLocation.Outside) {
                    var hullPoint = convexHull[convexHull.ClosestParameter(point)];

                    // This can be too restrictive if very close points are put into the hull.  It is probably 
                    // better to clean up in the caller before doing this, but this assert can also be relaxed.
                  Debug.Assert(ApproximateComparer.Close(point, hullPoint, ApproximateComparer.IntersectionEpsilon * 20), String.Format("not CloseIntersections: initial point {0}, hull point {1}", point, hullPoint));
                    
                }
            }
#endif // TEST_MSAGL
            return convexHull;
        }
    }
}
