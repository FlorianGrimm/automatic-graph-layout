using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    internal class StaircaseRemover {
        protected List<Path> Paths { get; set; }

        protected RTree<Polyline,Point> HierarchyOfObstacles { get; set; }

        private readonly RTree<SegWithIndex, Point> segTree=new RTree<SegWithIndex, Point>();
        private Set<Path> crossedOutPaths = new Set<Path>();

        private StaircaseRemover(List<Path> paths, RectangleNode<Polyline, Point> hierarchyOfObstacles) {
            this.HierarchyOfObstacles = new RTree<Polyline, Point>(hierarchyOfObstacles);
            this.Paths = paths;
        }


        internal static void RemoveStaircases(List<Path> paths, RectangleNode<Polyline, Point> hierarchyOfObstacles) {
            var r = new StaircaseRemover(paths, hierarchyOfObstacles);
            r.Calculate();
        }

        private void Calculate() {
            this.InitHierarchies();
            bool success;
            do {
                success = false;
                foreach (var path in this.Paths.Where(p=>!this.crossedOutPaths.Contains(p))) {
                    success |= this.ProcessPath(path);
                }
            } while (success);
        }

        private bool ProcessPath(Path path) {
            var pts = (Point[])path.PathPoints;
            bool canHaveStaircase;
            if (this.ProcessPoints(ref pts, out canHaveStaircase)) {
                path.PathPoints = pts;
                return true;
            }
            if (!canHaveStaircase) {
                this.crossedOutPaths.Insert(path);
            }

            return false;
        }

        private bool ProcessPoints(ref Point[] pts, out bool canHaveStaircase) {
            var staircaseStart  = this.FindStaircaseStart(pts, out canHaveStaircase);
            if (staircaseStart < 0) {
                return false;
            }

            pts = this.RemoveStaircase(pts, staircaseStart);
            return true;
        }

        private int FindStaircaseStart(Point[] pts, out bool canHaveStaircase) {
            canHaveStaircase = false;
            if (pts.Length < 5) // At least five points make a staircase
{
                return -1;
            }

            var segs = new[] {
                                 new SegWithIndex(pts, 0), new SegWithIndex(pts, 1), new SegWithIndex(pts, 2),
                                 new SegWithIndex(pts, 3)
                             };
            int segToReplace = 0;

            for (int i = 0;;) {
                bool canHaveStaircaseAtI;
                if (this.IsStaircase(pts, i, segs, out canHaveStaircaseAtI)) {
                    canHaveStaircase = true;
                    return i;                    
                }
                canHaveStaircase = canHaveStaircase || canHaveStaircaseAtI;
                i++;
                if (pts.Length < i + 5)// At least five points make a staircase
{
                    return -1;
                }

                segs[segToReplace] = new SegWithIndex(pts, i + 3);
                segToReplace += 1;
                segToReplace %= 4;
            }
        }

        private static Point GetFlippedPoint(Point[] pts, int offset) {
            var horiz = ApproximateComparer.Close(pts[offset].Y, pts[offset + 1].Y);
            return horiz ? new Point(pts[offset + 4].X, pts[offset].Y) : new Point(pts[offset].X, pts[offset + 4].Y);
        }

        /// <summary>
        /// ignoring crossing at a
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="segsToIgnore"></param>
        /// <returns></returns>
        private bool Crossing(Point a, Point b, SegWithIndex[] segsToIgnore) {
            return IsCrossing(new LineSegment(a, b), this.segTree, segsToIgnore);
        }

        /// <summary>
        /// ignoring crossing at ls.Start
        /// </summary>
        /// <param name="ls"></param>
        /// <param name="rTree"></param>
        /// <param name="segsToIgnore"></param>
        /// <returns></returns>
        private static bool IsCrossing(LineSegment ls, RTree<SegWithIndex,Point> rTree, SegWithIndex[] segsToIgnore) {
            return rTree.GetAllIntersecting(ls.BoundingBox).Where(seg => !segsToIgnore.Contains(seg)).Any();
        }

        private bool IntersectObstacleHierarchy(Point a, Point b, Point c) {
            return this.IntersectObstacleHierarchy(new LineSegment(a, b)) ||
                   this.IntersectObstacleHierarchy(new LineSegment(b, c));
        }

        private bool IntersectObstacleHierarchy(LineSegment ls) {
            return
                this.HierarchyOfObstacles.GetAllIntersecting(ls.BoundingBox).Any(
                    poly => Curve.CurveCurveIntersectionOne(ls, poly, false) != null);
        }

        private bool IsStaircase(Point[] pts, int offset, SegWithIndex[] segsToIgnore, out bool canHaveStaircaseAtI) {
            var a = pts[offset];
            var b = pts[offset + 1];
            var c = pts[offset + 2];
            var d = pts[offset + 3];
            var f = pts[offset + 4];
            canHaveStaircaseAtI = false;
            if (CompassVector.DirectionsFromPointToPoint(a, b) != CompassVector.DirectionsFromPointToPoint(c, d) ||
                CompassVector.DirectionsFromPointToPoint(b, c) != CompassVector.DirectionsFromPointToPoint(d, f)) {
                return false;
            }

            c = GetFlippedPoint(pts, offset);
            if (this.IntersectObstacleHierarchy(b, c, d)) {
                return false;
            }

            canHaveStaircaseAtI = true;
            return !this.Crossing(b, c, segsToIgnore);
        }

        private Point[] RemoveStaircase(Point[] pts, int staircaseStart) {
            Point a = pts[staircaseStart];
            Point b = pts[staircaseStart + 1];
            var horiz = Math.Abs(a.Y - b.Y) < ApproximateComparer.DistanceEpsilon/2;
            return this.RemoveStaircase(pts, staircaseStart, horiz);

        }

        private Point[] RemoveStaircase(Point[] pts, int staircaseStart, bool horiz) {
            this.RemoveSegs(pts);
            var ret = new Point[pts.Length - 2];
            Array.Copy(pts, ret, staircaseStart + 1);
            var a = pts[staircaseStart + 1];
            var c = pts[staircaseStart + 3];
            ret[staircaseStart + 1] = horiz ? new Point(c.X, a.Y) : new Point(a.X, c.Y);
            Array.Copy(pts, staircaseStart + 4, ret, staircaseStart + 2, ret.Length - staircaseStart - 2);
            this.InsertNewSegs(ret, staircaseStart);
            return ret;
        }

        private void RemoveSegs(Point[] pts) {
            for (int i = 0; i < pts.Length-1; i++) {
                this.RemoveSeg(new SegWithIndex(pts,i));
            }
        }

        private void RemoveSeg(SegWithIndex seg) {
            this.segTree.Remove(Rect(seg), seg);
        }

        private void InsertNewSegs(Point[] pts, int staircaseStart) {
            this.InsSeg(pts, staircaseStart);
            this.InsSeg(pts, staircaseStart+1);
        }

        private void InitHierarchies() {
            foreach (var path in this.Paths) {
                this.InsertPathSegs(path);
            }
        }

        private void InsertPathSegs(Path path) {
            this.InsertSegs((Point[])path.PathPoints);
        }

        private void InsertSegs(Point[] pts) {
            for (int i = 0; i < pts.Length - 1; i++) {
                this.InsSeg(pts, i);
            }
        }

        private void InsSeg(Point[] pts, int i) {
            var seg = new SegWithIndex(pts, i);
            this.segTree.Add(Rect(seg), seg);
        }

        private static Rectangle Rect(SegWithIndex seg) {
            return new Rectangle(seg.Start,seg.End);
        }
    }
}