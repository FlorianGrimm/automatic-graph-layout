using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing.Visibility;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    public class RouteSimplifier {
        private readonly LgPathRouter _pathRouter;
        private readonly RTree<LgNodeInfo, Point> _nodesTree = new RTree<LgNodeInfo, Point>();
        private readonly Set<Point> _fixedPoints;
        private readonly RTree<SymmetricSegment, Point> _symmetricSegmentsTree=new RTree<SymmetricSegment, Point>();

        internal RouteSimplifier(LgPathRouter pathRouter, IEnumerable<LgNodeInfo> nodes, Set<Point> fixedPoints) {
            this._pathRouter = pathRouter;
            this._fixedPoints = fixedPoints;
            foreach (var node in nodes) {
                this._nodesTree.Add(node.BoundaryOnLayer.BoundingBox, node);
            }

            foreach (var e in pathRouter.VisGraph.Edges) {
                var ss= new SymmetricSegment(e.SourcePoint, e.TargetPoint);
                this._symmetricSegmentsTree.Add(new Rectangle(ss.A, ss.B), ss);
            }
        }

        private Set<LgNodeInfo> IntersectedByLineSegNodeInfos(Point p1, Point p2) {
            var nodes = this._nodesTree.GetAllIntersecting(new Rectangle(p1, p2));
            return
                new Set<LgNodeInfo>(nodes.Where(node => Curve.CurvesIntersect(node.BoundaryOnLayer, new LineSegment(p1, p2))));
        }

        internal bool Run() {
            // it is critical to enumerate to an array here, because the quiery will change during the foreach loop below
            var candidates =
                new List<VisibilityVertex>(this._pathRouter.GetAllVertices().Where(v => (!this._fixedPoints.Contains(v.Point)) &&
                                                                                 v.Degree == 2 && (!v.IsTerminal))).ToArray();
            bool ret = false;
            foreach (var v in candidates) {
                var neighb = this.GetNeighbors(v);
                if (neighb.Count != 2) {
                    continue;
                }

                Debug.Assert(neighb.Count==2);
                var v0 = neighb[0];
                var v1 = neighb[1];
                if (this.AngleIsTooSmallAfterShortcut(v0, v, v1)) {
                    continue;
                }

                if (this.ShortcutIntersectsNewVerticesOrCuttingInsideNodes(v0, v, v1)) {
                    continue;
                }

                if (!this.KeepsPlanarity(v0.Point, v1.Point)) {
                    continue;
                }

                this.RemoveAction(v0.Point, v.Point);
                this.RemoveAction(v.Point, v1.Point);
                this._pathRouter.RemoveVisGraphVertex(v.Point);
                this.AddAction(v0.Point, v1.Point);
                ret = true;
            }
            return ret;
        }

        private void AddAction(Point a, Point b) {
            Debug.Assert(this._pathRouter.VisGraph.ContainsVertex(a));
            Debug.Assert(this._pathRouter.VisGraph.ContainsVertex(b));
            this._pathRouter.VisGraph.AddEdge(a, b);
            var ss = new SymmetricSegment(a, b);
            this._symmetricSegmentsTree.Add(new Rectangle(a, b), ss);
        }

        private void RemoveAction(Point a, Point b) {
#if TEST_MSAGL
            VisibilityEdge ve;
            Debug.Assert(this._pathRouter.FindVertex(a).TryGetEdge(this._pathRouter.FindVertex(b), out ve));
#endif
            this._pathRouter.RemoveEdge(a, b);
            var ss=new SymmetricSegment(a, b);
            this._symmetricSegmentsTree.Remove(new Rectangle(ss.A, ss.B), ss);
        }

        private Set<SymmetricSegment> GetSymmetricSegmentsIntersectedBySeg(Point a, Point b) {
            var rect = new Rectangle(a, b);
            return new Set<SymmetricSegment>(this._symmetricSegmentsTree.GetAllIntersecting(rect).Where(ss =>
                SymmetricSegmentIntersects(ss, a, b)));            
        }

        private static bool SymmetricSegmentIntersects(SymmetricSegment ss, Point a, Point b) {
            double t0, t1;
            var dist = LineSegment.MinDistBetweenLineSegments(ss.A, ss.B, a, b, out t0, out t1);
            return dist <0.001 && ((0.01 < t0 && t0 < 0.99) || (0.01 < t1 && t1 < 0.99));

        }

        private bool KeepsPlanarity(Point a, Point b) {
            return !this.GetSymmetricSegmentsIntersectedBySeg(a, b).Any();
        }

        private static bool AngleIsTooSmall(double t) {
            const double minAngle = 2 * Math.PI / 17; // make it slightly more than 20 degrees
            Debug.Assert(t >= 0 && 2*Math.PI >= t);
            return t < minAngle || 2*Math.PI - t < minAngle;
        }

        private bool AngleIsTooSmallAfterShortcut(VisibilityVertex v1, VisibilityVertex v2, VisibilityVertex v3) {
            return this.AngleIsTooSmallAfterShortcutAtVertex(v1, v2, v3) || this.AngleIsTooSmallAfterShortcutAtVertex(v3, v2, v1);
        }

        private bool AngleIsTooSmallAfterShortcutAtVertex(VisibilityVertex v1, VisibilityVertex v2, VisibilityVertex v3) {
            foreach (var edge in v1.OutEdges) {
                var t = edge.Target;
                if (t == v3 || t == v2) {
                    continue;
                }
                //LayoutAlgorithmSettings.ShowDebugCurves(new DebugCurve(1, "red", new LineSegment(v3.Point, v1.Point)),
                //new DebugCurve(1, "blue", new LineSegment(v1.Point, t.Point)));
                if (AngleIsTooSmall(Point.Angle(v3.Point, v1.Point, t.Point))) {
                    return true;
                }
            }

            foreach (var edge in v1.InEdges) {
                var t = edge.Source;
                if (t == v3 || t == v2) {
                    continue;
                }

                if (AngleIsTooSmall(Point.Angle(v3.Point, v1.Point, t.Point))) {
                    return true;
                }
            }

            return false;
        }

        private bool ShortcutIntersectsNewVerticesOrCuttingInsideNodes(VisibilityVertex v0, VisibilityVertex v, VisibilityVertex v1) {
            Set<LgNodeInfo> oldIntersected = this.IntersectedByLineSegNodeInfos(v0.Point, v.Point) +
                                             this.IntersectedByLineSegNodeInfos(v.Point, v1.Point);

            Set<LgNodeInfo> newIntersected = this.IntersectedByLineSegNodeInfos(v0.Point, v1.Point);
            if (this.CuttingInsideOfNodes(newIntersected, v0.Point, v1.Point)) {
                return true;
            }

            return !oldIntersected.Contains(newIntersected);
        }

        private bool CuttingInsideOfNodes(Set<LgNodeInfo> newIntersected, Point a, Point b) {
            return newIntersected.Any(v => this.GettingTooCloseToNode(v, a, b));
        }

        private bool GettingTooCloseToNode(LgNodeInfo lgNodeInfo, Point a, Point b) {            
            var center = lgNodeInfo.Center;
            if (a == center || b == center) {
                return false;
            }

            var closestPoint = Point.ClosestPointAtLineSegment(center, a, b);
            var where = Curve.PointRelativeToCurveLocation(closestPoint, lgNodeInfo.BoundaryOnLayer);
            return where == PointLocation.Inside;
        }

        private List<VisibilityVertex> GetNeighbors(VisibilityVertex v) {
            var neighbors = new List<VisibilityVertex>();
            neighbors.AddRange(v.InEdges.Select(e => e.Source));
            neighbors.AddRange(v.OutEdges.Select(e => e.Target));
            return neighbors;
        }
    }
}
