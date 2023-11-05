
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using System.Linq;
using System;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.DebugHelpers;
using System.Diagnostics;
using System.Threading;
using System.Reflection.Emit;
using Microsoft.Msagl.Layout.MDS;

namespace Microsoft.Msagl.Layout.Layered {

    internal class SmoothedPolylineCalculator {
        private CornerSite headSite;// corresponds to the bottom point
        private PolyIntEdge edgePath;
        private Anchor[] anchors;
        private GeometryGraph originalGraph;
        private ParallelogramNode eastHierarchy;
        private ParallelogramNode westHierarchy;
        private ParallelogramNode thinEastHierarchy;
        private ParallelogramNode thinWestHierarchy;
        private List<ParallelogramNode> thinRightNodes = new List<ParallelogramNode>();
        private List<ParallelogramNode> thinLefttNodes = new List<ParallelogramNode>();
        private Database database;
        private ProperLayeredGraph layeredGraph;
        private LayerArrays layerArrays;
        private SugiyamaLayoutSettings settings;

        /// <summary>
        /// Creates a smoothed polyline
        /// </summary>
        internal SmoothedPolylineCalculator(PolyIntEdge edgePathPar, Anchor[] anchorsP, GeometryGraph origGraph, SugiyamaLayoutSettings settings, LayerArrays la, ProperLayeredGraph layerGraph, Database databaseP) {
            this.database = databaseP;
            this.edgePath = edgePathPar;
            this.anchors = anchorsP;
            this.layerArrays = la;
            this.originalGraph = origGraph;
            this.settings = settings;
            this.layeredGraph = layerGraph;
            this.eastHierarchy = this.BuildEastHierarchy();
            this.westHierarchy = this.BuildWestHierarchy();
        }

        private ParallelogramNode BuildEastHierarchy() {
            List<Polyline> boundaryAnchorsCurves = this.FindEastBoundaryAnchorCurves();
            List<ParallelogramNode> l = new List<ParallelogramNode>();
            foreach (Polyline c in boundaryAnchorsCurves) {
                l.Add(c.ParallelogramNodeOverICurve);
            }

            this.thinEastHierarchy = HierarchyCalculator.Calculate(this.thinRightNodes, this.settings.GroupSplit);

            return HierarchyCalculator.Calculate(l, this.settings.GroupSplit);
        }

        private ParallelogramNode BuildWestHierarchy() {
            List<Polyline> boundaryAnchorCurves = this.FindWestBoundaryAnchorCurves();
            List<ParallelogramNode> l = new List<ParallelogramNode>();
            foreach (Polyline a in boundaryAnchorCurves) {
                l.Add(a.ParallelogramNodeOverICurve);
            }

            this.thinWestHierarchy = HierarchyCalculator.Calculate(this.thinLefttNodes, this.settings.GroupSplit);

            return HierarchyCalculator.Calculate(l, this.settings.GroupSplit);
        }

        private List<Polyline> FindEastBoundaryAnchorCurves() {
            List<Polyline> ret = new List<Polyline>();
            int uOffset = 0;

            foreach (int u in this.edgePath) {
                Anchor rightMostAnchor = null;
                foreach (int v in this.LeftBoundaryNodesOfANode(u, Routing.GetNodeKind(uOffset, this.edgePath))) {
                    Anchor a = this.anchors[v];
                    if (rightMostAnchor == null || rightMostAnchor.Origin.X < a.Origin.X) {
                        rightMostAnchor = a;
                    }

                    ret.Add(a.PolygonalBoundary);
                }
                if (rightMostAnchor != null) {
                    this.thinRightNodes.Add(new LineSegment(rightMostAnchor.Origin, this.originalGraph.Left, rightMostAnchor.Y).ParallelogramNodeOverICurve);
                }

                uOffset++;
            }

            //if (Routing.db) {
            //    var l = new List<DebugCurve>();
            //       l.AddRange(db.Anchors.Select(a=>new DebugCurve(100,1,"red", a.PolygonalBoundary)));
            //    l.AddRange(thinRightNodes.Select(n=>n.Parallelogram).Select(p=>new Polyline(p.Vertex(VertexId.Corner), p.Vertex(VertexId.VertexA),
            //        p.Vertex(VertexId.OtherCorner), p.Vertex(VertexId.VertexB))).Select(c=>new DebugCurve(100,3,"brown", c)));
            //    foreach (var le in this.edgePath.LayerEdges)
            //        l.Add(new DebugCurve(100, 1, "blue", new LineSegment(db.anchors[le.Source].Origin, db.anchors[le.Target].Origin)));


            //   LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
            //    // Database(db, thinRightNodes.Select(p=>new Polyline(p.Parallelogram.Vertex(VertexId.Corner), p.Parallelogram.Vertex(VertexId.VertexA),
            //        //p.Parallelogram.Vertex(VertexId.OtherCorner), p.Parallelogram.Vertex(VertexId.VertexB)){Closed=true}).ToArray());
            //}
            return ret;
        }

        private List<Polyline> FindWestBoundaryAnchorCurves() {
            List<Polyline> ret = new List<Polyline>();
            int uOffset = 0;

            foreach (int u in this.edgePath) {
                int leftMost = -1;
                foreach (int v in this.RightBoundaryNodesOfANode(u, Routing.GetNodeKind(uOffset, this.edgePath))) {
                    if (leftMost == -1 || this.layerArrays.X[v] < this.layerArrays.X[leftMost]) {
                        leftMost = v;
                    }

                    ret.Add(this.anchors[v].PolygonalBoundary);
                }
                if (leftMost != -1) {
                    Anchor a = this.anchors[leftMost];
                    this.thinLefttNodes.Add(new LineSegment(a.Origin, this.originalGraph.Right, a.Origin.Y).ParallelogramNodeOverICurve);
                }
                uOffset++;
            }
            return ret;
        }

        private IEnumerable<int> FillRightTopAndBottomVerts(int[] layer, int vPosition, NodeKind nodeKind) {
            double t = 0, b = 0;
            if (nodeKind == NodeKind.Bottom) {
                b = Single.MaxValue;//we don't have bottom boundaries here since they will be cut off
            }
            else if (nodeKind == NodeKind.Top) {
                t = Single.MaxValue;//we don't have top boundaries here since they will be cut off
            }

            int v = layer[vPosition];

            for (int i = vPosition + 1; i < layer.Length; i++) {
                int u = layer[i];
                Anchor anchor = this.anchors[u];
                if (anchor.TopAnchor > t) {
                    if (!this.NodeUCanBeCrossedByNodeV(u, v)) {
                        t = anchor.TopAnchor;
                        if (anchor.BottomAnchor > b) {
                            b = anchor.BottomAnchor;
                        }

                        yield return u;
                    }
                }
                else if (anchor.BottomAnchor > b) {
                    if (!this.NodeUCanBeCrossedByNodeV(u, v)) {
                        b = anchor.BottomAnchor;
                        if (anchor.TopAnchor > t) {
                            t = anchor.TopAnchor;
                        }

                        yield return u;
                    }
                }
            }

        }

        private IEnumerable<int> FillLeftTopAndBottomVerts(int[] layer, int vPosition, NodeKind nodeKind) {
            double t = 0, b = 0;
            if (nodeKind == NodeKind.Top) {
                t = Single.MaxValue; //there are no top vertices - they are cut down by the top boundaryCurve curve       
            } else if (nodeKind == NodeKind.Bottom) {
                b = Single.MaxValue; //there are no bottom vertices - they are cut down by the top boundaryCurve curve
            }

            int v = layer[vPosition];

            for (int i = vPosition - 1; i >= 0; i--) {
                int u = layer[i];
                Anchor anchor = this.anchors[u];
                if (anchor.TopAnchor > t + ApproximateComparer.DistanceEpsilon) {
                    if (!this.NodeUCanBeCrossedByNodeV(u, v)) {
                        t = anchor.TopAnchor;
                        b = Math.Max(b, anchor.BottomAnchor);
                        yield return u;
                    }
                }
                else if (anchor.BottomAnchor > b + ApproximateComparer.DistanceEpsilon) {
                    if (!this.NodeUCanBeCrossedByNodeV(u, v)) {
                        t = Math.Max(t, anchor.TopAnchor);
                        b = anchor.BottomAnchor;
                        yield return u;
                    }
                }
            }
        }

        #region Nodes and edges anylisis
        private bool IsVirtualVertex(int v) {
            return v >= this.originalGraph.Nodes.Count;
        }

        private bool IsLabel(int u) {
            return this.anchors[u].RepresentsLabel;
        }

        private bool NodeUCanBeCrossedByNodeV(int u, int v) {
            if (this.IsLabel(u)) {
                return false;
            }

            if (this.IsLabel(v)) {
                return false;
            }

            if (this.IsVirtualVertex(u) && this.IsVirtualVertex(v) && this.EdgesIntersectSomewhere(u, v)) {
                return true;
            }

            return false;

        }

        private bool EdgesIntersectSomewhere(int u, int v) {
            if (this.UVAreMiddlesOfTheSameMultiEdge(u, v)) {
                return false;
            }

            return this.IntersectAbove(u, v) || this.IntersectBelow(u, v);
        }

        private bool UVAreMiddlesOfTheSameMultiEdge(int u, int v) {
            if (this.database.MultipleMiddles.Contains(u) && this.database.MultipleMiddles.Contains(v)
                && this.SourceOfTheOriginalEdgeContainingAVirtualNode(u) == this.SourceOfTheOriginalEdgeContainingAVirtualNode(v)) {
                return true;
            }

            return false;
        }

        private int SourceOfTheOriginalEdgeContainingAVirtualNode(int u) {
            while (this.IsVirtualVertex(u)) {
                u = this.IncomingEdge(u).Source;
            }

            return u;
        }

        private bool IntersectBelow(int u, int v) {
            while (true) {
                LayerEdge eu = this.OutcomingEdge(u);
                LayerEdge ev = this.OutcomingEdge(v);
                if (this.Intersect(eu, ev)) {
                    return true;
                }

                u = eu.Target;
                v = ev.Target;
                if (!(this.IsVirtualVertex(u) && this.IsVirtualVertex(v))) {
                    if (v == u) {
                        return true;
                    }

                    return false;
                }
            }
        }

        private bool IntersectAbove(int u, int v) {
            while (true) {
                LayerEdge eu = this.IncomingEdge(u);
                LayerEdge ev = this.IncomingEdge(v);
                if (this.Intersect(eu, ev)) {
                    return true;
                }

                u = eu.Source;
                v = ev.Source;
                if (!(this.IsVirtualVertex(u) && this.IsVirtualVertex(v))) {
                    if (u == v) {
                        return true;
                    }

                    return false;
                }

            }

        }

        private bool Intersect(LayerEdge e, LayerEdge m) {
            int a = this.layerArrays.X[e.Source] - this.layerArrays.X[m.Source];
            int b = this.layerArrays.X[e.Target] - this.layerArrays.X[m.Target];
            return a > 0 && b < 0 || a < 0 && b > 0;
            //return (layerArrays.X[e.Source] - layerArrays.X[m.Source]) * (layerArrays.X[e.Target] - layerArrays.X[m.Target]) < 0;
        }
#endregion

#region Node queries
        //here u is a virtual vertex
        private LayerEdge IncomingEdge(int u) {
            return this.layeredGraph.InEdgeOfVirtualNode(u);
        }
        //here u is a virtual vertex
        private LayerEdge OutcomingEdge(int u) {
            return this.layeredGraph.OutEdgeOfVirtualNode(u);
        }

        private IEnumerable<int> RightBoundaryNodesOfANode(int i, NodeKind nodeKind) {
            return this.FillRightTopAndBottomVerts(this.NodeLayer(i), this.layerArrays.X[i], nodeKind);
        }

        private int[] NodeLayer(int i) {
            return this.layerArrays.Layers[this.layerArrays.Y[i]];
        }

        private IEnumerable<int> LeftBoundaryNodesOfANode(int i, NodeKind nodeKind) {
            return this.FillLeftTopAndBottomVerts(this.NodeLayer(i), this.layerArrays.X[i], nodeKind);
        }
#endregion

        internal ICurve GetSpline(bool optimizeShortEdges) {
            this.CreateRefinedPolyline(optimizeShortEdges);
            return this.CreateSmoothedPolyline();
        }

        #region debug stuff
#if TEST_MSAGL
        //   static int calls;
        // bool debug { get { return calls == 5;} }

        private Curve Poly() {
            Curve c = new Curve();
            for (CornerSite s = this.headSite; s.Next != null; s = s.Next) {
                c.AddSegment(new CubicBezierSegment(s.Point, 2 * s.Point / 3 + s.Next.Point / 3, s.Point / 3 + 2 * s.Next.Point / 3, s.Next.Point));
            }

            return c;
        }
#endif

#endregion

        internal SmoothedPolyline GetPolyline {
            get {
                System.Diagnostics.Debug.Assert(this.headSite != null);
                return new SmoothedPolyline(this.headSite);
            }
        }

        private bool LineSegIntersectBound(Point a, Point b) {
            var l = new LineSegment(a, b);
            return CurveIntersectsHierarchy(l, this.westHierarchy) || CurveIntersectsHierarchy(l, this.thinWestHierarchy) ||
                CurveIntersectsHierarchy(l, this.eastHierarchy) || CurveIntersectsHierarchy(l, this.thinEastHierarchy);
        }

        private bool SegIntersectEastBound(CornerSite a, CornerSite b) {
            return SegIntersectsBound(a, b, this.westHierarchy) || SegIntersectsBound(a, b, this.thinWestHierarchy);
        }

        private bool SegIntersectWestBound(CornerSite a, CornerSite b) {
            return SegIntersectsBound(a, b, this.eastHierarchy) || SegIntersectsBound(a, b, this.thinEastHierarchy);
        }

        private CornerSite TryToRemoveInflectionEdge(CornerSite s, out bool cut) {

            if (s.Next == null || s.Previous == null) {
                cut = false;
                return s.Next;
            }

            if ((s.Turn > 0 && this.SegIntersectEastBound(s.Previous, s.Next)) || (s.Turn < 0 && this.SegIntersectWestBound(s.Previous, s.Next))) {
                cut = false;
                return s.Next;
            }
            CornerSite ret = s.Next;
            s.Previous.Next = s.Next;                  //forget about s
            s.Next.Previous = s.Previous;
            cut = true;
            return ret;

        }

        private static bool SegIntersectsBound(CornerSite a, CornerSite b, ParallelogramNode hierarchy) {
            return CurveIntersectsHierarchy(new LineSegment(a.Point, b.Point), hierarchy);
        }

        private static bool CurveIntersectsHierarchy(LineSegment lineSeg, ParallelogramNode hierarchy) {
            if (hierarchy == null) {
                return false;
            }

            if (!Parallelogram.Intersect(lineSeg.ParallelogramNodeOverICurve.Parallelogram, hierarchy.Parallelogram)) {
                return false;
            }

            ParallelogramBinaryTreeNode n = hierarchy as ParallelogramBinaryTreeNode;
            if (n != null) {
                return CurveIntersectsHierarchy(lineSeg, n.LeftSon) || CurveIntersectsHierarchy(lineSeg, n.RightSon);
            }

            return Curve.CurveCurveIntersectionOne(lineSeg, ((ParallelogramNodeOverICurve)hierarchy).Seg, false) != null;


        }

        private static bool Flat(CornerSite i) {
            return Point.GetTriangleOrientation(i.Previous.Point, i.Point, i.Next.Point) == TriangleOrientation.Collinear;
        }

        internal SmoothedPolylineCalculator Reverse() {
            SmoothedPolylineCalculator ret = new SmoothedPolylineCalculator(this.edgePath, this.anchors, this.originalGraph, this.settings, this.layerArrays, this.layeredGraph, this.database);
            CornerSite site = this.headSite;
            CornerSite v = null;
            while (site != null) {
                ret.headSite = site.Clone();
                ret.headSite.Next = v;
                if (v != null) {
                    v.Previous = ret.headSite;
                }

                v = ret.headSite;
                site = site.Next;
            }
            return ret;
        }

        private void CreateRefinedPolyline(bool optimizeShortEdges) {
            this.CreateInitialListOfSites();

            CornerSite topSite = this.headSite;
            CornerSite bottomSite;
            for (int i = 0; i < this.edgePath.Count; i++) {
                bottomSite = topSite.Next;
                this.RefineBeetweenNeighborLayers(topSite, this.EdgePathNode(i), this.EdgePathNode(i + 1));
                topSite = bottomSite;
            }
            this.TryToRemoveInflections();
            if (optimizeShortEdges) {
                this.OptimizeShortPath();
            }
        }

        private void RefineBeetweenNeighborLayers(CornerSite topSite, int topNode, int bottomNode) {
            RefinerBetweenTwoLayers.Refine(topNode, bottomNode, topSite, this.anchors,
                                           this.layerArrays, this.layeredGraph, this.originalGraph,
                                           this.settings.LayerSeparation);
        }

        private void CreateInitialListOfSites() {
            CornerSite currentSite = this.headSite = new CornerSite(this.EdgePathPoint(0));
            for (int i = 1; i <= this.edgePath.Count; i++) {
                currentSite = new CornerSite(currentSite, this.EdgePathPoint(i));
            }
        }

        private CornerSite TailSite { get { CornerSite s = this.headSite; while (s.Next != null) { s = s.Next; } return s; } }

        private void OptimizeForThreeSites() {
            Debug.Assert(this.edgePath.LayerEdges.Count == 2);
            int top = this.EdgePathNode(0);
            int bottom = this.EdgePathNode(2);
            Anchor a = this.anchors[top];
            Anchor b = this.anchors[bottom];
            double ax = a.X;
            double bx = b.X;
            if (ApproximateComparer.Close(ax, bx)) {
                return;
            }

            int sign;
            if (!this.FindLegalPositions(a, b, ref ax, ref bx, out sign)) {
                return;
            }

            double ratio = (a.Y - b.Y) / (a.Bottom - b.Top);
            double xc = 0.5 * (ax + bx);
            double half = sign * (ax - bx) * 0.5;
            ax = xc + ratio * half * sign;
            bx = xc - ratio * half * sign;

            this.headSite.Point = new Point(ax, a.Y);
            var ms = this.headSite.Next;
            double mY = ms.Point.Y;
            ms.Point = new Point(this.MiddlePos(ax, bx, a, b, mY), mY);
            ms.Next.Point = new Point(bx, b.Y);
            Anchor ma = this.anchors[this.EdgePathNode(1)];
            ma.X = ms.Point.X;
            //show(new DebugCurve(200, 3, "yellow", new LineSegment(ax, a.Y, ms.Point.X, ms.Point.Y)),
            //    new DebugCurve(200, 3, "green", new LineSegment(bx, b.Y, ms.Point.X, ms.Point.Y)));
        }

        private void OptimizeForTwoSites() {
            Debug.Assert(this.edgePath.LayerEdges.Count == 1);
            int top = this.EdgePathNode(0);
            int bottom = this.EdgePathNode(1);
            Anchor a = this.anchors[top];
            Anchor b = this.anchors[bottom];
            double ax = a.X;
            double bx = b.X;
            if (ApproximateComparer.Close(ax, bx)) {
                return;
            }

            int sign;
            if (!this.FindPositions(a, b, ref ax, ref bx, out sign)) {
                return;
            }

            double ratio = (a.Y - b.Y) / (a.Bottom - b.Top);
            double xc = 0.5 * (ax + bx);
            double half = sign * (ax - bx) * 0.5;
            ax = xc + ratio * half * sign;
            bx = xc - ratio * half * sign;

            this.headSite.Point = new Point(ax, a.Y);
            this.headSite.Next.Point = new Point(bx, b.Y);
        }

        private bool FindLegalPositions(Anchor a, Anchor b, ref double ax, ref double bx, out int sign) {
            if (!this.FindPositions(a, b, ref ax, ref bx, out sign)) {
                return false;
            }

            return this.PositionsAreLegal(ax, bx, sign, a, b, this.EdgePathNode(1));                 
        }

        private bool FindPositions(Anchor a, Anchor b, ref double ax, ref double bx, out int sign) {
            double overlapMin, overlapMax;

            if (ax < bx) {
                sign = 1;
                overlapMin = Math.Max(ax, b.Left);
                overlapMax = Math.Min(a.Right, bx);
            }
            else {
                sign = -1;
                overlapMin = Math.Max(a.Left, bx);
                overlapMax = Math.Min(b.Right, ax); ;
            }
            if (overlapMin <= overlapMax) {
                ax = bx = 0.5 * (overlapMin + overlapMax);
            }
            else {
                if (this.OriginToOriginSegCrossesAnchorSide(a, b)) {
                    return false;
                }

                if (sign == 1) {
                    ax = a.Right - 0.1 * a.RightAnchor;
                    bx = b.Left;
                }
                else {
                    ax = a.Left + 0.1 * a.LeftAnchor;
                    bx = b.Right;
                }
            }
            return true;
        }

        private bool OriginToOriginSegCrossesAnchorSide(Anchor a, Anchor b) {
            Debug.Assert(a.Y > b.Y);
            var seg = new LineSegment(a.Origin, b.Origin);
            return (a.X < b.X
                &&
                Curve.CurvesIntersect(seg, new LineSegment(a.RightBottom, a.RightTop))
                    ||
                    Curve.CurvesIntersect(seg, new LineSegment(b.LeftBottom, a.LeftTop)))
                    ||
                    (a.X > b.X
                    &&
                    Curve.CurvesIntersect(seg, new LineSegment(a.LeftBottom, a.LeftTop))
                    ||
                    Curve.CurvesIntersect(seg, new LineSegment(b.RightBottom, a.RightTop)));
        }


        private void OptimizeShortPath() {
            if (this.edgePath.Count > 2) {
                return;
            }

            if (this.edgePath.Count == 2 && this.headSite.Next.Next != null && this.headSite.Next.Next.Next == null && this.anchors[this.EdgePathNode(1)].Node == null) {
                this.OptimizeForThreeSites();
            }
            else {
                if (this.edgePath.Count == 1) {
                    this.OptimizeForTwoSites();
                }
            }
        }

#if !SHARPKIT
        //void show(params DebugCurve[] cs) {
        //    var l = new List<DebugCurve>();
        //    l.AddRange(anchors.Select(aa => new DebugCurve(100, 1, "red", aa.PolygonalBoundary)));
        //    l.AddRange(thinRightNodes.Select(n => n.Parallelogram).Select(p => new Polyline(p.Vertex(VertexId.Corner), p.Vertex(VertexId.VertexA),
        //          p.Vertex(VertexId.OtherCorner), p.Vertex(VertexId.VertexB))).Select(c => new DebugCurve(100, 3, "brown", c)));
        //    foreach (var le in this.edgePath.LayerEdges)
        //        l.Add(new DebugCurve(100, 1, "blue", new LineSegment(anchors[le.Source].Origin, anchors[le.Target].Origin)));
        //    l.AddRange(cs);
        //    LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        //    // Database(db, thinRightNodes.Select(p=>new Polyline(p.Parallelogram.Vertex(VertexId.Corner), p.Parallelogram.Vertex(VertexId.VertexA),
        //    //p.Parallelogram.Vertex(VertexId.OtherCorner), p.Parallelogram.Vertex(VertexId.VertexB)){Closed=true}).ToArray());
        //}
#endif

        private bool PositionsAreLegal(double sax, double sbx, int sign, Anchor a, Anchor b, int middleNodeIndex) {

            if (!ApproximateComparer.Close(sax, sbx) && (sax - sbx) * sign > 0) {
                return false;
            }

            Anchor mAnchor = this.anchors[middleNodeIndex];
            double mx = this.MiddlePos(sax, sbx, a, b, mAnchor.Y);
            if (!this.MiddleAnchorLegal(mx, middleNodeIndex, mAnchor)) {
                return false;
            }

            return !this.LineSegIntersectBound(new Point(sax, a.Bottom), new Point(sbx, b.Top));
        }

        private bool MiddleAnchorLegal(double mx, int middleNodeIndex, Anchor mAnchor) {
            var mLayer = this.NodeLayer(middleNodeIndex);
            int pos = this.layerArrays.X[middleNodeIndex];
            double shift = mx - mAnchor.X;
            if (pos > 0) {
                Anchor l = this.anchors[mLayer[pos - 1]];
                if (l.Right > shift + mAnchor.Left) {
                    return false;
                }
            }
            if (pos < mLayer.Length - 1) {
                Anchor r = this.anchors[mLayer[pos + 1]];
                if (r.Left < shift + mAnchor.Right) {
                    return false;
                }
            }
            return true;
        }

        private double MiddlePos(double sax, double sbx, Anchor a, Anchor b, double mY) {
            double u = a.Y - mY;
            double l = mY - b.Y;
            Debug.Assert(u >= 0 && l >= 0);
            return (sax * u + sbx * l) / (u + l);
        }

        private void TryToRemoveInflections() {

            if (this.TurningAlwaySameDirection()) {
                return;
            }

            bool progress = true;
            while (progress) {
                progress = false;
                for (CornerSite s = this.headSite; s != null; ) {
                    bool cut;
                    s = this.TryToRemoveInflectionEdge(s, out cut);
                    progress |= cut;
                }
            }
        }

        private bool TurningAlwaySameDirection() {
            int sign = 0;//undecided
            for (CornerSite s = this.headSite.Next; s != null && s.Next != null; s = s.Next) {
                double nsign = s.Turn;
                if (sign == 0) {//try to set the sign
                    if (nsign > 0) {
                        sign = 1;
                    } else if (nsign < 0) {
                        sign = -1;
                    }
                }
                else {
                    if (sign * nsign < 0) {
                        return false;
                    }
                }
            }
            return true;
        }



        #region Edge path node access
        private Point EdgePathPoint(int i) {
            return this.anchors[this.EdgePathNode(i)].Origin;
        }

        private int EdgePathNode(int i) {
            int v;
            if (i == this.edgePath.Count) {
                v = this.edgePath[this.edgePath.Count - 1].Target;
            } else {
                v = this.edgePath[i].Source;
            }

            return v;
        }
        #endregion

        #region Fitting Bezier segs
        private Curve CreateSmoothedPolyline() {
            this.RemoveVerticesWithNoTurns();
            Curve curve = new Curve();
            CornerSite a = this.headSite;//the corner start
            CornerSite b; //the corner origin
            CornerSite c;//the corner other end
            if (Curve.FindCorner(a, out b, out c)) {
                this.CreateFilletCurve(curve, ref a, ref b, ref c);
                curve = this.ExtendCurveToEndpoints(curve);
            }
            else {
                curve.AddSegment(new LineSegment(this.headSite.Point, this.TailSite.Point));
            }
            return curve;
        }

        private void RemoveVerticesWithNoTurns() {

            while (this.RemoveVerticesWithNoTurnsOnePass()) {
                ;
            }
        }

        private bool RemoveVerticesWithNoTurnsOnePass() {

            bool ret = false;
            for (CornerSite s = this.headSite; s.Next != null && s.Next.Next != null; s = s.Next) {
                if (Flat(s.Next)) {
                    ret = true;
                    s.Next = s.Next.Next;//crossing out s.next
                    s.Next.Previous = s;
                }
            }

            return ret;
        }

        private Curve ExtendCurveToEndpoints(Curve curve) {
            Point p = this.headSite.Point;
            if (!ApproximateComparer.Close(p, curve.Start)) {
                Curve nc = new Curve();
                nc.AddSegs(new LineSegment(p, curve.Start), curve);
                curve = nc;
            }
            p = this.TailSite.Point;
            if (!ApproximateComparer.Close(p, curve.End)) {
                curve.AddSegment(new LineSegment(curve.End, p));
            }

            return curve;
        }

        private void CreateFilletCurve(Curve curve, ref CornerSite a, ref CornerSite b, ref CornerSite c) {
            do {
                this.AddSmoothedCorner(a, b, c, curve);
                a = b;
                b = c;
                if (b.Next != null) {
                    c = b.Next;
                } else {
                    break;
                }
            } while (true);
        }

        private void AddSmoothedCorner(CornerSite a, CornerSite b, CornerSite c, Curve curve) {
            double k = 0.5;
            CubicBezierSegment seg;
            do {
                seg = Curve.CreateBezierSeg(k, k, a, b, c);
                //if (Routing.db)
                //    LayoutAlgorithmSettings .Show(seg, CreatePolyTest());
                b.PreviousBezierSegmentFitCoefficient = k;
                k /= 2;
            } while (this.BezierSegIntersectsBoundary(seg));

            k *= 2; //that was the last k
            if (k < 0.5) {//one time try a smoother seg
                k = 0.5 * (k + k * 2);
                CubicBezierSegment nseg = Curve.CreateBezierSeg(k, k, a, b, c);
                if (!this.BezierSegIntersectsBoundary(nseg)) {
                    b.PreviousBezierSegmentFitCoefficient = b.NextBezierSegmentFitCoefficient = k;
                    seg = nseg;
                }
            }

            if (curve.Segments.Count > 0 && !ApproximateComparer.Close(curve.End, seg.Start)) {
                curve.AddSegment(new LineSegment(curve.End, seg.Start));
            }

            curve.AddSegment(seg);
        }

        private bool BezierSegIntersectsBoundary(CubicBezierSegment seg) {
            double side = Point.SignedDoubledTriangleArea(seg.B(0), seg.B(1), seg.B(2));
            if (side > 0) {
                return this.BezierSegIntersectsTree(seg, this.thinWestHierarchy) || this.BezierSegIntersectsTree(seg, this.westHierarchy);
            } else {
                return this.BezierSegIntersectsTree(seg, this.thinEastHierarchy) || this.BezierSegIntersectsTree(seg, this.eastHierarchy);
            }
        }

        private bool BezierSegIntersectsTree(CubicBezierSegment seg, ParallelogramNode tree) {
            if (tree == null) {
                return false;
            }

            if (Parallelogram.Intersect(seg.ParallelogramNodeOverICurve.Parallelogram, tree.Parallelogram)) {
                ParallelogramBinaryTreeNode n = tree as ParallelogramBinaryTreeNode;
                if (n != null) {
                    return this.BezierSegIntersectsTree(seg, n.LeftSon) || this.BezierSegIntersectsTree(seg, n.RightSon);
                }
                else {
                    return BezierSegIntersectsBoundary(seg, ((ParallelogramNodeOverICurve)tree).Seg);
                }
            }
            else {
                return false;
            }
        }

        private static bool BezierSegIntersectsBoundary(CubicBezierSegment seg, ICurve curve) {
            foreach (IntersectionInfo x in Curve.GetAllIntersections(seg, curve, false)) {
                Curve c = curve as Curve;
                if (c != null) {
                    if (Curve.RealCutWithClosedCurve(x, c, false)) {
                        return true;
                    }
                }
                else {
                    //curve is a line from a thin hierarchy that's forbidden to touch
                    return true;
                }
            }
            return false;
        }

        //static internal CubicBezierSegment CreateBezierSeg(double k, Site a, Site b, Site c) {
        //    Point t = (1 - k) * b.Point;
        //    Point s = k * a.Point + t;
        //    Point e = k * c.Point + t;
        //    t = (2.0 / 3.0) * b.Point;
        //    return new CubicBezierSegment(s, t + s / 3.0, t + e / 3.0, e);
        //}

#endregion
    }
}
