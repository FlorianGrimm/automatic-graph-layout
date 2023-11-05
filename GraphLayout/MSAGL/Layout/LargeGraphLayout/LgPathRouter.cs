using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.GraphmapsWithMesh;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Rectilinear.Nudging;
using Microsoft.Msagl.Routing.Visibility;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
namespace Microsoft.Msagl.Layout.LargeGraphLayout
{
    internal class LgPathRouter
    {
        private VisibilityGraph _visGraph; // = new VisibilityGraph();
        internal const double searchEps = 1e-5;
        private readonly RTree<VisibilityVertex, Point> _visGraphVerticesTree = new RTree<VisibilityVertex, Point>();
        private readonly Dictionary<VisibilityEdge, int> _usedEdges = new Dictionary<VisibilityEdge, int>();
        private readonly Set<VisibilityEdge> _edgesOnOldTrajectories = new Set<VisibilityEdge>();
        internal VisibilityGraph VisGraph
        {
            get { return this._visGraph; }
            set { this._visGraph = value; }
        }

        internal LgPathRouter()
        {
            this._visGraph = new VisibilityGraph();
        }

        internal bool IsOnOldTrajectory(VisibilityEdge e)
        {
            return this._edgesOnOldTrajectories.Contains(e);
        }

        internal void MarkEdgeUsed(Point p0, Point p1)
        {
            var e = this.FindEdge(p0, p1);
            int usage;
            if (!this._usedEdges.TryGetValue(e, out usage)) {
                this._usedEdges[e] = 1;
            } else {
                this._usedEdges[e]++;
            }
        }

        internal void MarkEdgeAsEdgeOnOldTrajectory(Point p0, Point p1)
        {
            var e = this.FindEdge(p0, p1);
            this._edgesOnOldTrajectories.Insert(e);
        }

        internal bool IsEdgeUsed(VisibilityEdge e)
        {
            int usage;
            if (!this._usedEdges.TryGetValue(e, out usage)) {
                return false;
            }

            return usage > 0;
        }

        internal void MarkEdgesUsedAlongPath(List<Point> path)
        {
            for (var i = 0; i < path.Count - 1; i++) {
                this.MarkEdgeUsed(path[i], path[i + 1]);
            }
        }

        internal void MarkEdgesAlongPathAsEdgesOnOldTrajectories(List<Point> path)
        {
            for (var i = 0; i < path.Count - 1; i++) {
                this.MarkEdgeAsEdgeOnOldTrajectory(path[i], path[i + 1]);
            }
        }

        internal VisibilityEdge AddVisGraphEdge(Point ep0, Point ep1)
        {
            var v0 = this.GetOrFindVisibilityVertex(ep0) ?? this.AddNewVertex(ep0);
            var v1 = this.GetOrFindVisibilityVertex(ep1) ?? this.AddNewVertex(ep1);
            return VisibilityGraph.AddEdge(v0, v1);
        }


        internal void AddVisGraphEdgesFromNodeCenterToNodeBorder(LgNodeInfo nodeInfo)
        {
            var vc = this.VisGraph.AddVertex(nodeInfo.Center);
            vc.IsTerminal = true; // we don't need to register this node in the tree
            foreach (var pt in nodeInfo.BoundaryOnLayer.PolylinePoints)
            {
                var vv = this.GetOrFindVisibilityVertex(pt.Point);
                var edge = VisibilityGraph.AddEdge(vc, vv);
                edge.IsPassable = () => this.EdgeIsPassable(edge);
            }
        }

        internal void RemoveVisGraphVertex(Point p)
        {
            var v = this._visGraph.FindVertex(p);
            if (v == null) {
                return;
            }

            this._visGraph.RemoveVertex(v);
            this._visGraphVerticesTree.Remove(new Rectangle(p), v);
        }

        internal VisibilityVertex GetOrFindVisibilityVertex(Point p)
        {
            var v = this._visGraph.FindVertex(p);
            if (v != null) {
                return v;
            }

            v = this._visGraph.AddVertex(p);
            this.RegisterInTree(v);
            return v;
        }

        internal void ModifySkeletonWithNewBoundaryOnLayer(LgNodeInfo nodeInfo)
        {
            Dictionary<VisibilityEdge, VisibilityVertex> edgeSnapMap = this.GetEdgeSnapAtVertexMap(nodeInfo);
            foreach (var t in edgeSnapMap)
            {
                VisibilityEdge edge = t.Key;
                VisibilityGraph.RemoveEdge(edge);
                var midleV = edgeSnapMap[edge];
                if (nodeInfo.Center != edge.SourcePoint) {
                    VisibilityGraph.AddEdge(edge.Source, midleV);
                } else {
                    VisibilityGraph.AddEdge(midleV, edge.Target);
                }
            }
        }

        private static void SortPointByAngles(LgNodeInfo nodeInfo, Point[] polySplitArray)
        {
            var angles = new double[polySplitArray.Length];
            for (int i = 0; i < polySplitArray.Length; i++) {
                angles[i] = Point.Angle(new Point(1, 0), polySplitArray[i] - nodeInfo.Center);
            }

            Array.Sort(angles, polySplitArray);
        }

        private VisibilityVertex GlueOrAddToPolylineAndVisGraph(Point[] polySplitArray, int i, VisibilityVertex v, Polyline poly)
        {
            var ip = polySplitArray[i];
            if (ApproximateComparer.Close(v.Point, ip)) {
                return v; // gluing ip to the previous point on the polyline
            }

            if (ApproximateComparer.Close(ip, poly.StartPoint.Point))
            {
                var vv = this.VisGraph.FindVertex(poly.StartPoint.Point);
                Debug.Assert(vv != null);
                return vv;
            }
            poly.AddPoint(ip);
            return this.VisGraph.AddVertex(ip);
        }

        private Dictionary<VisibilityEdge, VisibilityVertex> GetEdgeSnapAtVertexMap(LgNodeInfo nodeInfo)
        {
            var ret = new Dictionary<VisibilityEdge, VisibilityVertex>();
            var center = nodeInfo.Center;
            RbTree<VisibilityVertex> nodeBoundaryRbTree =
                new RbTree<VisibilityVertex>((a, b) => this.CompareByAngleFromNodeCenter(a, b, center));
            foreach (var p in nodeInfo.BoundaryOnLayer) {
                nodeBoundaryRbTree.Insert(this._visGraph.AddVertex(p));
            }

            var nodeInfoCenterV = this.VisGraph.FindVertex(center);
            if (nodeInfoCenterV == null) {
                return ret;
            }

            foreach (var e in nodeInfoCenterV.OutEdges) {
                this.SnapToAfterBefore(e.Target, nodeBoundaryRbTree, center, ret, e);
            }

            foreach (var e in nodeInfoCenterV.InEdges) {
                this.SnapToAfterBefore(e.Source, nodeBoundaryRbTree, center, ret, e);
            }

            return ret;
        }

        private void SnapToAfterBefore(VisibilityVertex v, RbTree<VisibilityVertex> nodeBoundaryRbTree, Point center, Dictionary<VisibilityEdge, VisibilityVertex> ret, VisibilityEdge e)
        {
            VisibilityVertex beforeV, afterV;
            this.FindBeforeAfterV(v, nodeBoundaryRbTree, out beforeV, out afterV, center);
            var beforeAngle = Point.Angle(beforeV.Point - center, v.Point - center);
            var afterAngle = Point.Angle(v.Point - center, afterV.Point - center);
            ret[e] = beforeAngle <= afterAngle ? beforeV : afterV;
        }

        private void FindBeforeAfterV(VisibilityVertex v, RbTree<VisibilityVertex> nodeBoundaryRbTree,
            out VisibilityVertex beforeV, out VisibilityVertex afterV, Point center)
        {
            Point xDir = new Point(1, 0);
            var vAngle = Point.Angle(xDir, v.Point - center);
            var rNode = nodeBoundaryRbTree.FindLast(w => Point.Angle(xDir, w.Point - center) <= vAngle);
            beforeV = rNode != null ? rNode.Item : nodeBoundaryRbTree.TreeMaximum().Item;
            rNode = nodeBoundaryRbTree.FindFirst(w => Point.Angle(xDir, w.Point - center) >= vAngle);
            afterV = rNode != null ? rNode.Item : nodeBoundaryRbTree.TreeMinimum().Item;
        }

        private int CompareByAngleFromNodeCenter(VisibilityVertex a, VisibilityVertex b, Point center)
        {
            var x = new Point(1, 0);
            var aAngle = Point.Angle(x, a.Point - center);
            var bAngle = Point.Angle(x, b.Point - center);
            return aAngle.CompareTo(bAngle);
        }

        private VisibilityVertex AddNewVertex(Point p)
        {
            var v = this._visGraph.AddVertex(p);
            this.RegisterInTree(v);
            return v;
        }

        internal Point AddVisGraphVertex(Point p)
        {
            if (this._visGraph.ContainsVertex(p)) {
                return p;
            }

            this.AddNewVertex(p);
            return p;
        }

        internal Point[] GetPortVertices(LgNodeInfo node)
        {
            return node.BoundaryOnLayer.PolylinePoints.Select(pp => pp.Point).ToArray();
        }

        internal List<Point> GetPath(VisibilityVertex vs, VisibilityVertex vt,
            bool shrinkDistances)
        {
            var pathPoints = new List<Point>();

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = true;
            this._visGraph.ClearPrevEdgesTable();
            var router = new SingleSourceSingleTargetShortestPathOnVisibilityGraph(this._visGraph, vs, vt)
            {
                LengthMultiplier = 0.8,
                LengthMultiplierForAStar = 0.3
            };
            var vpath = router.GetPath(shrinkDistances);
            if (vpath == null)
            {
                // seeing a null path
                vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = false;
                return pathPoints;
            }
            var path = vpath.ToList();
            for (int i = 0; i < path.Count(); i++) {
                pathPoints.Add(path[i].Point);
            }

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = false;
            return pathPoints;
        }
        internal List<Point> GetPath(VisibilityVertex vs, VisibilityVertex vt,
            bool shrinkDistances, Tiling g)
        {
            var pathPoints = new List<Point>();

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = true;
            this._visGraph.ClearPrevEdgesTable();
            var router = new SingleSourceSingleTargetShortestPathOnVisibilityGraph(this._visGraph, vs, vt, g)
            {
                LengthMultiplier = 0.8,
                LengthMultiplierForAStar = 0.3
            };
            var vpath = router.GetPath(shrinkDistances);
            if (vpath == null)
            {
                // seeing a null path
                vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = false;
                return pathPoints;
            }
            var path = vpath.ToList();
            for (int i = 0; i < path.Count(); i++) {
                pathPoints.Add(path[i].Point);
            }

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = false;
            return pathPoints;
        }


        internal List<VisibilityEdge> GetEdgesOfPath(List<Point> pathPoints)
        {
            var edges = new List<VisibilityEdge>();
            for (int i = 0; i < pathPoints.Count - 1; i++)
            {
                var v0 = this.GetOrFindVisibilityVertex(pathPoints[i]);
                if (v0 == null) {
                    continue;
                }

                var v1 = this.GetOrFindVisibilityVertex(pathPoints[i + 1]);
                if (v1 == null) {
                    continue;
                }

                var edge = this._visGraph.FindEdge(v0.Point, v1.Point);
                if (edge != null) {
                    edges.Add(edge);
                }
            }
            return edges;
        }

        internal VisibilityEdge FindEdge(Point p1, Point p2)
        {
            var v0 = this.FindVertex(p1);
            if (v0 == null) {
                return null;
            }

            var v1 = this.FindVertex(p2);
            return v1 == null ? null : this._visGraph.FindEdge(v0.Point, v1.Point);
        }

        internal List<Point> GetPath(LgNodeInfo s, LgNodeInfo t, bool shrinkDistances)
        {
            var vs = this.VisGraph.FindVertex(s.Center);
            Debug.Assert(vs != null);
            var vt = this.VisGraph.FindVertex(t.Center);
            Debug.Assert(vt != null);
            return this.GetPath(vs, vt, shrinkDistances);
        }


        internal List<Point> GetPath(LgNodeInfo s, LgNodeInfo t, bool shrinkDistances, Tiling g)
        {
            var vs = this.VisGraph.FindVertex(g.nodeToLoc[s.GeometryNode]);
            Debug.Assert(vs != null);
            var vt = this.VisGraph.FindVertex(g.nodeToLoc[t.GeometryNode]);
            Debug.Assert(vt != null);
            return this.GetPath(vs, vt, shrinkDistances, g);
        }
        internal List<VisibilityVertex> GetAllVertices()
        {
            return (this._visGraph.Vertices()).ToList();
        }

        internal List<Point> GetPointsOfVerticesOverlappingSegment(Point p0, Point p1)
        {
            var v0 = this.VisGraph.FindVertex(p0);
            var v1 = this.VisGraph.FindVertex(p1);
            return this.GetPath(v0, v1, false); //don't shrink weights here!
        }


        internal IEnumerable<VisibilityEdge> GetAllEdgesVisibilityEdges()
        {
            return this._visGraph.Edges;
        }


        internal bool ExistsEdge(Point s, Point t)
        {
            var p0 = this.GetOrFindVisibilityVertex(s);
            if (p0 == null) {
                return false;
            }

            var p1 = this.GetOrFindVisibilityVertex(t);
            if (p1 == null) {
                return false;
            }

            return this._visGraph.FindEdge(p0.Point, p1.Point) != null;
        }

        internal List<Point> GetPathOnSavedTrajectory(LgNodeInfo s, LgNodeInfo t, List<Point> trajectory,
            bool shrinkDistances)
        {
            if (trajectory == null || trajectory.Count < 2) {
                return this.GetPath(s, t, true);
            }

            var path = new List<Point> { trajectory[0] };
            for (int i = 0; i < trajectory.Count - 1; i++)
            {
                var p0 = trajectory[i];
                var p1 = trajectory[i + 1];
                var refinedPath = this.GetPointsOfVerticesOverlappingSegment(p0, p1);
                for (int j = 1; j < refinedPath.Count; j++) {
                    path.Add(refinedPath[j]);
                }
            }

            return path;
        }

        internal bool HasCycles(Point rootPoint)
        {
            var visited = new Set<Point>();
            var parent = new Dictionary<Point, Point>();

            var rp = this.AddVisGraphVertex(rootPoint);
            visited.Insert(rp);
            parent[rp] = rp;

            var queue = new List<Point> { rp };

            while (queue.Any())
            {
                var p = queue.First();
                queue.Remove(p);

                var v = this._visGraph.FindVertex(p);
                var neighb = new Set<Point>();
                neighb.InsertRange(v.OutEdges.Select(e => e.TargetPoint));
                neighb.InsertRange(v.InEdges.Select(e => e.SourcePoint));
                neighb.Remove(parent[p]);

                foreach (var q in neighb)
                {
                    parent[q] = p;
                    if (visited.Contains(q)) {
                        return true;
                    }

                    visited.Insert(q);
                    queue.Add(q);
                }
            }

            return false;
        }

        internal void DecreaseWeightOfEdgesAlongPath(List<Point> oldPath, double d)
        {
            var edges = this.GetEdgesOfPath(oldPath);
            foreach (var edge in edges)
            {
                edge.LengthMultiplier = Math.Min(edge.LengthMultiplier, d);
            }
        }

        internal void AssertEdgesPresentAndPassable(List<Point> path)
        {
            var vs = this.VisGraph.FindVertex(path[0]);
            Debug.Assert(vs != null);
            var vt = this.VisGraph.FindVertex(path[path.Count - 2]);
            Debug.Assert(vt != null);

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = true;

            var router = new SingleSourceSingleTargetShortestPathOnVisibilityGraph(this._visGraph, vs, vt)
            {
                LengthMultiplier = 0.8,
                LengthMultiplierForAStar = 0.0
            };

            List<VisibilityEdge> pathEdges = new List<VisibilityEdge>();
            for (int i = 0; i < path.Count - 1; i++)
            {
                var edge = this.FindEdge(path[i], path[i + 1]);
                Debug.Assert(edge != null);
                pathEdges.Add(edge);
            }

            router.AssertEdgesPassable(pathEdges);

            vs.IsShortestPathTerminal = vt.IsShortestPathTerminal = false;
        }

        internal void SetWeightOfEdgesAlongPathToMin(List<Point> oldPath, double dmin)
        {
            var edges = this.GetEdgesOfPath(oldPath);
            foreach (var edge in edges)
            {
                edge.LengthMultiplier = Math.Max(edge.LengthMultiplier, dmin);
            }
        }

        internal void ResetAllEdgeLengthMultipliers()
        {
            var edges = this._visGraph.Edges;
            foreach (var edge in edges)
            {
                edge.LengthMultiplier = 1;
            }
        }

        internal void SetAllEdgeLengthMultipliersMin(double wmin)
        {
            var edges = this._visGraph.Edges;
            foreach (var edge in edges)
            {
                edge.LengthMultiplier = Math.Max(edge.LengthMultiplier, wmin);
            }
        }

        internal bool ContainsVertex(Point point)
        {
            return this.VisGraph.FindVertex(point) != null;
        }

        internal VisibilityVertex FindVertex(Point point)
        {
            return this.VisGraph.FindVertex(point);
        }

        internal void RemoveEdge(Point a, Point b)
        {
            var e = this.VisGraph.FindEdge(a, b);
            this._usedEdges.Remove(e);

            this._edgesOnOldTrajectories.Remove(e);

            this._visGraph.RemoveEdge(e.Source, e.Target);
        }

        internal void RemoveVisibilityEdges(List<VisibilityEdge> edgesToRemove)
        {
            foreach (var e in edgesToRemove)
            {
                this._visGraph.RemoveEdge(e.Source, e.Target);
                if (e.Source.Degree == 0) {
                    this.RemoveVisGraphVertex(e.SourcePoint);
                }

                if (e.Target.Degree == 0) {
                    this.RemoveVisGraphVertex(e.TargetPoint);
                }
            }
        }

        internal void AddEdges(List<SymmetricSegment> toAdd)
        {
            foreach (var e in toAdd)
            {
                this.AddVisGraphEdge(e.A, e.B);
            }
        }

        private void RegisterInTree(VisibilityVertex vv)
        {
            var rect = new Rectangle(vv.Point);
            VisibilityVertex treeVv;
            if (this._visGraphVerticesTree.OneIntersecting(rect, out treeVv))
            {
                Debug.Assert(treeVv == vv);
                return;
            }
            this._visGraphVerticesTree.Add(rect, vv);
        }

        internal bool EdgeIsPassable(VisibilityEdge e)
        {
            return ((!e.Source.IsTerminal) && (!e.Target.IsTerminal)) ||
                   (e.Source.IsShortestPathTerminal || e.Target.IsShortestPathTerminal);
        }

        internal void ClearUsedEdges()
        {
            this._usedEdges.Clear();
        }

        internal void DiminishUsed(Point a, Point b)
        {
            var e = this.VisGraph.FindEdge(a, b);
            if (e == null) {
                return;
            }

            int usage;
            if (!this._usedEdges.TryGetValue(e, out usage)) {
                return;
            }

            if (usage > 0) {
                this._usedEdges[e]--;
            }
        }

        /// <summary>
        ///  debug
        /// </summary>
        /// <returns></returns>
        public IEnumerable<SymmetricSegment> UsedEdges()
        {
            return this._usedEdges.Where(p => p.Value > 0).Select(p => p.Key).Select(e => new SymmetricSegment(e.SourcePoint, e.TargetPoint));
        }

        public IEnumerable<SymmetricSegment> EdgesOnOldTrajectories()
        {
            var segs = new List<SymmetricSegment>();
            foreach (var edge in this._edgesOnOldTrajectories)
            {
                segs.Add(new SymmetricSegment(edge.SourcePoint, edge.TargetPoint));
            }
            return segs;
        }

        public List<SymmetricSegment> SegmentsNotOnOldTrajectories()
        {
            var segs = new List<SymmetricSegment>();
            foreach (var edge in this.VisGraph.Edges)
            {
                if (!this._edgesOnOldTrajectories.Contains(edge))
                {
                    segs.Add(new SymmetricSegment(edge.SourcePoint, edge.TargetPoint));
                }
            }
            return segs;
        }
    }
}
