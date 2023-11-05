using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.OverlapRemovalFixedSegments;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.Visibility;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    internal class LgSkeletonLevel {
        //internal RTree<Rail,Point> RailTree = new RTree<Rail,Point>();

        //internal Dictionary<SymmetricSegment, Rail> RailDictionary =
        //    new Dictionary<SymmetricSegment, Rail>();

        private readonly RTree<Point,Point> _visGraphVertices = new RTree<Point,Point>();

        internal int ZoomLevel;

        //VisibilityGraph VisGraph = new VisibilityGraph();
        internal LgPathRouter PathRouter = new LgPathRouter();
        private readonly Dictionary<SymmetricTuple<LgNodeInfo>, List<Point>> _edgeTrajectories =
            new Dictionary<SymmetricTuple<LgNodeInfo>, List<Point>>();

        internal Dictionary<SymmetricTuple<LgNodeInfo>, List<Point>> EdgeTrajectories {
            get { return this._edgeTrajectories; }
        }


        internal void AddGraphEdgesFromCentersToPointsOnBorders(IEnumerable<LgNodeInfo> nodeInfos) {
            foreach (var nodeInfo in nodeInfos) {
                this.PathRouter.AddVisGraphEdgesFromNodeCenterToNodeBorder(nodeInfo);
            }
        }


        internal void Clear() {
            this._visGraphVertices.Clear();
            this.PathRouter = new LgPathRouter();
        }

        internal void SetTrajectoryAndAddEdgesToUsed(LgNodeInfo s, LgNodeInfo t, List<Point> path) {
            var t1 = new SymmetricTuple<LgNodeInfo>(s, t);
            if (this._edgeTrajectories.ContainsKey(t1)) {
                return;
            }

            this._edgeTrajectories[t1] = path;
            this.PathRouter.MarkEdgesUsedAlongPath(path);
        }


        internal bool HasSavedTrajectory(LgNodeInfo s, LgNodeInfo t) {
            var t1 = new SymmetricTuple<LgNodeInfo>(s, t);
            return this.EdgeTrajectories.ContainsKey(t1);
        }

        internal List<Point> GetTrajectory(LgNodeInfo s, LgNodeInfo t) {
            List<Point> path;
            var tuple = new SymmetricTuple<LgNodeInfo>(s, t);
            this.EdgeTrajectories.TryGetValue(tuple, out path);
            return path;
        }


        internal void ClearSavedTrajectoriesAndUsedEdges() {
            this._edgeTrajectories.Clear();
            this.PathRouter.ClearUsedEdges();
        }

        internal Set<Point> GetPointsOnSavedTrajectories() {
            var points = new Set<Point>();
            foreach (var edgeTrajectory in this._edgeTrajectories.Values) {
                points.InsertRange(edgeTrajectory);
            }
            return points;
        }

        internal void RemoveUnusedGraphEdgesAndNodes() {
            List<VisibilityEdge> unusedEdges = this.GetUnusedGraphEdges();
            this.PathRouter.RemoveVisibilityEdges(unusedEdges);
        }

        internal List<VisibilityEdge> GetUnusedGraphEdges() {
            return this.PathRouter.GetAllEdgesVisibilityEdges().Where(e => !this.PathRouter.IsEdgeUsed(e)).ToList();
        }

        private IEnumerable<SymmetricSegment> SymSegsOfPointList(List<Point> ps) {
            for (int i = 0; i < ps.Count - 1; i++) {
                yield return new SymmetricSegment(ps[i], ps[i + 1]);
            }
        }

        internal bool RoutesAreConsistent() {
            var usedEdges=new Set<SymmetricSegment>(this.PathRouter.UsedEdges());
            var routesDump =
                new Set<SymmetricSegment>(this._edgeTrajectories.Select(p => p.Value).SelectMany(this.SymSegsOfPointList));
            var visEdgeDump =
                new Set<SymmetricSegment>(
                    this.PathRouter.VisGraph.Edges.Select(e => new SymmetricSegment(e.SourcePoint, e.TargetPoint)));
#if TEST_MSAGL && !SHARPKIT
            var routesOutOfVisGraph = routesDump - visEdgeDump;
            if (routesOutOfVisGraph.Count > 0) {
                SplineRouter.ShowVisGraph(this.PathRouter.VisGraph, null,null, this.Ttt(routesOutOfVisGraph));
            }

#endif
            return routesDump == visEdgeDump && usedEdges==routesDump;
        }

#if TEST_MSAGL
        private IEnumerable<ICurve> Ttt(Set<SymmetricSegment> routesOutOfVisGraph) {
            foreach (var symmetricTuple in routesOutOfVisGraph) {
                yield return new LineSegment(symmetricTuple.A,symmetricTuple.B);
            }
        }
#endif

        internal void RemoveSomeEdgeTrajectories(List<SymmetricTuple<LgNodeInfo>> removeList) {
            foreach (var symmetricTuple in removeList) {
                this.RemoveEdgeTrajectory(symmetricTuple);
            }

            this.RemoveUnusedGraphEdgesAndNodes();
        }

        private void RemoveEdgeTrajectory(SymmetricTuple<LgNodeInfo> symmetricTuple) {
            List<Point> trajectory;
            if (this._edgeTrajectories.TryGetValue(symmetricTuple, out trajectory)) {
                for (int i = 0; i < trajectory.Count - 1; i++) {
                    this.DiminishUsed(trajectory[i], trajectory[i + 1]);
                }
                this._edgeTrajectories.Remove(symmetricTuple);
            }
        }

        private void DiminishUsed(Point a, Point b) {
            this.PathRouter.DiminishUsed(a, b);
        }

        public void MarkEdgesAlongPathAsEdgesOnOldTrajectories(List<Point> trajectory)
        {
            this.PathRouter.MarkEdgesAlongPathAsEdgesOnOldTrajectories(trajectory);
        }
    }
}
