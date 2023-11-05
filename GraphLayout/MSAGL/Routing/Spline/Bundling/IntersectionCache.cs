using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Routing.Visibility;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// Stores intersections between edges, hubs, and obstacles to speed up simulated annealing
    /// </summary>
    internal class IntersectionCache {
        private readonly MetroGraphData metroGraphData;
        private readonly BundlingSettings bundlingSettings;
        private readonly CostCalculator costCalculator;
        private readonly Cdt cdt;

        public IntersectionCache(MetroGraphData metroGraphData, BundlingSettings bundlingSettings, CostCalculator costCalculator, Cdt cdt) {
            Debug.Assert(cdt!=null);
            this.metroGraphData = metroGraphData;
            this.bundlingSettings = bundlingSettings;
            this.costCalculator = costCalculator;
            this.cdt = cdt;
        }

        internal void InitializeCostCache() {
            foreach (var v in this.metroGraphData.VirtualNodes()) {
                v.cachedIdealRadius = HubRadiiCalculator.CalculateIdealHubRadiusWithNeighbors(this.metroGraphData, this.bundlingSettings, v);
                v.cachedRadiusCost = this.costCalculator.RadiusCost(v, v.Position);
                v.cachedBundleCost = 0;
            }

            foreach (var edge in this.metroGraphData.VirtualEdges()) {
                var v = edge.Item1;
                var u = edge.Item2;
                StationEdgeInfo edgeInfo = this.metroGraphData.GetIjInfo(v, u);
                edgeInfo.cachedBundleCost = this.costCalculator.BundleCost(v, u, v.Position);
                v.cachedBundleCost += edgeInfo.cachedBundleCost;
                u.cachedBundleCost += edgeInfo.cachedBundleCost;
            }
        }

        internal void UpdateCostCache(Station node) {
            RectangleNode<CdtTriangle,Point> cdtTree = this.cdt.GetCdtTree();
            node.CdtTriangle = cdtTree.FirstHitNode(node.Position, Test).UserData;

            node.cachedIdealRadius = HubRadiiCalculator.CalculateIdealHubRadiusWithNeighbors(this.metroGraphData, this.bundlingSettings, node);
            node.cachedRadiusCost = this.costCalculator.RadiusCost(node, node.Position);
            node.cachedBundleCost = 0;

            foreach (var adj in node.Neighbors) {
                if (!adj.IsRealNode) {
                    adj.cachedIdealRadius = HubRadiiCalculator.CalculateIdealHubRadiusWithNeighbors(this.metroGraphData, this.bundlingSettings, adj);
                    adj.cachedRadiusCost = this.costCalculator.RadiusCost(adj, adj.Position);
                }

                StationEdgeInfo edgeInfo = this.metroGraphData.GetIjInfo(node, adj);
                adj.cachedBundleCost -= edgeInfo.cachedBundleCost;

                edgeInfo.cachedBundleCost = this.costCalculator.BundleCost(node, adj, node.Position);
                node.cachedBundleCost += edgeInfo.cachedBundleCost;
                adj.cachedBundleCost += edgeInfo.cachedBundleCost;
            }
        }

        static internal HitTestBehavior Test(Point pnt, CdtTriangle t) {
            return Cdt.PointIsInsideOfTriangle(pnt, t) ? HitTestBehavior.Stop : HitTestBehavior.Continue;
        }

    }
}