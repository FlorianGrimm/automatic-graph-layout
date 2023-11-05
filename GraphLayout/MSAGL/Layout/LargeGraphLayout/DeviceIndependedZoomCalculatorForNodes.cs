using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Layout.LargeGraphLayout
{
    /// <summary>
    /// sets zoom levels for LgNodeInfos
    /// </summary>
    internal class DeviceIndependendZoomCalculatorForNodes : IZoomLevelCalculator
    {
        public Func<Node, LgNodeInfo> NodeToLgNodeInfo;
        private readonly int maxAmountPerTile;
        public GeometryGraph Graph { get; set; }
        public LgLayoutSettings Settings { get; set; }
        public List<LgNodeInfo> SortedLgNodeInfos { get { return this.sortedLgNodeInfos; } }

        private readonly List<int> _levelNodeCounts = new List<int>();

        public List<int> LevelNodeCounts { get { return this._levelNodeCounts; } }

        private int unassigned;
        private List<LgNodeInfo> sortedLgNodeInfos;
        private int zoomLevel;
        internal DeviceIndependendZoomCalculatorForNodes(
            Func<Node, LgNodeInfo> nodeToLgNodeInfo, GeometryGraph graph, LgLayoutSettings settings, int maxAmountPerTile)
        {
            this.NodeToLgNodeInfo = nodeToLgNodeInfo;
            this.maxAmountPerTile = maxAmountPerTile;
            this.Graph = graph;
            this.Settings = settings;
            this.unassigned = graph.Nodes.Count;
        }

        public void RunAfterFlow(LgData _lgData)
        {

            this.sortedLgNodeInfos = this.GetSortedLgNodeInfos2();
            this.Graph.UpdateBoundingBox();

            double gridSize = Math.Max(this.Graph.Width, this.Graph.Height);

            this.zoomLevel = 1;

            while (this.SomeNodesAreNotAssigned())
            {
                this.DrawNodesOnLevel2(gridSize, this.zoomLevel);
                this.zoomLevel *= 2;
                if (this.zoomLevel == 2) {
                    gridSize /= 2.5;
                }

                if (this.zoomLevel == 4) {
                    gridSize /= 2;
                }

                if (this.zoomLevel == 8) {
                    gridSize /= 1.5;
                }

                if (this.zoomLevel >= 8) {
                    gridSize /= 10;
                }

                if (this.zoomLevel >= 256) {
                    gridSize /= 10;
                }
            }
        }
        /// <summary>
        /// We expect that the node Ranks are set before the method call.
        /// </summary>
        public void Run()
        {
            this.sortedLgNodeInfos = this.GetSortedLgNodeInfos();
            this.Graph.UpdateBoundingBox();
            double gridSize = Math.Max(this.Graph.Width, this.Graph.Height);
            this.zoomLevel = 1;

            while (this.SomeNodesAreNotAssigned())
            {
                this.DrawNodesOnLevel(gridSize, this.zoomLevel);
                this.zoomLevel *= 2;
                if (this.zoomLevel == 2) {
                    gridSize /= 2.5;
                }

                if (this.zoomLevel == 4) {
                    gridSize /= 2;
                }

                if (this.zoomLevel == 8) {
                    gridSize /= 1.5;
                }

                if (this.zoomLevel >= 8) {
                    gridSize /= 3;//1.25;
                }

                if (this.zoomLevel >= 256) {
                    gridSize /= 10;
                }
            }
        }

        internal static Tuple<int, int> PointToTuple(Point graphLeftBottom, Point point, double gridSize)
        {
            var dx = point.X - graphLeftBottom.X;
            var dy = point.Y - graphLeftBottom.Y;

            return new Tuple<int, int>((int)(dx / gridSize), (int)(dy / gridSize));
        }

        private void DrawNodesOnLevel2(double gridSize, int currentLevel)
        {
            int[] nodeBoundPerLevel = new int[530];
            nodeBoundPerLevel[1] = 40;
            nodeBoundPerLevel[2] = 30;
            nodeBoundPerLevel[4] = 20;
            nodeBoundPerLevel[8] = 10;
            nodeBoundPerLevel[16] = 5;
            nodeBoundPerLevel[32] = 5;
            nodeBoundPerLevel[64] = 5;
            nodeBoundPerLevel[128] = 20;
            nodeBoundPerLevel[256] = 1000;


            var tileTable = new Dictionary<Tuple<int, int>, int>();
            for (int i = 0; i < this.sortedLgNodeInfos.Count; i++)
            {
                var ni = this.sortedLgNodeInfos[i];
                var tuple = PointToTuple(this.Graph.LeftBottom, ni.Center, gridSize);
                if (!tileTable.ContainsKey(tuple)) {
                    tileTable[tuple] = 0;
                }

                int countForTile = tileTable[tuple]++ + 1;
                //if (countForTile > nodeBoundPerLevel[currentLevel])
                if (countForTile > this.maxAmountPerTile)
                {
                    this._levelNodeCounts.Add(i);
                    break;
                }

                if (ni.ZoomLevel == this.zoomLevel)
                {
                    //ni.ZoomLevel = zoomLevel;
                    this.unassigned--;
                }

                if (this.unassigned == 0)
                {
                    this._levelNodeCounts.Add(i + 1);
                    break;
                }

                if (this.zoomLevel > 32)
                {
                    this.unassigned = 0;
                    break;
                }
            }

        }

        private void DrawNodesOnLevel(double gridSize, int currentLevel)
        {
            int[] nodeBoundPerLevel = new int[530];
            nodeBoundPerLevel[1] = 40;
            nodeBoundPerLevel[2] = 30;
            nodeBoundPerLevel[4] = 20;
            nodeBoundPerLevel[8] = 10;
            nodeBoundPerLevel[16] = 5;
            nodeBoundPerLevel[32] = 5;
            nodeBoundPerLevel[64] = 5;
            nodeBoundPerLevel[128] = 20;
            nodeBoundPerLevel[256] = 1000;


            var tileTable = new Dictionary<Tuple<int, int>, int>();
            for (int i = 0; i < this.sortedLgNodeInfos.Count; i++)
            {
                var ni = this.sortedLgNodeInfos[i];
                var tuple = PointToTuple(this.Graph.LeftBottom, ni.Center, gridSize);
                if (!tileTable.ContainsKey(tuple)) {
                    tileTable[tuple] = 0;
                }

                int countForTile = tileTable[tuple]++ + 1;
                //if (countForTile > nodeBoundPerLevel[currentLevel])
                if (countForTile > this.maxAmountPerTile)
                {
                    this._levelNodeCounts.Add(i);
                    break;
                }

                if (ni.ZoomLevelIsNotSet)
                {
                    ni.ZoomLevel = this.zoomLevel;
                    this.unassigned--;
                }

                if (this.unassigned == 0)
                {
                    this._levelNodeCounts.Add(i + 1);
                    break;
                }
            }

        }

        private bool SomeNodesAreNotAssigned()
        {
            return this.unassigned > 0;
        }

        static internal double GetDistBetweenBoundingBoxes(Node source, Node target)
        {
            var sb = source.BoundingBox;
            var tb = target.BoundingBox;
            if (sb.Intersects(tb)) {
                return 0;
            }

            var spolygon = PolygonFromBox(sb);
            var tpolygon = PolygonFromBox(tb);
            return Polygon.Distance(spolygon, tpolygon);
        }

        private static Polygon PolygonFromBox(Rectangle sb)
        {
            var spolygon =
                new Polygon(new Polyline(sb.LeftBottom, sb.LeftTop, sb.RightTop, sb.RightBottom) { Closed = true });
            return spolygon;
        }

        private List<LgNodeInfo> GetSortedLgNodeInfos()
        {
            var ret = this.Graph.Nodes.Select(n => this.NodeToLgNodeInfo(n)).ToList();
            ret.Sort((a, b) => b.Rank.CompareTo(a.Rank));
            return ret;
        }

        private List<LgNodeInfo> GetSortedLgNodeInfos2()
        {
            var ret = this.Graph.Nodes.Select(n => this.NodeToLgNodeInfo(n)).ToList();
            ret.Sort((a, b) => b.ZoomLevel.CompareTo(a.ZoomLevel));
            return ret;
        }

#if TEST_MSAGL


        //        void ShowZoomLevels() {
        //            var l = new List<DebugCurve>();
        //            foreach (LgNodeInfo node in Graph.Nodes.Select(n => NodeToLgNodeInfo(n))) {
        //                l.Add(new DebugCurve(200, 1, "blue", node.BoundaryCurve, String.Format("{0:###.#}", node.ZoomLevel)));
        //                l.Add(new DebugCurve(100, 1, "black", node.DominatedRect.Perimeter()));
        //                l.Add(new DebugCurve(new LineSegment(node.BoundaryCurve[0], node.DominatedRect.Center)));
        //            }
        //            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        //        }
#endif




    }
}