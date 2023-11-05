using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.OverlapRemovalFixedSegments;
using Microsoft.Msagl.Miscellaneous.RegularGrid;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.Visibility;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;

namespace Microsoft.Msagl.Layout.LargeGraphLayout.NodeRailLevelCalculator {
    public class GreedyNodeRailLevelCalculator {

        public List<LgNodeInfo> SortedLgNodeInfos { get; set; }

        public int ReroutingAttempts = 0;

        public Rectangle BoundingBox;
        public double MaxGridSize;

        public int NumCones = 8;

        public int MaxLevel = 1024*16;

        public int MaxAmountNodesPerTile = 15; //20; debug

        public int MaxAmountRailsPerTile = 20*8*2/4; //debug

        private readonly List<LgNodeInfo> _insertedNodes = new List<LgNodeInfo>();
        private readonly RTree<LgNodeInfo, Point> _insertedNodesTree = new RTree<LgNodeInfo, Point>();
        private readonly RTree<Rectangle, Point> _insertedNodeRectsTree = new RTree<Rectangle, Point>();
        private readonly RTree<SymmetricSegment, Point> _insertedSegmentsTree = new RTree<SymmetricSegment, Point>();
        private Dictionary<Tuple<int, int>, int> _nodeTileTable = new Dictionary<Tuple<int, int>, int>();
        private Dictionary<Tuple<int, int>, int> _segmentTileTable = new Dictionary<Tuple<int, int>, int>();
        private readonly Dictionary<int, List<LgNodeInfo>> _nodeLevels = new Dictionary<int, List<LgNodeInfo>>();
        
        public double ScaleBbox = 1.10;

        public bool UpdateBitmapForEveryInsertion = true;
        private readonly LgPathRouter _pathRouter = new LgPathRouter();

        public GreedyNodeRailLevelCalculator(List<LgNodeInfo> sortedLgNodeInfos) {
            this.SortedLgNodeInfos = sortedLgNodeInfos;

            foreach (var node in this.SortedLgNodeInfos) {
                node.Processed = false;
            }

            this.InitBoundingBox();
        }

        public void InitBoundingBox() {
            this.BoundingBox = new Rectangle(this.SortedLgNodeInfos.Select(ni => ni.Center));
            //Point center = BoundingBox.Center;
            //MaxGridSize = Math.Max(BoundingBox.Width, BoundingBox.Height)*ScaleBbox;
            //BoundingBox = new Rectangle(new Size(MaxGridSize, MaxGridSize), center);
        }


        public void PlaceNodesOnly(Rectangle bbox)
        {
            this.BoundingBox = bbox;
            int numInserted = 0;
            int level = 1;
            int iLevel = 0;

            while (numInserted < this.SortedLgNodeInfos.Count && level <= this.MaxLevel) {
                numInserted = this.DrawNodesOnlyOnLevel(level, numInserted);
                this.AddAllToNodeLevel(iLevel);
                level *= 2;
                iLevel++;
            }
        }

        private void MarkAllNodesNotProcessed() {
            foreach (var node in this.SortedLgNodeInfos) {
                node.Processed = false;
            }
        }

        internal int TryInsertingNodesAndRoutes(int numNodesToInsert,
            Dictionary<SymmetricTuple<LgNodeInfo>, List<Point>> trajectories,
            List<SymmetricSegment> oldSegments,  
            int zoomLevel, int numNodesOnPreviousLevel,
            GridTraversal grid, LgPathRouter pathRouter)
        {
            this.MarkAllNodesNotProcessed();
            this._segmentTileTable = new Dictionary<Tuple<int, int>, int>();
            this._nodeTileTable = new Dictionary<Tuple<int, int>, int>();

            var canAddOldSegments = this.TryAddingOldSegments(oldSegments, grid);
            if (!canAddOldSegments)
            {
                return 0;
            }

            this.AddOldNodes(numNodesOnPreviousLevel, grid);

            int i;
            for (i = numNodesOnPreviousLevel; i < numNodesToInsert; i++)
            {
                var ni = this.SortedLgNodeInfos[i];
                var nodeTile = grid.PointToTuple(ni.Center);
                if (!this._nodeTileTable.ContainsKey(nodeTile)) {
                    this._nodeTileTable[nodeTile] = 0;
                }

                if (this._nodeTileTable[nodeTile] >= this.MaxNodesPerTile(zoomLevel)) //test MaxAmountNodesPerTile
                {
                    this.ShowDebugInsertedSegments(grid, zoomLevel, ni, null, null);

                    break;
                }

                Set<VisibilityEdge> edges = this.GetSegmentsOnPathsToInsertedNeighborsNotOnOldTrajectories(ni, trajectories,
                    pathRouter);

                Set<SymmetricSegment> segments = new Set<SymmetricSegment>(
                    edges.Select(e => new SymmetricSegment(e.SourcePoint, e.TargetPoint)));

                var newToAdd = segments.Where(seg => !this.IsSegmentAlreadyAdded(seg)).ToList();

                Set<SymmetricSegment> insertedSegments;
                bool canInsertPaths = this.TryAddingSegmentsUpdateTiles(newToAdd, grid, out insertedSegments);

                if (canInsertPaths) {

                    this.AddSegmentsToRtree(newToAdd);
                    ni.Processed = true;
                    this._nodeTileTable[nodeTile]++;
                    this._insertedNodes.Add(ni);
                    continue;
                }
                //debug output
                //AddSegmentsToRtree(newToAdd);   //remove
            //    ShowDebugInsertedSegments(grid, zoomLevel, ni, newToAdd, segments);
                break;
            }

            var nextNode = numNodesToInsert < this.SortedLgNodeInfos.Count ? this.SortedLgNodeInfos[numNodesToInsert] :
            null;
           // ShowDebugInsertedSegments(grid, zoomLevel, nextNode, null, null);

            return i;
        }

        private Set<VisibilityEdge> GetSegmentsOnPathsToInsertedNeighborsNotOnOldTrajectories(LgNodeInfo ni, Dictionary<SymmetricTuple<LgNodeInfo>, List<Point>> trajectories, LgPathRouter pathRouter)
        {
            var edges = new Set<VisibilityEdge>();
            var neighbors = this.GetAdjacentProcessed(ni);
            foreach (var neighb in neighbors) {
                var t1 = new SymmetricTuple<LgNodeInfo>(ni, neighb);
                List<Point> trajectory;
                if (trajectories.ContainsKey(t1)) {
                    trajectory = trajectories[t1];
                } else {
                    continue;
                }

                for (int i = 0; i < trajectory.Count - 1; i++)
                {
                    var p0 = trajectory[i];
                    var p1 = trajectory[i + 1];

                    var e = pathRouter.FindEdge(p0, p1);

                    Debug.Assert(e!=null, "VisibilityEdge on trajectory not found!");

                    if (!pathRouter.IsOnOldTrajectory(e))
                    {
                        edges.Insert(e);
                    }
                }
            }
            return edges;
        }

        private bool TryAddingOldSegments(List<SymmetricSegment> oldSegments, GridTraversal grid) {
            Set<SymmetricSegment> insertedSegments;
            bool canInsertOldPaths = this.TryAddingSegmentsUpdateTiles(oldSegments, grid, out insertedSegments);

            if (canInsertOldPaths) {
                this.AddSegmentsToRtree(oldSegments);
                return true;
            }

            // if couldn't even insert previous level, terminate
            return false;            
        }

        private void AddOldNodes(int numNodesOnPreviousLevel, GridTraversal grid) {
            for (int i = 0; i < numNodesOnPreviousLevel; i++) {
                var ni = this.SortedLgNodeInfos[i];
                var nodeTile = grid.PointToTuple(ni.Center);
                if (!this._nodeTileTable.ContainsKey(nodeTile)) {
                    this._nodeTileTable[nodeTile] = 0;
                }

                ni.Processed = true;
                this._nodeTileTable[nodeTile]++;
                this._insertedNodes.Add(ni);
            }
        }

        
        private void ShowDebugInsertedSegments(GridTraversal grid, int zoomLevel, LgNodeInfo nodeToAdd, IEnumerable<SymmetricSegment> newToAdd, IEnumerable<SymmetricSegment> allOnNewEdges)
        {
#if TEST_MSAGL && !SHARPKIT && PREPARE_DEMO

            var edges = _pathRouter.GetAllEdgesVisibilityEdges();
            var ll = new List<DebugCurve>();

            foreach (var ni in _insertedNodes) {
                ll.Add(new DebugCurve(5, "green", ni.BoundaryCurve));
            }

            if (nodeToAdd != null) {
                var curve = _insertedNodes.Last().BoundaryCurve.Clone();
                curve.Translate(nodeToAdd.Center - _insertedNodes.Last().Center);
                ll.Add(new DebugCurve(5, "red", curve));
            }
            
            foreach (var e in edges)
            {
                ll.Add(new DebugCurve(new LineSegment(e.SourcePoint, e.TargetPoint)));
            }

            int n = zoomLevel;
            int maxNodes = MaxNodesPerTile(zoomLevel);

            for (int ix = 0; ix < n; ix++)
            {
                for (int iy = 0; iy < n; iy++)
                {
                    var tile = new Tuple<int, int>(ix, iy);
                    var r = grid.GetTileRect(ix, iy);
                    
                    if (_nodeTileTable.ContainsKey(tile)
                        && _nodeTileTable[tile] >= maxNodes)
                    {
                        ll.Add(new DebugCurve(5, "yellow", CurveFactory.CreateRectangle(r)));
                    }
                    else if (_segmentTileTable.ContainsKey(tile)
                             && _segmentTileTable[tile] >= MaxAmountRailsPerTile)
                    {
                        ll.Add(new DebugCurve(5, "orange", CurveFactory.CreateRectangle(r)));                    
                    }
                    else
                    {
                        ll.Add(new DebugCurve(5, "blue", CurveFactory.CreateRectangle(r)));
                    }
                }
            }

            if (allOnNewEdges != null) {
                foreach (var seg in allOnNewEdges) {
                    ll.Add(new DebugCurve(5, "yellow", new LineSegment(seg.A, seg.B)));
                }
            }

            if (newToAdd != null)
            {
                foreach (var seg in newToAdd)
                {
                    ll.Add(new DebugCurve(5, "red", new LineSegment(seg.A, seg.B)));
                }
            }

            LayoutAlgorithmSettings.ShowDebugCurves(ll.ToArray());

            PrintInsertedNodesLabels();
#endif
        }

        public string PrintInsertedNodesLabels()
        {
            var bbox = this.BoundingBox;
            var str = "<group>\n<path stroke=\"black\">";
            str += bbox.LeftTop.X + " " + bbox.LeftTop.Y + " m\n";
            str += bbox.LeftBottom.X + " " + bbox.LeftBottom.Y + " l\n";
            str += bbox.RightBottom.X + " " + bbox.RightBottom.Y + " l\n";
            str += bbox.RightTop.X + " " + bbox.RightTop.Y + " l\n";
            str += "h\n</path>\n";

            int i = 0;
            foreach (var ni in this._insertedNodes)
            {
                str += "<text transformations=\"translations\" pos=\"";
                str += ni.Center.X + " " + ni.Center.Y;
                str += "\" stroke=\"black\" type=\"label\" valign=\"baseline\">" +i +"</text>\n";
                i++;
            }
            str += "</group>";
            return str;
        }

        private bool TryAddingSegmentsUpdateTiles(IEnumerable<SymmetricSegment> segments, GridTraversal grid,
            out Set<SymmetricSegment> insertedSegments) {
            if (!this.IfCanInsertLooseSegmentsUpdateTiles(segments.ToList(), out insertedSegments, grid)) {
                // quota broken when inserting node boundary segments
                this.RemoveLooseSegmentsDecrementTiles(insertedSegments, grid);
                return false;
            }
            return true;
        }

        private void AddNodeToInserted(LgNodeInfo ni) {
            this._insertedNodes.Add(ni);

            var rect = new Rectangle(ni.Center, ni.Center);

            this._insertedNodesTree.Add(rect, ni);
            this._insertedNodeRectsTree.Add(rect, rect);
        }

        private void AddAllToNodeLevel(int iLevel) {
            if (!this._nodeLevels.ContainsKey(iLevel)) {
                this._nodeLevels[iLevel] = new List<LgNodeInfo>();
            }

            foreach (var ni in this._insertedNodes)
            {
                this._nodeLevels[iLevel].Add(ni);
            }
        }

        public List<int> GetLevelNodeCounts()
        {
            var nl = new List<int>(this._nodeLevels.Values.Select(l => l.Count));
            nl.Sort();
            return nl;
        }

        private int DrawNodesOnlyOnLevel(int level, int startInd)
        {
            int iLevel = (int) Math.Log(level, 2);
            GridTraversal grid= new GridTraversal(this.BoundingBox, iLevel);
            this.UpdateTilesCountInsertedNodesOnly(level, grid);
            for (int i = startInd; i < this.SortedLgNodeInfos.Count; i++) {
                var ni = this.SortedLgNodeInfos[i];
                var tuple = grid.PointToTuple(ni.Center);
                if (!this._nodeTileTable.ContainsKey(tuple)) {
                    this._nodeTileTable[tuple] = 0;
                }

                if (this._nodeTileTable[tuple] >= this.MaxNodesPerTile(level)) {
                    return i;
                }

                this.PerformNodeInsertion(ni, tuple);
                ni.ZoomLevel = level;
            }

            return this.SortedLgNodeInfos.Count;
        }

        private int MaxNodesPerTile(int zoomLevel) {
            //if (iLevel > 2) return MaxAmountNodesPerTile;
            //return 3*MaxAmountNodesPerTile/2;
            var iLevel = Math.Log(zoomLevel, 2);
            int maxNodes = this.MaxAmountNodesPerTile;
            //maxNodes += (int)(IncreaseNodeQuota * Math.Sqrt(iLevel) * MaxAmountNodesPerTile);
            maxNodes += (int) (this.IncreaseNodeQuota *Math.Pow(iLevel, 0.4)* this.MaxAmountNodesPerTile);
            return maxNodes;
        }

        private Set<LgNodeInfo> GetAdjacentProcessed(LgNodeInfo ni) {
            var nodes = new Set<LgNodeInfo>(from edge in ni.GeometryNode.Edges
                let s = this.GeometryNodesToLgNodeInfos[edge.Source]
                let t = this.GeometryNodesToLgNodeInfos[edge.Target]
                select ni == s ? t : s
                into v
                where v.Processed
                select v);
            return nodes;
        }

        private void PerformNodeInsertion(LgNodeInfo node, Tuple<int, int> tile) {
            this._nodeTileTable[tile]++;
            this.AddNodeToInserted(node);
            //orb.DrawDilated(node.BoundingBox);
        }


        internal bool IfCanInsertLooseSegmentUpdateTiles(SymmetricSegment seg,  GridTraversal grid) {
            //test if already inserted
            if (this.IsSegmentAlreadyAdded(seg)) {
                return true;
            }

            var intersectedTiles = this.GetIntersectedTiles(seg.A, seg.B, grid);

            int maxNumRailPerTile = 0;

            bool canInsertSegment = true;
            foreach (var tile in intersectedTiles) {
                if (!this._segmentTileTable.ContainsKey(tile)) {
                    this._segmentTileTable[tile] = 0;
                }

                if (maxNumRailPerTile< this._segmentTileTable[tile]) {
                    maxNumRailPerTile = this._segmentTileTable[tile];
                }

                canInsertSegment &= this._segmentTileTable[tile] < this.MaxAmountRailsPerTile;
            }

            if (!canInsertSegment)
            {
                return false;
            }

            foreach (var tile in intersectedTiles) {
                this._segmentTileTable[tile]++;
            }
            return true;
        }

        private bool IfCanInsertLooseSegmentsUpdateTiles(IEnumerable<SymmetricSegment> segs, out Set<SymmetricSegment> insertedSegs, GridTraversal grid) {
            insertedSegs = new Set<SymmetricSegment>();
            foreach (var seg in segs) {
                if (this.IfCanInsertLooseSegmentUpdateTiles(seg, grid)) {
                    insertedSegs.Insert(seg);
                } else {
                    return false;
                }
            }
            return true;
        }

        private void RemoveLooseSegmentsDecrementTiles(IEnumerable<SymmetricSegment> segs, GridTraversal grid) {
            foreach (var seg in segs) {
                this.RemoveLooseSegmentDecrementTiles(seg, grid);
            }
        }

        private void RemoveLooseSegmentDecrementTiles(SymmetricSegment seg, GridTraversal grid) {
            var intersectedTiles = this.GetIntersectedTiles(seg.A, seg.B,  grid);

            foreach (var tile in intersectedTiles) {
                if (this._segmentTileTable.ContainsKey(tile)) {
                    this._segmentTileTable[tile]--;
                }
            }
        }

        private void AddSegmentToRtree(SymmetricSegment seg) {
            if (this.IsSegmentAlreadyAdded(seg)) {
                return;
            }

            var rect = new Rectangle(seg.A, seg.B);
            //rect = new Rectangle(rect.LeftBottom - segRtreeBuffer*new Point(1, 1),
            //    rect.RightTop + segRtreeBuffer*new Point(1, 1));

            this._insertedSegmentsTree.Add(rect, seg);

            // add to visgraph
            this._pathRouter.AddVisGraphEdge(seg.A, seg.B);
        }

        private bool IsSegmentAlreadyAdded(SymmetricSegment seg) {
            return this._pathRouter.ExistsEdge(seg.A, seg.B);
        }

        private void AddSegmentsToRtree(IEnumerable<SymmetricSegment> segs) {
            foreach (var seg in segs) {
                this.AddSegmentToRtree(seg);
            }
        }

        private void UpdateTilesCountInsertedNodes(int level, GridTraversal grid) {
            foreach (var node in this._insertedNodes) {
                var tuple = grid.PointToTuple(node.Center);
                if (!this._nodeTileTable.ContainsKey(tuple)) {
                    this._nodeTileTable[tuple] = 0;
                }

                this._nodeTileTable[tuple]++;
            }
        }

        private void UpdateTilesCountInsertedNodesOnly(int level, GridTraversal grid) {
            this._nodeTileTable = new Dictionary<Tuple<int, int>, int>();
            this.UpdateTilesCountInsertedNodes(level, grid);
        }

        private List<Tuple<int, int>> GetIntersectedTiles(Point p1, Point p2,  GridTraversal grid) {
            return grid.GetTilesIntersectedByLineSeg(p1, p2);
        }

        public Dictionary<Node, LgNodeInfo> GeometryNodesToLgNodeInfos { get; set; }
        public double IncreaseNodeQuota { get; set; }
    }
}
