using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using System;
using System.Collections.Generic;
using System.Linq;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
namespace Microsoft.Msagl.Layout.OverlapRemovalFixedSegments
{

    public enum SiteType { CenterBoxMoveable, CenterBoxFixed, PointOnSegment }

    public class OverlapRemovalFixedSegmentsMst
    {
        public int time = 0;
        private Rectangle[] moveableRectangles;
        private Rectangle[] fixedRectangles;
        private SymmetricSegment[] fixedSegments;
        private Point[] oldPositionsMoveable;
        private String[] nodeLabelsMoveable;
        private String[] nodeLabelsFixed;
        private Cdt cdt;
        private Dictionary<Point, TreeNode> pointToTreeNode = new Dictionary<Point, TreeNode>();
        private RTree<Segment, Point> _segmentTree = new RTree<Segment, Point>();
        private RTree<TreeNode, Point> _moveableRectanglesTree = new RTree<TreeNode, Point>();
        private RTree<TreeNode, Point> _fixedRectanglesTree = new RTree<TreeNode, Point>();
        private RTree<TreeNode, Point> _rectNodesRtree = new RTree<TreeNode, Point>();
        private Dictionary<TreeNode, List<TreeNode>> movedCriticalNodes = new Dictionary<TreeNode, List<TreeNode>>();
        private Dictionary<TreeNode, List<TreeNode>> oldOverlapsBoxes = new Dictionary<TreeNode, List<TreeNode>>();
        private Dictionary<TreeNode, List<Segment>> oldOverlapsSegments = new Dictionary<TreeNode, List<Segment>>();
        private List<List<TreeNode>> _subtrees = new List<List<TreeNode>>();
        private List<TreeNode> _roots = new List<TreeNode>();
        private List<SymmetricSegment> treeSegments;
        private const int Precision = 5;
        private const double PrecisionDelta = 1e-5;
        private double EdgeContractionFactor = 0.1;
        private double EdgeExpansionFactor = 1.1;
        private double BoxSegmentOverlapShiftFactor = 1.1;

        public class Segment
        {
            public int segmentId;
            private Point point1;
            private Point point2;

            public Segment(Point point1, Point point2)
            {
                this.point1 = point1;
                this.point2 = point2;
            }
            public Point p1 { get { return this.point1; } }
            public Point p2 { get { return this.point2; } }
        }

        public enum SiteType
        {
            RectFixed,
            RectMoveable,
            AdditionalPointBoxSegmentOverlap
        };

        public class TreeNode
        {
            public static int numNodes = 0;

            public Point shiftToRoot = new Point(0, 0);
            public Set<TreeNode> neighbors = new Set<TreeNode>();
            public int visited = 0;
            public TreeNode parent = null;
            public bool isFixed = false;
            public Rectangle rect;
            public int id;
            public int rectId;
            public string label;

            public Segment segment;

            public Point sitePoint;

            public SiteType type;

            public TreeNode(){
                this.id = numNodes++;
            }

            public override string ToString()
            {
                return (this.label != null ? this.label.ToString() : "null"); // +", " + sitePoint.ToString() + ", " + rect.ToString();
            }
        }

        private Point Round(Point p) {
            return ApproximateComparer.Round(p, Precision);
        }

        public void SetNodeLabels(String[] labelsMovealbe, String[] labelsFixed) {
            this.nodeLabelsMoveable = labelsMovealbe;
            this.nodeLabelsFixed = labelsFixed;
        }

        public void SaveOldPositionsMoveable() {
            this.oldPositionsMoveable = new Point[this.moveableRectangles.Length];

            for(int i=0; i< this.moveableRectangles.Length; i++)
            {
                this.oldPositionsMoveable[i] = this.moveableRectangles[i].Center;
            }
        }

        public Point[] GetTranslations() {            
            var nodes = this._moveableRectanglesTree.GetAllLeaves();
            Point[] translation = new Point[this._moveableRectanglesTree.Count];

            foreach (var n in nodes)
            {
                translation[n.rectId] = n.rect.Center - this.oldPositionsMoveable[n.rectId];
            }
            return translation;
        }

        public OverlapRemovalFixedSegmentsMst(Rectangle[] moveableRectangles, Rectangle[] fixedRectangles, SymmetricSegment[] fixedSegments)
        {
            TreeNode.numNodes = 0;
            this.moveableRectangles = moveableRectangles;
            this.fixedRectangles = fixedRectangles;
            this.fixedSegments = fixedSegments;

            this.SaveOldPositionsMoveable();
        }

        public void Init()
        {
            this.InitCdt();
            this.InitTree();
        }

        public void InitCdt() {

            this.InitSegmentTree();

            for (int i = 0; i < this.fixedRectangles.Length; i++) {
                Point p = this.Round(this.fixedRectangles[i].Center );
                var node = new TreeNode { isFixed = true, rectId = i, rect = this.fixedRectangles[i], sitePoint = p, type = SiteType.RectFixed };
                if (this.nodeLabelsFixed != null)
                {
                    node.label = this.nodeLabelsFixed[i];
                }
                this.pointToTreeNode[p] = node;
                this._rectNodesRtree.Add(node.rect, node);
                this._fixedRectanglesTree.Add(this.fixedRectangles[i], node);
            }

            for (int i = 0; i < this.moveableRectangles.Length; i++)
            {
                Point p = this.Round(this.moveableRectangles[i].Center);
                var node = new TreeNode { isFixed = false, rectId = i, rect = this.moveableRectangles[i], sitePoint = p, type = SiteType.RectMoveable };
                if (this.nodeLabelsMoveable != null)
                {
                    node.label = this.nodeLabelsMoveable[i];
                }

                this.pointToTreeNode[p] = node;
                this._rectNodesRtree.Add(node.rect, node);
                this._moveableRectanglesTree.Add(this.moveableRectangles[i], node);
            }

            var sites = this.pointToTreeNode.Keys.ToList();

            //AddSitesForBoxSegmentOverlaps(sites);            

            this.cdt = new Cdt(sites, null, null);
            this.cdt.Run();
        }

        private void AddNodesForBoxSegmentOverlaps(out List<Point> sites, out List<SymmetricSegment> edges)
        {
            sites = new List<Point>();
            edges = new List<SymmetricSegment>();

            foreach (var rect in this.moveableRectangles)
            {
                List<Segment> segments = this.GetAllSegmentsIntersecting(rect);
                if (!segments.Any()) {
                    continue;
                }

                var seg = segments.First();
                //foreach (var seg in segments)
                //{
                    Point p;
                    Point pClosestOnSeg = RectSegIntersection.ClosestPointOnSegment(seg.p1, seg.p2, rect.Center);
                    
                    // if too close
                    if ((pClosestOnSeg - rect.Center).Length < 10*PrecisionDelta)
                    {
                        Point d = (seg.p2 - seg.p2).Rotate90Ccw();
                        Point delta = 0.5* this.GetShiftUntilNoLongerOverlapRectSeg(rect, seg, d);
                        p = this.Round(rect.Center + delta);
                    }
                    else
                    {
                        p = this.Round(0.5 * (rect.Center + pClosestOnSeg));
                    }
                                        
                    TreeNode node = new TreeNode { isFixed = false, sitePoint = p, type = SiteType.AdditionalPointBoxSegmentOverlap, segment = seg, rect = rect, label = "SegOvlp "};
                    if (!this.pointToTreeNode.ContainsKey(p)) {
                        sites.Add(p);
                    this.pointToTreeNode[p] = node;
                        edges.Add(new SymmetricSegment(p, this.Round(rect.Center)) );
                    }

                    TreeNode box = this.pointToTreeNode[this.Round(rect.Center)];
                    if(box.type == SiteType.RectMoveable) {
                    node.label += box.label;
                }
                //}
            }
        }

        private List<Segment> GetAllSegmentsIntersecting(Rectangle rect)
        {
            var segmentsIntersecting = new List<Segment>();
            var boxIntersecting = this._segmentTree.GetAllIntersecting(rect);
            foreach(var seg in boxIntersecting)
            {
                if(RectSegIntersection.Intersect(rect, seg.p1, seg.p2)) {
                    segmentsIntersecting.Add(seg);
                }
            }
            return segmentsIntersecting;
        }

        public void InitSegmentTree() {
            for (int i = 0; i < this.fixedSegments.Length; i++) {
                Segment seg = new Segment(this.fixedSegments[i].A, this.fixedSegments[i].B);
                var bbox = new Rectangle(seg.p1, seg.p2);
                this._segmentTree.Add(bbox, seg);
            }
        }

        public List<SymmetricSegment> GetMstFromCdt() {
            Func<CdtEdge, double> weights = this.GetWeightOfCdtEdgeDefault;//GetWeightOfCdtEdgeDefault;// GetWeightOfCdtEdgePenalizeFixed; //GetWeightOfCdtEdgePenalizeSegmentCrossings ;
            var mstEdges = MstOnDelaunayTriangulation.GetMstOnCdt(this.cdt, weights);
            return (from e in mstEdges select new SymmetricSegment(e.upperSite.Point, e.lowerSite.Point)).ToList();
        }

        public void BuildForestFromCdtEdges(List<SymmetricSegment> edges) {
            foreach (var edge in edges) {
                Point p1 = edge.A;
                Point p2 = edge.B;
                this.AddTreeEdge(p1, p2);
            }
        }

        private void AddTreeEdge(Point p1, Point p2)
        {
            var n1 = this.pointToTreeNode[p1];
            var n2 = this.pointToTreeNode[p2];
            n1.neighbors.Insert(n2);
            n2.neighbors.Insert(n1);
        }

        private bool RectsOverlap(TreeNode n1, TreeNode n2) {
            return !Rectangle.Intersect(n1.rect, n2.rect).IsEmpty;
        }

        public double GetWeightOfCdtEdgeDefault(CdtEdge e) {
            Point point1 = this.Round(e.upperSite.Point);
            Point point2 = this.Round(e.lowerSite.Point);
            TreeNode n1 = this.pointToTreeNode[point1];
            TreeNode n2 = this.pointToTreeNode[point2];

            if (n1.type == SiteType.AdditionalPointBoxSegmentOverlap ||
                n2.type == SiteType.AdditionalPointBoxSegmentOverlap) {
                return -Math.Max(n1.rect.Diagonal, n2.rect.Diagonal); // todo: better values? should be very small
            }

            Rectangle box1 = n1.rect;
            Rectangle box2 = n2.rect;

            double t;

            if (!Rectangle.Intersect(box1, box2).IsEmpty) {
                return this.GetWeightOverlappingRectangles(box1, box2, out t);
            }

            return this.GetDistance(box1, box2);
        }

        public double GetWeightOfCdtEdgePenalizeFixed(CdtEdge e)
        {
            Point point1 = this.Round(e.upperSite.Point);
            Point point2 = this.Round(e.lowerSite.Point);
            TreeNode n1 = this.pointToTreeNode[point1];
            TreeNode n2 = this.pointToTreeNode[point2];

            Rectangle box1 = n1.rect;
            Rectangle box2 = n2.rect;

            bool overlap = Rectangle.Intersect(box1, box2).IsEmpty;
            double t;

            if (!Rectangle.Intersect(box1, box2).IsEmpty) {
                return this.GetWeightOverlappingRectangles(box1, box2, out t);
            }

            double factor = (n1.isFixed || n2.isFixed ? 10 : 1);

            return factor * this.GetDistance(box1, box2);
        }

        public double GetWeightOfCdtEdgePenalizeSegmentCrossings(CdtEdge e)
        {
            Point point1 = this.Round(e.upperSite.Point);
            Point point2 = this.Round(e.lowerSite.Point);
            TreeNode n1 = this.pointToTreeNode[point1];
            TreeNode n2 = this.pointToTreeNode[point2];

            Rectangle box1 = n1.rect;
            Rectangle box2 = n2.rect;

            bool overlap = Rectangle.Intersect(box1, box2).IsEmpty;
            double t;

            if (!Rectangle.Intersect(box1, box2).IsEmpty) {
                return this.GetWeightOverlappingRectangles(box1, box2, out t);
            }

            double factor = (this.IsCrossedBySegment(new SymmetricSegment(point1, point2)) ? 3 : 1 );

            return factor * this.GetDistance(box1, box2);
        }

        public double GetWeightOverlappingRectangles(Rectangle box1, Rectangle box2, out double t) {

            Point point1 = box1.Center;
            Point point2 = box2.Center;

            double dist = (point1 - point2).Length;
            double dx = Math.Abs(point1.X - point2.X);
            double dy = Math.Abs(point1.Y - point2.Y);

            double wx = (box1.Width / 2 + box2.Width / 2);
            double wy = (box1.Height / 2 + box2.Height / 2);

            const double machineAcc = 1.0e-16;

            //double t;
            if (dx < machineAcc * wx)
            {
                t = wy / dy;
            }
            else if (dy < machineAcc * wy)
            {
                t = wx / dx;
            }
            else
            {
                t = Math.Min(wx / dx, wy / dy);
            }

            //if (t > 1) t = Math.Max(t, 1.001); // must be done, otherwise the convergence is very slow
            ////            tmax = Math.Max(tmax, t);
            ////            tmin = Math.Min(tmin, t);
            //t = Math.Min(expandMax, t);
            //t = Math.Max(expandMin, t);
            //tRes = t;
            //return t * dist;
            return - (t-1)*dist;
        }

        private double getT(Rectangle box1, Rectangle box2) {
            Point point1 = box1.Center;
            Point point2 = box2.Center;

            double dist = (point1 - point2).Length;
            double dx = Math.Abs(point1.X - point2.X);
            double dy = Math.Abs(point1.Y - point2.Y);

            double wx = (box1.Width / 2 + box2.Width / 2);
            double wy = (box1.Height / 2 + box2.Height / 2);

            const double machineAcc = 1.0e-16;

            double t;
            if (dx < machineAcc * wx)
            {
                t = wy / dy;
            }
            else if (dy < machineAcc * wy)
            {
                t = wx / dx;
            }
            else
            {
                t = Math.Min(wx / dx, wy / dy);
            }
            return t;
        }

        /// <summary>
        /// vector to shift box2 along the line between the centers until overlap
        /// </summary>
        /// <param name="box1"></param>
        /// <param name="box2"></param>
        /// <returns></returns>
        public Point GetShiftUntilOverlap(Rectangle box1, Rectangle box2) {
            Point point1 = box1.Center;
            Point point2 = box2.Center;

            double t = this.getT(box1, box2);

            return (1 - t) * (point1 - point2);
        }

        /// <summary>
        /// vector to shift box2 along the line between the centers until no longer overlap
        /// </summary>
        /// <param name="box1"></param>
        /// <param name="box2"></param>
        /// <returns></returns>
        public Point GetShiftUntilNoLongerOverlap(Rectangle box1, Rectangle box2)
        {
            Point point1 = box1.Center;
            Point point2 = box2.Center;

            double t = this.getT(box1, box2);

            return (t - 1) * (point2 - point1);
        }

        public double GetDistance(Rectangle rect1, Rectangle rect2) {
            if (!Rectangle.Intersect(rect1, rect2).IsEmpty) {
                return 0;
            }

            Rectangle leftmost = rect1.Left <= rect2.Left ? rect1 : rect2;
            Rectangle notLeftmost = rect1.Left <= rect2.Left ? rect2 : rect1;

            Rectangle botommost = rect1.Bottom <= rect2.Bottom ? rect1 : rect2;
            Rectangle notBotommost = rect1.Bottom <= rect2.Bottom ? rect2 : rect1;

            double dx = notLeftmost.Left - leftmost.Right;
            double dy = notBotommost.Bottom - botommost.Top;

            if(rect1.IntersectsOnX(rect2)) {
                return dy;
            }

            if (rect1.IntersectsOnY(rect2)) {
                return dx;
            }

            return Math.Sqrt(dx * dx + dy * dy);
        }

        public Rectangle GetInitialBoundingBox() {
            Rectangle bbox = new Rectangle();
            bbox.SetToEmpty();
            foreach (var rect in this.fixedRectangles) {
                bbox.Add(rect);
            }
            foreach (var rect in this.moveableRectangles)
            {
                bbox.Add(rect);
            }
            return bbox;
        }

        public Rectangle GetBoundingBox(IEnumerable<TreeNode> nodes)
        {
            Rectangle bbox = new Rectangle();
            bbox.SetToEmpty();
            foreach (var node in nodes)
            {
                bbox.Add(node.rect);
            }
            return bbox;
        }

        public Point GetClosestSiteTo(Point p) {
            double dist = Double.MaxValue;
            Point closest = this.pointToTreeNode.Keys.First();
            foreach (var s in this.pointToTreeNode.Keys)
            {
                double dist1 = (p - s).LengthSquared;
                if (dist1 < dist) {
                    dist = dist1;
                    closest = s;
                }
            }
            return closest;
        }

        public Point GetClosestSiteTo(List<TreeNode> nodes, Point p)
        {
            double dist = Double.MaxValue;
            Point closest = this.pointToTreeNode.Keys.First();
            foreach (var node in nodes)
            {
                Point s = node.sitePoint;
                double dist1 = (p - s).LengthSquared;
                if (dist1 < dist)
                {
                    dist = dist1;
                    closest = s;
                }
            }
            return closest;
        }

        public Point GetClosestSiteToPreferFixed(List<TreeNode> nodes, Point p)
        {
            double dist = Double.MaxValue;
            Point closest = this.pointToTreeNode.Keys.First();
            foreach (var node in nodes)
            {
                Point s = node.sitePoint;
                double dist1 = (p - s).LengthSquared;

                if (node.isFixed) {
                    dist1 /= 4;
                }

                if (dist1 < dist)
                {
                    dist = dist1;
                    closest = s;
                }
            }
            return closest;
        }

        public TreeNode GetRandom(List<TreeNode> nodes)
        {
            Random rand = new Random(1);
            int i = rand.Next(0, nodes.Count);
            return nodes[i];
        }

        private void OrientTreeEdges(TreeNode root) {
            List<TreeNode> L = new List<TreeNode>();
            this.time++;
            root.visited = this.time;
            root.parent = null;
            L.Add(root);

            while (L.Any())
            {
                TreeNode p = L[L.Count() - 1];
                L.RemoveAt(L.Count() - 1);
                foreach (var neighb in p.neighbors) {
                    if (neighb.visited < this.time) {
                        neighb.parent = p;
                        neighb.visited = this.time;
                        L.Add(neighb);
                    }
                }
            }
        }

        public void PrecomputeMovedCriticalNodes(TreeNode n)
        {
            if (this.movedCriticalNodes.ContainsKey(n)) {
                return; // never happens
            }

            this.movedCriticalNodes[n] = new List<TreeNode>();
                            
            foreach(var neighb in n.neighbors){
                if (neighb != n.parent)
                {
                    this.PrecomputeMovedCriticalNodes(neighb);
                    if (!n.isFixed)
                    {
                        this.movedCriticalNodes[n].AddRange(this.movedCriticalNodes[neighb]);
                    }
                }
            }
            //int numOverlaps = _nodesRtree.GetAllIntersecting(n.rect).Count();
            //if(!n.isFixed && numOverlaps > 1)
            if (!n.isFixed && (this.oldOverlapsBoxes.ContainsKey(n) ))
            {
                this.movedCriticalNodes[n].Add(n);
            }
        }

        public void PrecomputeMovedCriticalNodesBoxesSegments(TreeNode n)
        {
            if (this.movedCriticalNodes.ContainsKey(n)) {
                return; // never happens
            }

            this.movedCriticalNodes[n] = new List<TreeNode>();

            foreach (var neighb in n.neighbors)
            {
                if (neighb != n.parent)
                {
                    this.PrecomputeMovedCriticalNodesBoxesSegments(neighb);
                    if (!n.isFixed)
                    {
                        this.movedCriticalNodes[n].AddRange(this.movedCriticalNodes[neighb]);
                    }
                }
            }
            if (n.type == SiteType.RectMoveable && (this.oldOverlapsBoxes.ContainsKey(n) || this.oldOverlapsSegments.ContainsKey(n)))
            {
                this.movedCriticalNodes[n].Add(n);
            }
        }

        public void MoveSubtree(TreeNode n, Point delta) {
            if (n.isFixed) {
                return;
            }

            foreach (var neighb in n.neighbors)
            {
                if (neighb != n.parent)
                {
                    this.MoveSubtree(neighb, delta);
                }
            }
            n.rect = this.translate(n.rect, delta);            
        }

        public void PrecomputeMovedCriticalNodesRoot(TreeNode root)
        {
            foreach (var n in root.neighbors)
            {
                this.PrecomputeMovedCriticalNodes(n);
            }
        }

        public void PrecomputeMovedCriticalNodesBoxesSegmentsRoot(TreeNode root)
        {
            foreach (var n in root.neighbors)
            {
                this.PrecomputeMovedCriticalNodesBoxesSegments(n);
            }
        }

        public void PrecomputeMovedCriticalNodesBoxesSegmentsAllRoots()
        {
            foreach (var root in this._roots)
            {
                this.PrecomputeMovedCriticalNodesBoxesSegmentsRoot(root);
            }
        }

        public void PrecomputeMovedCriticalNodesAllRoots()
        {
            foreach (var root in this._roots)
            {
                this.PrecomputeMovedCriticalNodesRoot(root);
            }
        }

        public void GetDfsOrder(TreeNode node, List<TreeNode> nodes) {
            foreach (var neighb in node.neighbors) {
                if (neighb != node.parent) {
                    nodes.Add(neighb);
                    this.GetDfsOrder(neighb, nodes);
                }
            }
        }

        private void SaveCurrentOverlapsBoxes() {
            // assume all Rtrees are updated

            this.oldOverlapsBoxes = new Dictionary<TreeNode, List<TreeNode>>();
            foreach (var v in this._moveableRectanglesTree.GetAllLeaves())
            {
                var vOverlaps = this._rectNodesRtree.GetAllIntersecting(v.rect);
                if (vOverlaps.Count() <= 1) {
                    continue;
                }

                var vOverlapsList = new List<TreeNode>();
                foreach (var u in vOverlaps)
                {
                    if (u != v) {
                        vOverlapsList.Add(u);
                    }
                }
                if (vOverlapsList.Any()) {
                    this.oldOverlapsBoxes.Add(v, vOverlapsList);
                }
            }        
        }

        private void SaveCurrentOverlapsSegments() {
            this.oldOverlapsSegments = new Dictionary<TreeNode, List<Segment>>();
            foreach (var v in this._moveableRectanglesTree.GetAllLeaves())
            {
                var vOverlaps = this._segmentTree.GetAllIntersecting(v.rect);
                if (!vOverlaps.Any()) {
                    continue;
                }

                var vOverlapsList = new List<Segment>();
                foreach (var s in vOverlaps)
                {
                    if(RectSegIntersection.Intersect(v.rect, s.p1, s.p2)) {
                        vOverlapsList.Add(s);
                    }
                }
                if (vOverlapsList.Any()) {
                    this.oldOverlapsSegments.Add(v, vOverlapsList);
                }
            }  
        }

        public void SaveCurrentOverlapsBoxesSegments() {
            this.SaveCurrentOverlapsBoxes();
            this.SaveCurrentOverlapsSegments();
        }

        public void DoOneIterationBoxesSegmentsAllRoots()
        {
            this.SaveCurrentOverlapsBoxesSegments();
            this.PrecomputeMovedCriticalNodesBoxesSegmentsAllRoots();

            foreach (var root in this._roots)
            {
                this.DoOneIterationBoxesSegments(root);
            }
        }

        public void DoOneIterationBoxesSegments(TreeNode root)
        {
            //SaveCurrentOverlapsBoxesSegments(); // do once 

            List<TreeNode> dfsOrderedNodes = new List<TreeNode>();
            this.GetDfsOrder(root, dfsOrderedNodes); //doesn't contain root

            foreach (var v in dfsOrderedNodes)
            {
                var vMovesCritical = this.movedCriticalNodes[v];
                if (!vMovesCritical.Any()) {
                    continue;
                }

                Point delta;

                if (v.type == SiteType.AdditionalPointBoxSegmentOverlap)
                {
                    this.HandleCaseAdditionalPointOvlp(v, out delta);
                }
                else// if (v.type == SiteType.RectMoveable)
                {
                    this.HandleCaseRectMoveable(v, out delta);
                }

                double a1 = this.GetAreaOldOverlapsBoxesSegments(vMovesCritical);

                // shift and compare
                this.translateAllRects(vMovesCritical, delta);
                double a2 = this.GetAreaOldOverlapsBoxesSegments(vMovesCritical);
                this.translateAllRects(vMovesCritical, -delta);

                ////test: small chance of extanding edge instead of contracting
                //if (contractingEdge && a1 <= a2)
                //{
                //    Random rnd = new Random(1);
                //    int r = rnd.Next(0, 100);
                //    if (r < 50)
                //    {
                //        delta = -delta;
                //        translateAllRects(vMovesCritical, delta);
                //        a2 = GetAreaOldOverlapsBoxesSegments(vMovesCritical);
                //        translateAllRects(vMovesCritical, -delta);
                //    }
                //}
                /////////////////////////////////

                if (a1 <= a2) {
                    continue;
                }

                this.MoveSubtree(v, delta);
            }
        }

        private void HandleCaseRectMoveable(TreeNode v, out Point delta)
        {
            if (v.parent.type == SiteType.RectFixed || v.parent.type == SiteType.RectMoveable)
            {
                if (this.RectsOverlap(v, v.parent))
                {
                    delta = this.EdgeExpansionFactor * this.GetShiftUntilNoLongerOverlap(v.parent.rect, v.rect);
                }
                else
                {
                    delta = this.EdgeContractionFactor * this.GetShiftUntilOverlap(v.parent.rect, v.rect);
                }
            }
            else //if (v.parent.type == SiteType.AdditionalPointBoxSegmentOverlap)
            {
                Point d = v.sitePoint - v.parent.sitePoint;
                delta = this.BoxSegmentOverlapShiftFactor * this.GetShiftUntilNoLongerOverlapRectSeg(v.rect, v.parent.segment, d);
            }
        }

        private Point GetShiftUntilNoLongerOverlapRectSeg(Rectangle rect, Segment segment, Point moveDir)
        {
            return RectSegIntersection.GetOrthShiftUntilNoLongerOverlapRectSeg(rect, segment.p1, segment.p2, moveDir);
        }

        private void HandleCaseAdditionalPointOvlp(TreeNode v, out Point delta)
        {
            // v should be assigned the corresponding rectangle
            if (this.RectsOverlap(v, v.parent))
            {
                delta = this.EdgeExpansionFactor * this.GetShiftUntilNoLongerOverlap(v.parent.rect, v.rect);
            }
            else
            {
                delta = this.EdgeContractionFactor * this.GetShiftUntilOverlap(v.parent.rect, v.rect);
            }
        }

        private double GetAreaOldOverlapsBoxesSegments(List<TreeNode> vMovesCritical)
        {
            double a = 0;
            foreach (var u in vMovesCritical)
            {
                if (this.oldOverlapsBoxes.ContainsKey(u))
                {
                    var uOverlaps = this.oldOverlapsBoxes[u];
                    foreach (var w in uOverlaps)
                    {
                        var rect = Rectangle.Intersect(u.rect, w.rect);
                        if (!rect.IsEmpty) {
                            a += rect.Area;
                        }
                    }
                }
                if (this.oldOverlapsSegments.ContainsKey(u)) {
                    var uOverlaps = this.oldOverlapsSegments[u];
                    foreach (var seg in uOverlaps)
                    {
                        a += RectSegIntersection.GetOverlapAmount(u.rect, seg.p1, seg.p2);
                    }
                }
            }
            return a;
        }

        private Rectangle translate(Rectangle rect, Point delta) {
            return new Rectangle(rect.LeftBottom + delta, rect.RightTop + delta);
        }

        private void translateAllRects(List<TreeNode> nodes, Point delta)
        {
            foreach (var v in nodes) {
                v.rect = this.translate(v.rect, delta);
            }
        }


        public List<SymmetricSegment> GetEdgesWithoutSegmentCrossings(List<SymmetricSegment> edges)
        {
            List<SymmetricSegment> edgesLeft = new List<SymmetricSegment>();
            foreach (var edge in edges)
            {
                if (!this.IsCrossedBySegment(edge)) {
                    edgesLeft.Add(edge);
                }
            }
            return edgesLeft;
        }

        public void InitConnectedComponents(List<SymmetricSegment> edges)
        {
            var treeNodes = new TreeNode[this.pointToTreeNode.Count];
            foreach (var node in this.pointToTreeNode.Values)
            {
                treeNodes[node.id] = node;
            }

            var intEdges = new List<SimpleIntEdge>();
            foreach (var edge in edges)
            {
                int sourceId = this.pointToTreeNode[edge.A].id;
                int targetId = this.pointToTreeNode[edge.B].id;
                intEdges.Add(new SimpleIntEdge { Source = sourceId, Target = targetId });                              
            }
            var components = ConnectedComponentCalculator<SimpleIntEdge>.GetComponents(new BasicGraphOnEdges<SimpleIntEdge>(intEdges, this.pointToTreeNode.Count));

            foreach (var component in components)
            {
                List<TreeNode> nodeList = new List<TreeNode>();
                foreach (var nodeId in component)
                {
                    nodeList.Add(treeNodes[nodeId]);
                }
                this._subtrees.Add(nodeList);
            }

            //ChooseRoots();
            this.ChooseRootsRandom();

            this.BuildForestFromCdtEdges(edges);

            foreach (var root in this._roots)
            {
                this.OrientTreeEdges(root);
            }
        }

        public void ChooseRoots()
        {
            foreach (var subtree in this._subtrees)
            {
                var bbox = this.GetBoundingBox(subtree);
                var p = this.GetClosestSiteToPreferFixed(subtree, bbox.Center);
                this._roots.Add(this.pointToTreeNode[p]);
            }
        }

        public void ChooseRootsRandom()
        {
            foreach (var subtree in this._subtrees)
            {
                bool rootChosen = false;
                foreach (var n in subtree)
                {
                    if (n.type == SiteType.AdditionalPointBoxSegmentOverlap)
                    {
                        this._roots.Add(n);
                        rootChosen = true;
                        break;
                    }
                }
                if (!rootChosen)
                {
                    var n = this.GetRandom(subtree);
                    this._roots.Add(n);
                }
            }
        }

        private bool IsCrossedBySegment(SymmetricSegment edge)
        {
            var intersectingSegments = this._segmentTree.GetAllIntersecting(new Rectangle(edge.A, edge.B));
            bool hasIntersection = false;
            foreach (var seg in intersectingSegments)
            {
                if (RectSegIntersection.SegmentsIntersect(edge.A, edge.B, seg.p1, seg.p2))
                {
                    hasIntersection = true;
                    break;
                }
            }
            return hasIntersection;
        }

        public void InitTree()
        {
            var mstEdges = this.GetMstFromCdt();

            List<SymmetricSegment> treeEdges = this.GetEdgesWithoutSegmentCrossings(mstEdges);

            List<Point> addSites;
            List<SymmetricSegment> addEdges;

            this.AddNodesForBoxSegmentOverlaps(out addSites, out addEdges);

            treeEdges.AddRange(addEdges);

            this.InitConnectedComponents(treeEdges);

            this.treeSegments = treeEdges;
        }

        public List<SymmetricSegment> GetTreeEdges()
        {
            return this.treeSegments;
        }
    }
}
