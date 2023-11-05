using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;

namespace Microsoft.Msagl.Prototype.Phylo {
    internal class PhyloTreeLayoutCalclulation : AlgorithmBase{
        private Anchor[] anchors;
        private ProperLayeredGraph properLayeredGraph;
        private Database dataBase;
        private PhyloTree tree;
        private BasicGraph<Node, PolyIntEdge> intGraph;
        private LayerArrays layerArrays;
        private SortedDictionary<int, double> gridLayerOffsets = new SortedDictionary<int, double>();
        private double[] layerOffsets;
        private double cellSize;
        private Dictionary<Node, double> nodeOffsets = new Dictionary<Node, double>();
        private Dictionary<Node, int> nodesToIndices = new Dictionary<Node, int>();
        private int[] originalNodeToGridLayerIndices;
        private Dictionary<int, int> gridLayerToLayer=new Dictionary<int,int>();

        ///// <summary>
        ///// the layout responsible for the algorithm parameters
        ///// </summary>
        internal SugiyamaLayoutSettings LayoutSettings { get; private set; }

        internal PhyloTreeLayoutCalclulation(PhyloTree phyloTreeP, SugiyamaLayoutSettings settings, BasicGraph<Node, PolyIntEdge> intGraphP, Database dataBase) {
            this.dataBase = dataBase;
            this.tree = phyloTreeP;
            this.LayoutSettings = settings;
            this.intGraph = intGraphP;
            this.originalNodeToGridLayerIndices = new int[this.intGraph.Nodes.Count];
        }

        protected override void RunInternal() {
            if (!this.IsATree()) {
                throw new InvalidDataException("the graph is not a tree");
            }

            this.DefineCellSize();
            this.CalculateOriginalNodeToGridLayerIndices();
            this.CreateLayerArraysAndProperLayeredGraph();
            this.FillDataBase();
            this.RunXCoordinateAssignmentsByBrandes();
            this.CalcTheBoxFromAnchors();
            this.StretchIfNeeded();
            this.ProcessPositionedAnchors();

            //            SugiyamaLayoutSettings.ShowDataBase(this.dataBase);

            this.RouteSplines();
        }

        private bool IsATree() {
            Set<Node> visited = new Set<Node>();
            Node root = this.tree.Nodes.FirstOrDefault(n => !n.InEdges.Any());
            if (root == null) {
                return false;
            }

            return IsATreeUnderNode(root, visited) && visited.Count== this.tree.Nodes.Count;
        }

        private static bool IsATreeUnderNode(Node node, Set<Node> visited) {
            if (visited.Contains(node)) {
                return false;
            }

            visited.Insert(node);
            return node.OutEdges.All(outEdge => IsATreeUnderNode(outEdge.Target, visited));
        }

        private void StretchIfNeeded() {
            if (this.LayoutSettings.AspectRatio != 0) {
                double aspectRatio = this.tree.Width / this.tree.Height;
                this.StretchToDesiredAspectRatio(aspectRatio, this.LayoutSettings.AspectRatio);
            }
        }

        private void StretchToDesiredAspectRatio(double aspectRatio, double desiredAR) {
            if (aspectRatio > desiredAR) {
                this.StretchInYDirection(aspectRatio / desiredAR);
            } else if (aspectRatio < desiredAR) {
                this.StretchInXDirection(desiredAR / aspectRatio);
            }
        }
        private void StretchInYDirection(double scaleFactor) {
            double center = (this.tree.BoundingBox.Top + this.tree.BoundingBox.Bottom) / 2;
            foreach (Anchor a in this.dataBase.Anchors) {
                a.BottomAnchor *= scaleFactor;
                a.TopAnchor *= scaleFactor;
                a.Y = center + scaleFactor * (a.Y - center);
            }
            double h = this.tree.Height * scaleFactor;
            this.tree.BoundingBox = new Rectangle(this.tree.BoundingBox.Left, center + h / 2, this.tree.BoundingBox.Right, center - h / 2);

        }
       
        private void StretchInXDirection(double scaleFactor) {
            double center = (this.tree.BoundingBox.Left + this.tree.BoundingBox.Right) / 2;
            foreach (Anchor a in this.dataBase.Anchors) {
                a.LeftAnchor *= scaleFactor;
                a.RightAnchor *= scaleFactor;
                a.X = center + scaleFactor * (a.X - center);
            }
            double w = this.tree.Width * scaleFactor;
            this.tree.BoundingBox =
                new Rectangle(center - w / 2, this.tree.BoundingBox.Top,
                center + w / 2, this.tree.BoundingBox.Bottom);
        }

        private void DefineCellSize() {

            double min = double.MaxValue;
            foreach (PhyloEdge e in this.tree.Edges) {
                min = Math.Min(min, e.Length);
            }

            this.cellSize=0.3*min;

        }

        private void CalculateOriginalNodeToGridLayerIndices() {
            this.InitNodesToIndices();
            this.FillNodeOffsets();
            foreach (KeyValuePair<Node, double> kv in this.nodeOffsets) {
                int nodeIndex = this.nodesToIndices[kv.Key];
                int gridLayerIndex= this.originalNodeToGridLayerIndices[nodeIndex] = this.GetGridLayerIndex(kv.Value);
                if (!this.gridLayerOffsets.ContainsKey(gridLayerIndex)) {
                    this.gridLayerOffsets[gridLayerIndex] = kv.Value;
                }
            }
        }

        private int GetGridLayerIndex(double len) {
            return (int)(len / this.cellSize + 0.5);
        }

        private void InitNodesToIndices() {
            for (int i = 0; i < this.intGraph.Nodes.Count; i++) {
                this.nodesToIndices[this.intGraph.Nodes[i]] = i;
            }
        }

        private void FillNodeOffsets() {
            this.FillNodeOffsets(0.0, this.tree.Root);
        }

        private void FillNodeOffsets(double p, Node node) {
            this.nodeOffsets[node] = p;
            foreach (PhyloEdge e in node.OutEdges) {
                this.FillNodeOffsets(p+e.Length, e.Target);
            }
        }


        private  void FillDataBase() {
            foreach (PolyIntEdge e in this.intGraph.Edges) {
                this.dataBase.RegisterOriginalEdgeInMultiedges(e);
            }

            this.SizeAnchors();
            this.FigureYCoordinates();

        }

        private void FigureYCoordinates() {
            double m = this.GetMultiplier();
            int root = this.nodesToIndices[this.tree.Root];
            this.CalculateAnchorsY(root,m,0);

            for (int i = this.intGraph.NodeCount; i < this.dataBase.Anchors.Length; i++) {
                this.dataBase.Anchors[i].Y = -m * this.layerOffsets[this.layerArrays.Y[i]];
            }

            //fix layer offsets
            for (int i = 0; i < this.layerOffsets.Length; i++) {
                this.layerOffsets[i] *= m;
            }
        }

        private double GetMultiplier() {
            double m = 1;
            for (int i = this.layerArrays.Layers.Length - 1; i > 0; i--) {
                double nm = this.GetMultiplierBetweenLayers(i);
                if (nm > m) {
                    m = nm;
                }
            }

            return m;
        }

        private double GetMultiplierBetweenLayers(int i) {
            int a = this.FindLowestBottomOnLayer(i);
            int b = this.FindHighestTopOnLayer(i - 1);
            double ay = this.NodeY(i, a);
            double by = this.NodeY(i - 1, b);
            // we need to have m*(a[y]-b[y])>=anchors[a].BottomAnchor+anchors[b].TopAnchor+layerSeparation;
            double diff = ay - by;
            if (diff < 0) {
                throw new InvalidOperationException();
            }

            double nm = (this.dataBase.Anchors[a].BottomAnchor + this.dataBase.Anchors[b].TopAnchor + this.LayoutSettings.LayerSeparation) / diff;
            if (nm > 1) {
                return nm;
            }

            return 1;
        }

        private int FindHighestTopOnLayer(int layerIndex) {
            int[] layer = this.layerArrays.Layers[layerIndex];
            int ret = layer[0];
            double top = this.NodeY(layerIndex, ret) + this.dataBase.Anchors[ret].TopAnchor;
            for (int i = 1; i < layer.Length; i++) {
                int node=layer[i];
                double nt = this.NodeY(layerIndex, node) + this.dataBase.Anchors[node].TopAnchor;
                if (nt > top) {
                    top = nt;
                    ret = node;
                }

            }
            return ret;
        }

        private int FindLowestBottomOnLayer(int layerIndex) {
            int[] layer = this.layerArrays.Layers[layerIndex];
            int ret = layer[0];
            double bottom = this.NodeY(layerIndex, ret) - this.dataBase.Anchors[ret].BottomAnchor;
            for (int i = 1; i < layer.Length; i++) {
                int node = layer[i];
                double nb = this.NodeY(layerIndex, node) - this.dataBase.Anchors[node].BottomAnchor;
                if (nb < bottom) {
                    bottom = nb;
                    ret = node;
                }

            }
            return ret;
        }

        private double NodeY(int layer, int node) {
            return - (this.IsOriginal(node) ? this.nodeOffsets[this.intGraph.Nodes[node]] : this.layerOffsets[layer]);
        }

        private bool IsOriginal(int node) {
            return node < this.intGraph.NodeCount;
        }

        private void CalculateAnchorsY(int node, double m, double y) {
            //go over original nodes
            this.dataBase.Anchors[node].Y = -y;
            foreach (PolyIntEdge e in this.intGraph.OutEdges(node)) {
                this.CalculateAnchorsY(e.Target, m, y + e.Edge.Length * m);
            }
        }

        private void SizeAnchors() {
            this.dataBase.Anchors = this.anchors = new Anchor[this.properLayeredGraph.NodeCount];

            for (int i = 0; i < this.anchors.Length; i++) {
                this.anchors[i] = new Anchor(this.LayoutSettings.LabelCornersPreserveCoefficient);
            }

            //go over the old vertices
            for (int i = 0; i < this.intGraph.NodeCount; i++) {
                this.CalcAnchorsForOriginalNode(i);
            }

            //go over virtual vertices
            foreach (PolyIntEdge intEdge in this.dataBase.AllIntEdges) {
                if (intEdge.LayerEdges != null) {
                    foreach (LayerEdge layerEdge in intEdge.LayerEdges) {
                        int v = layerEdge.Target;
                        if (v != intEdge.Target) {
                            Anchor anchor = this.anchors[v];
                            if (!this.dataBase.MultipleMiddles.Contains(v)) {
                                anchor.LeftAnchor = anchor.RightAnchor = VirtualNodeWidth / 2.0f;
                                anchor.TopAnchor = anchor.BottomAnchor = this.VirtualNodeHeight / 2.0f;
                            } else {
                                anchor.LeftAnchor = anchor.RightAnchor = VirtualNodeWidth * 4;
                                anchor.TopAnchor = anchor.BottomAnchor = this.VirtualNodeHeight / 2.0f;
                            }
                        }
                    }
                    //fix label vertices      

                    if (intEdge.Edge.Label!=null) {
                        int lj = intEdge.LayerEdges[intEdge.LayerEdges.Count / 2].Source;
                        Anchor a = this.anchors[lj];
                        double w = intEdge.LabelWidth, h = intEdge.LabelHeight;
                        a.RightAnchor = w;
                        a.LeftAnchor = this.LayoutSettings.NodeSeparation;

                        if (a.TopAnchor < h / 2.0) {
                            a.TopAnchor = a.BottomAnchor = h / 2.0;
                        }

                        a.LabelToTheRightOfAnchorCenter = true;
                    }
                }
            }

        }

        /// <summary>
        /// the width of dummy nodes
        /// </summary>
        private static double VirtualNodeWidth {
            get {
                return 1;
            }
        }

        /// <summary>
        /// the height of dummy nodes
        /// </summary>
        private double VirtualNodeHeight {
            get {
                return this.LayoutSettings.MinNodeHeight * 1.5f / 8;
            }
        }

        private void CalcAnchorsForOriginalNode(int i) {

            double leftAnchor = 0;
            double rightAnchor = leftAnchor;
            double topAnchor = 0;
            double bottomAnchor = topAnchor;

            //that's what we would have without the label and multiedges 

            if (this.intGraph.Nodes != null) {
                Node node = this.intGraph.Nodes[i];
                ExtendStandardAnchors(ref leftAnchor, ref rightAnchor, ref topAnchor, ref bottomAnchor, node);
            }

            this.RightAnchorMultiSelfEdges(i, ref rightAnchor, ref topAnchor, ref bottomAnchor);

            double hw = this.LayoutSettings.MinNodeWidth / 2;
            if (leftAnchor < hw) {
                leftAnchor = hw;
            }

            if (rightAnchor < hw) {
                rightAnchor = hw;
            }

            double hh = this.LayoutSettings.MinNodeHeight / 2;

            if (topAnchor < hh) {
                topAnchor = hh;
            }

            if (bottomAnchor < hh) {
                bottomAnchor = hh;
            }

            this.anchors[i] = new Anchor(leftAnchor, rightAnchor, topAnchor, bottomAnchor, this.intGraph.Nodes[i], this.LayoutSettings.LabelCornersPreserveCoefficient)
                         {Padding = this.intGraph.Nodes[i].Padding};
#if TEST_MSAGL
            //anchors[i].Id = this.intGraph.Nodes[i].Id;
#endif
        }

        private static void ExtendStandardAnchors(ref double leftAnchor, ref double rightAnchor, ref double topAnchor, ref double bottomAnchor, Node node) {
            double w = node.Width;
            double h = node.Height;


            w /= 2.0;
            h /= 2.0;


            rightAnchor = leftAnchor = w;
            topAnchor = bottomAnchor = h;
        }

        private void RightAnchorMultiSelfEdges(int i, ref double rightAnchor, ref double topAnchor, ref double bottomAnchor) {
            double delta = this.WidthOfSelfeEdge(i, ref rightAnchor, ref topAnchor, ref bottomAnchor);

            rightAnchor += delta;
        }


        private double WidthOfSelfeEdge(int i, ref double rightAnchor, ref double topAnchor, ref double bottomAnchor) {
            double delta = 0;
            List<PolyIntEdge> multiedges = this.dataBase.GetMultiedge(i, i);
            //it could be a multiple self edge
            if (multiedges.Count > 0) {
                foreach (PolyIntEdge e in multiedges) {
                    if (e.Edge.Label != null) {
                        rightAnchor += e.Edge.Label.Width;
                        if (topAnchor < e.Edge.Label.Height / 2.0) {
                            topAnchor = bottomAnchor = e.Edge.Label.Height / 2.0f;
                        }
                    }
                }

                delta += (this.LayoutSettings.NodeSeparation + this.LayoutSettings.MinNodeWidth) * multiedges.Count;
            }
            return delta;
        }

        private  void RouteSplines() {
            Layout.Layered.Routing routing = new Layout.Layered.Routing(this.LayoutSettings, this.tree, this.dataBase, this.layerArrays, this.properLayeredGraph, null);
            routing.Run();
        }

        private  void RunXCoordinateAssignmentsByBrandes() {
            XCoordsWithAlignment.CalculateXCoordinates(this.layerArrays, this.properLayeredGraph, this.tree.Nodes.Count, this.dataBase.Anchors, this.LayoutSettings.NodeSeparation);
        }

        private  void CreateLayerArraysAndProperLayeredGraph() {
            int numberOfLayers = this.gridLayerOffsets.Count;
            this.layerOffsets=new double[numberOfLayers];
            int i = numberOfLayers-1;

            foreach (KeyValuePair<int, double> kv in this.gridLayerOffsets) {
                this.layerOffsets[i] = kv.Value;
                this.gridLayerToLayer[kv.Key] = i--;
            }

            int nOfNodes= this.CountTotalNodesIncludingVirtual(this.nodesToIndices[this.tree.Root]);

            ////debugging !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //int tt = 0;
            //foreach (IntEdge ie in this.intGraph.Edges)
            //    tt += this.OriginalNodeLayer(ie.Source) - this.OriginalNodeLayer(ie.Target) - 1;
            //if (tt + this.intGraph.Nodes.Count != nOfNodes)
            //    throw new Exception();

            int[] layering = new int[nOfNodes];

            List<int>[] layers = new List<int>[numberOfLayers];
            for (i = 0; i < numberOfLayers; i++) {
                layers[i] = new List<int>();
            }

            this.WalkTreeAndInsertLayerEdges(layering, layers);

            this.layerArrays = new LayerArrays(layering);

            int[][]ll= this.layerArrays.Layers=new int[numberOfLayers][];

            i = 0;
            foreach (List<int> layer in layers) {
                ll[i++] = layer.ToArray();
            }

            this.properLayeredGraph = new ProperLayeredGraph(this.intGraph);
        }

    
        private int CountTotalNodesIncludingVirtual(int node) {
            int ret = 1;
            foreach (PolyIntEdge edge in this.intGraph.OutEdges(node)) {
                ret += this.NumberOfVirtualNodesOnEdge(edge) + this.CountTotalNodesIncludingVirtual(edge.Target);
            }

            return ret;
        }

        private int NumberOfVirtualNodesOnEdge(PolyIntEdge edge) {
            return this.OriginalNodeLayer(edge.Source) - this.OriginalNodeLayer(edge.Target) - 1;
        }

        private int OriginalNodeLayer(int node) {
            return this.gridLayerToLayer[this.originalNodeToGridLayerIndices[node]];
        }

        private void WalkTreeAndInsertLayerEdges(int[] layering, List<int>[] layers) {
            int virtualNode = this.intGraph.NodeCount;
            int root = this.nodesToIndices[this.tree.Root];
            int l;
            layering[root] = l = this.OriginalNodeLayer(root);
            layers[l].Add(root);
            this.WalkTreeAndInsertLayerEdges(layering, layers, root , ref virtualNode);
        }

        private void WalkTreeAndInsertLayerEdges(int[] layering, List<int>[] layers, int node, ref int virtualNode) {
            foreach (PolyIntEdge edge in this.intGraph.OutEdges(node)) {
                this.InsertLayerEdgesForEdge(edge, layering, ref virtualNode, layers);
            }
        }

        private void InsertLayerEdgesForEdge(PolyIntEdge edge, int[] layering, ref int virtualNode, List<int>[] layers) {
            int span = this.OriginalNodeLayer(edge.Source) - this.OriginalNodeLayer(edge.Target);
            edge.LayerEdges=new LayerEdge[span];
            for (int i = 0; i < span; i++) {
                edge.LayerEdges[i] = new LayerEdge(GetSource(i, edge, ref virtualNode), GetTarget(i, span, edge, virtualNode), edge.CrossingWeight);
            }

            int l = this.OriginalNodeLayer(edge.Source) - 1;
            for (int i = 0; i < span; i++) {
                int node=edge.LayerEdges[i].Target;
                layering[node] = l;
                layers[l--].Add(node);
            }

            this.WalkTreeAndInsertLayerEdges(layering, layers, edge.Target, ref virtualNode);

        }

        static private int GetTarget(int i, int span, PolyIntEdge edge, int virtualNode) {
            if (i < span-1) {
                return virtualNode;
            }

            return edge.Target;
        }

        static private int GetSource(int i, PolyIntEdge edge, ref int virtualNode) {
            if (i > 0) {
                return virtualNode++;
            }

            return edge.Source;
        }

        private void ProcessPositionedAnchors() {
            for (int i = 0; i < this.tree.Nodes.Count; i++) {
                this.intGraph.Nodes[i].Center= this.anchors[i].Origin;
            }
        }

        private void CalcTheBoxFromAnchors() {
            if (this.anchors.Length > 0) {

                Rectangle box = new Rectangle(this.anchors[0].Left, this.anchors[0].Top, this.anchors[0].Right, this.anchors[0].Bottom);

                for (int i = 1; i < this.anchors.Length; i++) {
                    Anchor a = this.anchors[i];
                    box.Add(a.LeftTop);
                    box.Add(a.RightBottom);
                }


                double m = Math.Max(box.Width, box.Height);

                double delta = this.tree.Margins / 100.0 * m;

                Point del = new Point(-delta, delta);
                box.Add(box.LeftTop + del);
                box.Add(box.RightBottom - del);
                this.tree.BoundingBox = box;
            }
        }

    }
}
