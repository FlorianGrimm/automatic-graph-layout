using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.ProjectionSolver;

namespace Microsoft.Msagl.Layout.Layered {
    internal class ConstrainedOrdering {
        private readonly GeometryGraph geometryGraph;
        private readonly BasicGraph<Node, PolyIntEdge> intGraph;
        internal ProperLayeredGraph ProperLayeredGraph;
        private readonly int[] initialLayering;
        private LayerInfo[] layerInfos;
        internal LayerArrays LayerArrays;
        private readonly HorizontalConstraintsForSugiyama horizontalConstraints;
        private int numberOfNodesOfProperGraph;
        private readonly Database database;
        private double[][] xPositions;
        private int[][] yetBestLayers;
        private readonly List<PolyIntEdge> verticalEdges = new List<PolyIntEdge>();
        private readonly AdjacentSwapsWithConstraints adjSwapper;
        private SugiyamaLayoutSettings settings;
        private int numberOfLayers = -1;
        private int noGainSteps;
        private const int MaxNumberOfNoGainSteps=5;

        private int NumberOfLayers {
            get {
                if (this.numberOfLayers > 0) {
                    return this.numberOfLayers;
                }

                return this.numberOfLayers = this.initialLayering.Max(i => i + 1);
            }
        }

        private double NodeSeparation() {
            return this.settings.NodeSeparation;
        }

        internal ConstrainedOrdering(
            GeometryGraph geomGraph,
            BasicGraph<Node, PolyIntEdge> basicIntGraph,
            int[] layering,
            Dictionary<Node, int> nodeIdToIndex,
            Database database,
            SugiyamaLayoutSettings settings) {

            this.settings = settings;
            this.horizontalConstraints = settings.HorizontalConstraints;

            this.horizontalConstraints.PrepareForOrdering(nodeIdToIndex, layering);

            this.geometryGraph = geomGraph;
            this.database = database;
            this.intGraph = basicIntGraph;
            this.initialLayering = layering;
            //this has to be changed only to insert layers that are needed
            if (this.NeedToInsertLayers(layering)) {
                for (int i = 0; i < layering.Length; i++) {
                    layering[i] *= 2;
                }

                this.LayersAreDoubled = true;
                this.numberOfLayers = -1;
            }

            this.PrepareProperLayeredGraphAndFillLayerInfos();

            this.adjSwapper = new AdjacentSwapsWithConstraints(
                this.LayerArrays,
                this.HasCrossWeights(),
                this.ProperLayeredGraph,
                this.layerInfos);
        }

        private bool LayersAreDoubled { get; set; }

        private bool NeedToInsertLayers(int[] layering) {
            return ExistsShortLabeledEdge(layering, this.intGraph.Edges) ||
                   ExistsShortMultiEdge(layering, this.database.Multiedges);
        }

        private static bool ExistsShortMultiEdge(int[] layering, Dictionary<IntPair, List<PolyIntEdge>> multiedges) {
            return multiedges.Any(multiedge => multiedge.Value.Count > 2 && layering[multiedge.Key.x] == 1 + layering[multiedge.Key.y]);
        }

        internal void Calculate() {
            this.AllocateXPositions();
            var originalGraph = this.intGraph.Nodes[0].GeometryParent as GeometryGraph;
            LayeredLayoutEngine.CalculateAnchorSizes(this.database, out this.database.anchors, this.ProperLayeredGraph, originalGraph, this.intGraph, this.settings);
            LayeredLayoutEngine.CalcInitialYAnchorLocations(this.LayerArrays, 500, this.geometryGraph, this.database, this.intGraph, this.settings, this.LayersAreDoubled);
            this.Order();
        }

        private ConstrainedOrderMeasure CreateMeasure() {
            return new ConstrainedOrderMeasure(Ordering.GetCrossingsTotal(this.ProperLayeredGraph, this.LayerArrays));
        }

        private bool HasCrossWeights() {
            return this.ProperLayeredGraph.Edges.Any(le => le.CrossingWeight != 1);
        }

        private static bool ExistsShortLabeledEdge(int[] layering, IEnumerable<PolyIntEdge> edges) {
            return edges.Any(edge => layering[edge.Source] == layering[edge.Target] + 1 && edge.Edge.Label != null);
        }

        private void AllocateXPositions() {
            this.xPositions = new double[this.NumberOfLayers][];
            for (int i = 0; i < this.NumberOfLayers; i++) {
                this.xPositions[i] = new double[this.LayerArrays.Layers[i].Length];
            }
        }

        private void Order() {
            this.CreateInitialOrderInLayers();
            this.TryPushingOutStrangersFromHorizontalBlocks();
            int n = 5;

            ConstrainedOrderMeasure measure = null;
        
            while (n-- > 0 && this.noGainSteps <= MaxNumberOfNoGainSteps) {

                this.SetXPositions();
                
                ConstrainedOrderMeasure newMeasure = this.CreateMeasure();
                if (measure == null || newMeasure < measure) {
                    this.noGainSteps = 0;
                    Ordering.CloneLayers(this.LayerArrays.Layers, ref this.yetBestLayers);
                    measure = newMeasure;
                } else {
                    this.noGainSteps++;
                    this.RestoreState();
                }
                
            }
        }

        private void SetXPositions() {
            SolverShell solver = this.InitSolverWithoutOrder();
            this.ImproveWithAdjacentSwaps();
            this.PutLayerNodeSeparationsIntoSolver(solver);
            solver.Solve();
            this.SortLayers(solver);
            for (int i = 0; i < this.LayerArrays.Y.Length; i++) {
                this.database.Anchors[i].X = solver.GetVariableResolvedPosition(i);
            }
        }

        private SolverShell InitSolverWithoutOrder() {
            var solver=new SolverShell();
            this.InitSolverVars(solver);

            this.PutLeftRightConstraintsIntoSolver(solver);
            this.PutVerticalConstraintsIntoSolver(solver);
            this.AddGoalsToKeepProperEdgesShort(solver);

            this.AddGoalsToKeepFlatEdgesShort(solver);
            return solver;
        }

        private void SortLayers(SolverShell solver) {
            for (int i = 0; i < this.LayerArrays.Layers.Length; i++) {
                this.SortLayerBasedOnSolution(this.LayerArrays.Layers[i], solver);
            }
        }

        private void AddGoalsToKeepFlatEdgesShort(SolverShell solver) {
            foreach (var layerInfo in this.layerInfos) {
                AddGoalToKeepFlatEdgesShortOnBlockLevel(layerInfo, solver);
            }
        }

        private void InitSolverVars(SolverShell solver) {
            for (int i = 0; i < this.LayerArrays.Y.Length; i++) {
                solver.AddVariableWithIdealPosition(i, 0);
            }
        }

        private void AddGoalsToKeepProperEdgesShort(SolverShell solver) {
            foreach (var edge in this.ProperLayeredGraph.Edges) {
                solver.AddGoalTwoVariablesAreClose(edge.Source, edge.Target, PositionOverBaricenterWeight);
            }
        }

        private void PutVerticalConstraintsIntoSolver(SolverShell solver) {
            foreach (var pair in this.horizontalConstraints.VerticalInts) {
                solver.AddGoalTwoVariablesAreClose(pair.Item1, pair.Item2, ConstrainedVarWeight);
            }
        }

        private void PutLeftRightConstraintsIntoSolver(SolverShell solver) {
            foreach (var pair in this.horizontalConstraints.LeftRighInts) {
                solver.AddLeftRightSeparationConstraint(pair.Item1, pair.Item2, this.SimpleGapBetweenTwoNodes(pair.Item1, pair.Item2));
            }
        }

        private void PutLayerNodeSeparationsIntoSolver(SolverShell solver) {
            foreach (var layer in this.LayerArrays.Layers) {
                for (int i = 0; i < layer.Length - 1; i++) {
                    int l = layer[i];
                    int r = layer[i + 1];
                    solver.AddLeftRightSeparationConstraint(l, r, this.SimpleGapBetweenTwoNodes(l, r));
                }
            }
        }

        private void ImproveWithAdjacentSwaps() {
            this.adjSwapper.DoSwaps();
        }

        private void TryPushingOutStrangersFromHorizontalBlocks() {

        }

        private void CreateInitialOrderInLayers() {
            //the idea is to topologically ordering all nodes horizontally, by using vertical components, then fill the layers according to this order
            Dictionary<int, int> nodesToVerticalComponentsRoots = this.CreateVerticalComponents();
            IEnumerable<IntPair> liftedLeftRightRelations = this.LiftLeftRightRelationsToComponentRoots(nodesToVerticalComponentsRoots).ToArray();
            int[] orderOfVerticalComponentRoots = TopologicalSort.GetOrderOnEdges(liftedLeftRightRelations);
            this.FillLayersWithVerticalComponentsOrder(orderOfVerticalComponentRoots, nodesToVerticalComponentsRoots);
            this.LayerArrays.UpdateXFromLayers();
        }

        private void FillLayersWithVerticalComponentsOrder(int[] order, Dictionary<int, int> nodesToVerticalComponentsRoots) {
            Dictionary<int, List<int>> componentRootsToComponents = CreateComponentRootsToComponentsMap(nodesToVerticalComponentsRoots);
            var alreadyInLayers = new bool[this.LayerArrays.Y.Length];
            var runninglayerCounts = new int[this.LayerArrays.Layers.Length];
            foreach (var vertCompRoot in order) {
                this.PutVerticalComponentIntoLayers(this.EnumerateVertComponent(componentRootsToComponents, vertCompRoot), runninglayerCounts, alreadyInLayers);
            }

            for (int i = 0; i < this.ProperLayeredGraph.NodeCount; i++) {
                if (alreadyInLayers[i] == false) {
                    this.AddVertToLayers(i, runninglayerCounts, alreadyInLayers);
                }
            }
        }

        private IEnumerable<int> EnumerateVertComponent(Dictionary<int, List<int>> componentRootsToComponents, int vertCompRoot) {
            List<int> compList;
            if (componentRootsToComponents.TryGetValue(vertCompRoot, out compList)) {
                foreach (var i in compList) {
                    yield return i;
                }
            } else {
                yield return vertCompRoot;
            }
        }

        private void PutVerticalComponentIntoLayers(IEnumerable<int> vertComponent, int[] runningLayerCounts, bool[] alreadyInLayers) {
            foreach (var i in vertComponent) {
                this.AddVertToLayers(i, runningLayerCounts, alreadyInLayers);
            }
        }

        private void AddVertToLayers(int i, int[] runningLayerCounts, bool[] alreadyInLayers) {
            if (alreadyInLayers[i]) {
                return;
            }

            int layerIndex = this.LayerArrays.Y[i];

            int xIndex = runningLayerCounts[layerIndex];
            var layer = this.LayerArrays.Layers[layerIndex];

            layer[xIndex++] = i;
            alreadyInLayers[i] = true;
            List<int> block;
            if (this.horizontalConstraints.BlockRootToBlock.TryGetValue(i, out block)) {
                foreach (var v in block) {
                    if (alreadyInLayers[v]) {
                        continue;
                    }

                    layer[xIndex++] = v;
                    alreadyInLayers[v] = true;
                }
            }

            runningLayerCounts[layerIndex] = xIndex;
        }

        private static Dictionary<int, List<int>> CreateComponentRootsToComponentsMap(Dictionary<int, int> nodesToVerticalComponentsRoots) {
            var d = new Dictionary<int, List<int>>();
            foreach (var kv in nodesToVerticalComponentsRoots) {
                int i = kv.Key;
                var root = kv.Value;
                List<int> component;
                if (!d.TryGetValue(root, out component)) {
                    d[root] = component = new List<int>();
                }
                component.Add(i);
            }
            return d;
        }

        private IEnumerable<IntPair> LiftLeftRightRelationsToComponentRoots(Dictionary<int, int> nodesToVerticalComponentsRoots) {
            foreach (var pair in this.horizontalConstraints.LeftRighInts) {
                yield return new IntPair(GetFromDictionaryOrIdentical(nodesToVerticalComponentsRoots, pair.Item1),
                    GetFromDictionaryOrIdentical(nodesToVerticalComponentsRoots, pair.Item2));
            }

            foreach (var pair in this.horizontalConstraints.LeftRightIntNeibs) {
                yield return new IntPair(GetFromDictionaryOrIdentical(nodesToVerticalComponentsRoots, pair.Item1),
                    GetFromDictionaryOrIdentical(nodesToVerticalComponentsRoots, pair.Item2));
            }
        }

        private static int GetFromDictionaryOrIdentical(Dictionary<int, int> d, int key) {
            int i;
            if (d.TryGetValue(key, out i)) {
                return i;
            }

            return key;
        }

        /// <summary>
        /// These blocks are connected components in the vertical constraints. They don't necesserely span consequent layers.
        /// </summary>
        /// <returns></returns>
        private Dictionary<int, int> CreateVerticalComponents() {
            var vertGraph = new BasicGraphOnEdges<PolyIntEdge>(from pair in this.horizontalConstraints.VerticalInts select new PolyIntEdge(pair.Item1, pair.Item2));
            var verticalComponents = ConnectedComponentCalculator<PolyIntEdge>.GetComponents(vertGraph);
            var nodesToComponentRoots = new Dictionary<int, int>();
            foreach (var component in verticalComponents) {
                var ca = component.ToArray();
                if (ca.Length == 1) {
                    continue;
                }

                int componentRoot = -1;
                foreach (var j in component) {
                    if (componentRoot == -1) {
                        componentRoot = j;
                    }

                    nodesToComponentRoots[j] = componentRoot;
                }
            }
            return nodesToComponentRoots;
        }

        private void RestoreState() {
            this.LayerArrays.UpdateLayers(this.yetBestLayers);
        }

#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledCode")]
        private void Show() {
            SugiyamaLayoutSettings.ShowDatabase(this.database);
        }
#endif

#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "System.Diagnostics.Debug.Write(System.String)"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledCode")]
        private static void PrintPositions(double[] positions) {
            for (int j = 0; j < positions.Length; j++) {
                System.Diagnostics.Debug.Write(" " + positions[j]);
            }

            System.Diagnostics.Debug.WriteLine("");
        }
#endif


        private void SortLayerBasedOnSolution(int[] layer, SolverShell solver) {
            int length = layer.Length;
            var positions = new double[length];
            int k = 0;
            foreach (int v in layer) {
                positions[k++] = solver.GetVariableResolvedPosition(v);
            }

            Array.Sort(positions, layer);
            int i = 0;
            foreach (int v in layer) {
                this.LayerArrays.X[v] = i++;
            }
        }

        private const double ConstrainedVarWeight = 10e6;
        private const double PositionOverBaricenterWeight = 5;

        private static int NodeToBlockRootSoftOnLayerInfo(LayerInfo layerInfo, int node) {
            int root;
            return layerInfo.nodeToBlockRoot.TryGetValue(node, out root) ? root : node;
        }

        private static void AddGoalToKeepFlatEdgesShortOnBlockLevel(LayerInfo layerInfo, SolverShell solver) {
            if (layerInfo != null) {
                foreach (var couple in layerInfo.flatEdges) {
                    int sourceBlockRoot = NodeToBlockRootSoftOnLayerInfo(layerInfo, couple.Item1);
                    int targetBlockRoot = NodeToBlockRootSoftOnLayerInfo(layerInfo, couple.Item2);
                    if (sourceBlockRoot != targetBlockRoot) {
                        solver.AddGoalTwoVariablesAreClose(sourceBlockRoot, targetBlockRoot);
                    }
                }
            }
        }

        private static bool NodeIsConstrainedBelow(int v, LayerInfo layerInfo) {
            if (layerInfo == null) {
                return false;
            }

            return layerInfo.constrainedFromBelow.ContainsKey(v);
        }

        private static bool NodeIsConstrainedAbove(int v, LayerInfo layerInfo) {
            if (layerInfo == null) {
                return false;
            }

            return layerInfo.constrainedFromAbove.ContainsKey(v);
        }

        internal static bool BelongsToNeighbBlock(int p, LayerInfo layerInfo) {
            return layerInfo != null && (layerInfo.nodeToBlockRoot.ContainsKey(p) || layerInfo.neigBlocks.ContainsKey(p));
            //p is a root of the block
        }

        private static bool NodesAreConstrainedBelow(int leftNode, int rightNode, LayerInfo layerInfo) {
            return NodeIsConstrainedBelow(leftNode, layerInfo) && NodeIsConstrainedBelow(rightNode, layerInfo);
        }

        private static bool NodesAreConstrainedAbove(int leftNode, int rightNode, LayerInfo layerInfo) {
            return NodeIsConstrainedAbove(leftNode, layerInfo) && NodeIsConstrainedAbove(rightNode, layerInfo);
        }

        private double GetGapFromNodeNodesConstrainedBelow(int leftNode, int rightNode, LayerInfo layerInfo,
                                                          int layerIndex) {
            double gap = this.SimpleGapBetweenTwoNodes(leftNode, rightNode);
            leftNode = layerInfo.constrainedFromBelow[leftNode];
            rightNode = layerInfo.constrainedFromBelow[rightNode];
            layerIndex--;
            layerInfo = this.layerInfos[layerIndex];
            if (layerIndex > 0 && NodesAreConstrainedBelow(leftNode, rightNode, layerInfo)) {
                return Math.Max(gap, this.GetGapFromNodeNodesConstrainedBelow(leftNode, rightNode, layerInfo, layerIndex));
            }

            return Math.Max(gap, this.SimpleGapBetweenTwoNodes(leftNode, rightNode));
        }

        private double GetGapFromNodeNodesConstrainedAbove(int leftNode, int rightNode, LayerInfo layerInfo,
                                                          int layerIndex) {
            double gap = this.SimpleGapBetweenTwoNodes(leftNode, rightNode);
            leftNode = layerInfo.constrainedFromAbove[leftNode];
            rightNode = layerInfo.constrainedFromAbove[rightNode];
            layerIndex++;
            layerInfo = this.layerInfos[layerIndex];
            if (layerIndex < this.LayerArrays.Layers.Length - 1 && NodesAreConstrainedAbove(leftNode, rightNode, layerInfo)) {
                return Math.Max(gap, this.GetGapFromNodeNodesConstrainedAbove(leftNode, rightNode, layerInfo, layerIndex));
            }

            return Math.Max(gap, this.SimpleGapBetweenTwoNodes(leftNode, rightNode));
        }

        private double SimpleGapBetweenTwoNodes(int leftNode, int rightNode) {
            return this.database.anchors[leftNode].RightAnchor +
                   this.NodeSeparation() + this.database.anchors[rightNode].LeftAnchor;
        }

        private void PrepareProperLayeredGraphAndFillLayerInfos() {
            this.layerInfos = new LayerInfo[this.NumberOfLayers];
            this.CreateProperLayeredGraph();
            this.CreateExtendedLayerArrays();
            this.FillBlockRootToBlock();
            this.FillLeftRightPairs();
            this.FillFlatEdges();
            this.FillAboveBelow();
            this.FillBlockRootToVertConstrainedNode();
        }

        private void FillBlockRootToVertConstrainedNode() {
            foreach (LayerInfo layerInfo in this.layerInfos) {
                foreach (int v in VertConstrainedNodesOfLayer(layerInfo)) {
                    int blockRoot;
                    if (TryGetBlockRoot(v, out blockRoot, layerInfo)) {
                        layerInfo.blockRootToVertConstrainedNodeOfBlock[blockRoot] = v;
                    }
                }
            }
        }

        private static bool TryGetBlockRoot(int v, out int blockRoot, LayerInfo layerInfo) {
            if (layerInfo.nodeToBlockRoot.TryGetValue(v, out blockRoot)) {
                return true;
            }

            if (layerInfo.neigBlocks.ContainsKey(v)) {
                blockRoot = v;
                return true;
            }
            return false;
        }

        private static IEnumerable<int> VertConstrainedNodesOfLayer(LayerInfo layerInfo) {
            if (layerInfo != null) {
                foreach (int v in layerInfo.constrainedFromAbove.Keys) {
                    yield return v;
                }

                foreach (int v in layerInfo.constrainedFromBelow.Keys) {
                    yield return v;
                }
            }
        }

        private void CreateExtendedLayerArrays() {
            var layeringExt = new int[this.numberOfNodesOfProperGraph];
            Array.Copy(this.initialLayering, layeringExt, this.initialLayering.Length);
            foreach (PolyIntEdge edge in this.ProperLayeredGraph.BaseGraph.Edges) {
                var ledges = (LayerEdge[])edge.LayerEdges;
                if (ledges != null && ledges.Length > 1) {
                    int layerIndex = this.initialLayering[edge.Source] - 1;
                    for (int i = 0; i < ledges.Length - 1; i++) {
                        layeringExt[ledges[i].Target] = layerIndex--;
                    }
                }
            }
            this.LayerArrays = new LayerArrays(layeringExt);
        }

        private void CreateProperLayeredGraph() {
            IEnumerable<PolyIntEdge> edges = this.CreatePathEdgesOnIntGraph();
            var nodeCount = Math.Max(this.intGraph.NodeCount, BasicGraph<Node, PolyIntEdge>.VertexCount(edges));
            var baseGraph = new BasicGraph<Node, PolyIntEdge>(edges, nodeCount) { Nodes = this.intGraph.Nodes };
            this.ProperLayeredGraph = new ProperLayeredGraph(baseGraph);
        }

        private IEnumerable<PolyIntEdge> CreatePathEdgesOnIntGraph() {
            this.numberOfNodesOfProperGraph = this.intGraph.NodeCount;
            var ret = new List<PolyIntEdge>();
            foreach (PolyIntEdge ie in this.intGraph.Edges) {
                if (this.initialLayering[ie.Source] > this.initialLayering[ie.Target]) {
                    this.CreateLayerEdgesUnderIntEdge(ie);
                    ret.Add(ie);
                    if (this.horizontalConstraints.VerticalInts.Contains(new Tuple<int, int>(ie.Source, ie.Target))) {
                        this.verticalEdges.Add(ie);
                    }
                }
            }

            return ret;
        }

        private void CreateLayerEdgesUnderIntEdge(PolyIntEdge ie) {
            int source = ie.Source;
            int target = ie.Target;

            int span = LayeredLayoutEngine.EdgeSpan(this.initialLayering, ie);
            ie.LayerEdges = new LayerEdge[span];
            Debug.Assert(span > 0);
            if (span == 1) {
                ie.LayerEdges[0] = new LayerEdge(ie.Source, ie.Target, ie.CrossingWeight);
            } else {
                ie.LayerEdges[0] = new LayerEdge(source, this.numberOfNodesOfProperGraph, ie.CrossingWeight);
                for (int i = 0; i < span - 2; i++) {
                    ie.LayerEdges[i + 1] = new LayerEdge(this.numberOfNodesOfProperGraph++, this.numberOfNodesOfProperGraph,
                                                         ie.CrossingWeight);
                }

                ie.LayerEdges[span - 1] = new LayerEdge(this.numberOfNodesOfProperGraph++, target, ie.CrossingWeight);
            }
        }

        private void FillAboveBelow() {
            foreach (PolyIntEdge ie in this.verticalEdges) {
                foreach (LayerEdge le in ie.LayerEdges) {
                    int upper = le.Source;
                    int lower = le.Target;
                    this.RegisterAboveBelowOnConstrainedUpperLower(upper, lower);
                }
            }

            foreach (var p in this.horizontalConstraints.VerticalInts) {
                this.RegisterAboveBelowOnConstrainedUpperLower(p.Item1, p.Item2);
            }
        }

        private void RegisterAboveBelowOnConstrainedUpperLower(int upper, int lower) {
            LayerInfo topLayerInfo = this.GetOrCreateLayerInfo(this.LayerArrays.Y[upper]);
            LayerInfo bottomLayerInfo = this.GetOrCreateLayerInfo(this.LayerArrays.Y[lower]);

            topLayerInfo.constrainedFromBelow[upper] = lower;
            bottomLayerInfo.constrainedFromAbove[lower] = upper;
        }

        private void FillFlatEdges() {
            foreach (PolyIntEdge edge in this.intGraph.Edges) {
                int l = this.initialLayering[edge.Source];
                if (l == this.initialLayering[edge.Target]) {
                    this.GetOrCreateLayerInfo(l).flatEdges.Insert(new Tuple<int, int>(edge.Source, edge.Target));
                }
            }
        }

        private void FillLeftRightPairs() {
            foreach (var p in this.horizontalConstraints.LeftRighInts) {
                LayerInfo layerInfo = this.GetOrCreateLayerInfo(this.initialLayering[p.Item1]);
                layerInfo.leftRight.Insert(p);
            }
        }

        /// <summary>
        /// when we call this function we know that a LayerInfo is needed
        /// </summary>
        /// <param name="layerNumber"></param>
        /// <returns></returns>
        private LayerInfo GetOrCreateLayerInfo(int layerNumber) {
            LayerInfo layerInfo = this.layerInfos[layerNumber] ?? (this.layerInfos[layerNumber] = new LayerInfo());
            return layerInfo;
        }

        private void FillBlockRootToBlock() {
            foreach (var p in this.horizontalConstraints.BlockRootToBlock) {
                LayerInfo layerInfo = this.GetOrCreateLayerInfo(this.initialLayering[p.Key]);
                layerInfo.neigBlocks[p.Key] = p.Value;
                foreach (int i in p.Value) {
                    layerInfo.nodeToBlockRoot[i] = p.Key;
                }
            }
        }
    }
}