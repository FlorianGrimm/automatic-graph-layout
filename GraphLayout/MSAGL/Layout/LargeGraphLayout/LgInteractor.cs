using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.GraphmapsWithMesh;
using Microsoft.Msagl.Layout.Incremental;
using Microsoft.Msagl.Layout.Initial;
using Microsoft.Msagl.Layout.LargeGraphLayout.NodeRailLevelCalculator;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Layout.MDS;
using Microsoft.Msagl.Layout.OverlapRemovalFixedSegments;
using Microsoft.Msagl.Miscellaneous;
using Microsoft.Msagl.Miscellaneous.ConstrainedSkeleton;
using Microsoft.Msagl.Miscellaneous.RegularGrid;
using Microsoft.Msagl.Routing.Visibility;
using Edge = Microsoft.Msagl.Core.Layout.Edge;
using LineSegment = Microsoft.Msagl.Core.Geometry.Curves.LineSegment;
using Point = Microsoft.Msagl.Core.Geometry.Point;
using Rectangle = Microsoft.Msagl.Core.Geometry.Rectangle;
using Size = Microsoft.Msagl.Core.DataStructures.Size;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
#if TEST_MSAGL
#endif

namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    /// <summary>
    ///     enables to interactively explore a large graph
    /// </summary>
    public class LgInteractor
    {
        private const bool ShrinkEdgeLengths = true;
        private readonly LgData _lgData;
        private readonly LgLayoutSettings _lgLayoutSettings;
        private readonly CancelToken _cancelToken;
        private readonly GeometryGraph _mainGeometryGraph;        
        public Tiling g;
        private Dictionary<Point, int> PointToId = new Dictionary<Point, int>();
        public Dictionary<Point, int> locationtoNode = new Dictionary<Point, int>();
        public Dictionary<int, Node> idToNode;

        /// <summary>
        /// </summary>
        private bool _runInParallel = true;
        private RailGraph _railGraph;
        private Rectangle _visibleRectangle;

        public Dictionary<LgNodeInfo, LabelPlacement> SelectedNodeLabels = new Dictionary<LgNodeInfo, LabelPlacement>();

        /// <summary>
        ///     constructor
        /// </summary>
        /// <param name="geometryGraph"></param>
        /// <param name="lgLayoutSettings"></param>
        /// <param name="cancelToken"></param>
        public LgInteractor(GeometryGraph geometryGraph, LgLayoutSettings lgLayoutSettings, CancelToken cancelToken)
        {
            this._mainGeometryGraph = geometryGraph;
            this._lgLayoutSettings = lgLayoutSettings;
            this._cancelToken = cancelToken;
            if (geometryGraph.LgData == null)
            {
                this._lgData = new LgData(geometryGraph)
                {
                    GeometryNodesToLgNodeInfos = lgLayoutSettings.GeometryNodesToLgNodeInfos
                };

                geometryGraph.LgData = this._lgData;
            }
            else
            {
                this._lgData = geometryGraph.LgData;
                this._lgLayoutSettings.GeometryNodesToLgNodeInfos = this._lgData.GeometryNodesToLgNodeInfos;
            }
            // add skeleton levels
            this.AddSkeletonLevels();
        }

        private double CurrentZoomLevel { get; set; }

        /// <summary>
        ///     this graph is currently visible set of nodes and pieces of edges
        /// </summary>
        public RailGraph RailGraph
        {
            get { return this._railGraph; }
        }

        /// <summary>
        /// </summary>
        public IDictionary<Edge, LgEdgeInfo> GeometryEdgesToLgEdgeInfos
        {
            get { return this._lgData.GeometryEdgesToLgEdgeInfos; }
        }

        public Set<LgNodeInfo> SelectedNodeInfos
        {
            get { return this._lgData.SelectedNodeInfos; }
        }

        private void AddSkeletonLevels()
        {
            for (int i = this._lgData.SkeletonLevels.Count; i < this._lgData.Levels.Count(); i++)
            {
                this._lgData.SkeletonLevels.Add(new LgSkeletonLevel { ZoomLevel = this._lgData.Levels[i].ZoomLevel });
            }
        }

        public Tiling[] calculateGraphsForEachZoomLevel(Tiling g, Dictionary<Node, int> nodeToId, bool hugeGraph)
        {
            var stopwatch = new Stopwatch();

            stopwatch.Start();
            List<Tiling> graphs = new List<Tiling>();
            Tiling OldGraphHolder = null;
            DijkstraAlgo dijkstra = new DijkstraAlgo();
            
            g.pathList = new Dictionary<Edge, List<int>>();
            Dictionary<Edge,List<VertexNeighbor>> ssp  = new Dictionary<Edge, List<VertexNeighbor>>();
            //for (int i = g.maxTheoreticalZoomLevel; i >= 1; i /= 2)
            for (int i = 1; i <= g.maxTheoreticalZoomLevel; i *= 2)            
            {
                //create a new graph and copy the vertex properties of the basic graph
                Tiling newG = new Tiling(g.NumOfnodes, true);                
                newG.N = g.N;
                for (int j = 0; j < g.NumOfnodes; j++)
                {
                    newG.VList[j] = new Vertex(g.VList[j].XLoc, g.VList[j].YLoc) { Id = g.VList[j].Id, ZoomLevel = g.VList[j].ZoomLevel };
                    newG.VList[j].TargetX = newG.VList[j].PreciseX = newG.VList[j].XLoc;
                    newG.VList[j].TargetY = newG.VList[j].PreciseY = newG.VList[j].YLoc;
                }

                //copy bottom level graph O(V+E)
                if (OldGraphHolder != null)
                {
                    for (int j = 0; j < OldGraphHolder.NumOfnodes; j++)
                    {
                        for (int k = 0; k < OldGraphHolder.DegList[j]; k++)
                        {
                            newG.AddEdge(j, OldGraphHolder.EList[j, k].NodeId, OldGraphHolder.EList[j, k].Selected, OldGraphHolder.EList[j, k].Used);
                        }
                    }
                } 
                 

 
                //create the edges of the new graph
                //for each edge (s,t), if both s and t are below ith zoomlevel, then find a s-t path

                //try lev's code for shortest path here

                foreach (Edge edge in this._mainGeometryGraph.Edges)
                {
                    int sourceZoomLevel = g.VList[nodeToId[edge.Source]].ZoomLevel;
                    int targetZoomLevel = g.VList[nodeToId[edge.Target]].ZoomLevel;
                    if (sourceZoomLevel <= i && targetZoomLevel <= i)
                    {

                        //if the edge is already in top level then copy it
                        if (sourceZoomLevel < i && targetZoomLevel < i)
                        {
                            /* 
                            //jyoti: this seems unnecesary since we copied the edges earlier
                            //I will remove this later if all the tests pass
                            int[] p = OldGraphHolder.pathList[edge].ToArray();
                            for (int pindex = 0; pindex < p.Length - 1; pindex++)
                            {
                                int dindex;
                                for (dindex = 0; dindex < g.DegList[p[pindex]]; dindex++)
                                    if (g.EList[p[pindex], dindex].NodeId == p[pindex + 1]) break;
                                newG.AddEdge(p[pindex], p[pindex + 1], g.EList[p[pindex], dindex].Selected, g.EList[p[pindex], dindex].Used);
                            }*/
                            newG.pathList[edge] = OldGraphHolder.pathList[edge];
                            continue;
                        }
                        

                        newG.pathList[edge] = new List<int>();
                        List<int> pathvertices;
                        if (!ssp.ContainsKey(edge))
                        {
                            pathvertices = dijkstra.MSAGLAstarShortestPath(g.VList, g.EList, g.DegList,
                                nodeToId[edge.Source], nodeToId[edge.Target], g.NumOfnodes);
                            if(pathvertices.Count ==0 ) {
                                Console.WriteLine("missing path!");
                            }

                            g.pathList.Add(edge, pathvertices);
                            ssp.Add(edge, dijkstra.Edgelist);                          
                        }

                        
                        foreach (int vertexId in g.pathList[edge]) {
                            newG.pathList[edge].Add(vertexId);
                        }

                        foreach (VertexNeighbor vn in ssp[edge]) {
                            newG.AddEdge(vn.A, g.EList[vn.A, vn.Neighbor].NodeId, g.EList[vn.A, vn.Neighbor].Selected, g.EList[vn.A, vn.Neighbor].Used);
                        }
                        //this is necessary, new edges are being created here
                        //foreach (VertexNeighbor vn in dijkstra.Edgelist)
                        //    newG.AddEdge(vn.A, g.EList[vn.A, vn.Neighbor].NodeId, g.EList[vn.A, vn.Neighbor].Selected, g.EList[vn.A, vn.Neighbor].Used);

                    }
                }
                OldGraphHolder = newG;
                graphs.Add(newG);
            }
            


            stopwatch.Stop();



            ComputePathSimplification(g, stopwatch, graphs);

            Tiling[] allGraphs = graphs.ToArray();



            for (int index = 0; index < allGraphs.Length; index++)
            {
                for (int nodeindex = allGraphs[index].N; nodeindex < allGraphs[index].NumOfnodes; nodeindex++)
                {
                    if (index < allGraphs.Length - 1)
                    {
                        allGraphs[index].VList[nodeindex].TargetX = allGraphs[index + 1].VList[nodeindex].PreciseX;
                        allGraphs[index].VList[nodeindex].TargetY = allGraphs[index + 1].VList[nodeindex].PreciseY;
                    }
                    else
                    {
                        allGraphs[index].VList[nodeindex].TargetX = allGraphs[index].VList[nodeindex].PreciseX;
                        allGraphs[index].VList[nodeindex].TargetY = allGraphs[index].VList[nodeindex].PreciseY;
                    }
                }
            }
            stopwatch.Stop();

            return allGraphs;
        }

        private static void ComputePathSimplification(Tiling g, Stopwatch stopwatch, List<Tiling> graphs)
        {
            Dictionary<int,int> cycle = new Dictionary<int, int>();

            stopwatch.Start();
            foreach (Tiling graph in graphs)
            {
                //find all degree two vertices.
                List<int> Deg2Vertices = new List<int>();
                for (int j = graph.N; j < graph.NumOfnodes; j++)
                {
                    if (graph.DegList[j] == 2) {
                        Deg2Vertices.Add(j);
                    }

                    graph.VList[j].Visited = false;
                }
                foreach (int w in Deg2Vertices)
                {
                    if (graph.VList[w].Visited) {
                        continue;
                    }
                    //find the one end of the path
                    int currentVertexId = w;
                    int oldVertexId = -1;
                    cycle[w] = 1;
                    while (graph.DegList[currentVertexId] == 2 && currentVertexId >= g.N)
                    {
                        int neighbor1 = graph.EList[currentVertexId, 0].NodeId;
                        ;
                        int neighbor2 = graph.EList[currentVertexId, 1].NodeId;
                        ;
                        if (oldVertexId != neighbor1)
                        {
                            oldVertexId = currentVertexId;
                            currentVertexId = neighbor1;
                        }
                        else if (oldVertexId != neighbor2)
                        {
                            oldVertexId = currentVertexId;
                            currentVertexId = neighbor2;
                        }
                        if (cycle.ContainsKey(currentVertexId))
                        {
                            cycle.Clear();
                            break;
                        }
                        cycle.Add(currentVertexId, 1);
                    }
                    if (cycle.Count == 0) {
                        continue;
                    }

                    //find the path from this end 
                    var tempId = oldVertexId;
                    oldVertexId = currentVertexId;
                    currentVertexId = tempId;

                    List<int> path = new List<int>();
                    path.Add(oldVertexId);
                    path.Add(currentVertexId);
                    graph.VList[oldVertexId].Visited = true;
                    graph.VList[currentVertexId].Visited = true;

                    //find the path and add it into a list
                    while (graph.DegList[currentVertexId] == 2 && currentVertexId >= g.N)
                    {
                        int neighbor1 = graph.EList[currentVertexId, 0].NodeId;
                        ;
                        int neighbor2 = graph.EList[currentVertexId, 1].NodeId;
                        ;
                        if (oldVertexId != neighbor1)
                        {
                            oldVertexId = currentVertexId;
                            currentVertexId = neighbor1;
                        }
                        else if (oldVertexId != neighbor2)
                        {
                            oldVertexId = currentVertexId;
                            currentVertexId = neighbor2;
                        }
                        path.Add(currentVertexId);
                        graph.VList[currentVertexId].Visited = true;
                    }

                    int[] pathVertices = path.ToArray();
                    Point[] PointList = new Point[path.Count];
                    int q = 0;
                    foreach (int vertex in path)
                    {
                        PointList[q++] = new Point(graph.VList[vertex].XLoc, graph.VList[vertex].YLoc);
                    }
                    LocalModifications.PolygonalChainSimplification(PointList, 0, PointList.Length - 1, 1000);

                    //Modify graph according to the simplified path
                    for (int currentPoint = 0; currentPoint < PointList.Length - 1;)
                    {
                        int nextPoint = currentPoint + 1;
                        for (; nextPoint < PointList.Length; nextPoint++)
                        {
                            if (PointList[nextPoint].X == -1)
                            {
                                //jyoti: RemoveEdge was added to make things fast 
                                graph.RemoveEdge(pathVertices[nextPoint], pathVertices[nextPoint - 1]);
                                continue;
                            }
                            //jyoti: RemoveEdge was added to make things fast 
                            graph.RemoveEdge(pathVertices[nextPoint], pathVertices[nextPoint - 1]);
                            break;
                        }
                        if (nextPoint < PointList.Length)
                        {
                            //////graph.AddEdge(pathVertices[nextPoint], pathVertices[currentPoint]);
                            int left = currentPoint;
                            int right = nextPoint;
                            while (graph.noCrossings(pathVertices, pathVertices[left], pathVertices[right]) == false &&
                                   left < right)
                            {
                                left++;
                                right--;
                            }
                            if (left < right)
                            {
                                //connect each path vertrex in between to this path
                                for (int index = left + 1; index < right; index++)
                                {
                                    Vertex intermediateVertex = graph.VList[pathVertices[index]];
                                    Vertex leftVertex = graph.VList[pathVertices[left]];
                                    Vertex rightVertex = graph.VList[pathVertices[right]];
                                    Point p = PointToSegmentDistance.getClosestPoint
                                        (leftVertex, rightVertex, intermediateVertex);
                                    intermediateVertex.PreciseX = p.X;
                                    intermediateVertex.PreciseY = p.Y;

                                    intermediateVertex.LeftX = leftVertex.XLoc;
                                    intermediateVertex.LeftY = leftVertex.YLoc;
                                    intermediateVertex.RightX = rightVertex.XLoc;
                                    intermediateVertex.RightY = rightVertex.YLoc;
                                }
                            }
                            currentPoint = nextPoint;
                        }
                    }
                }
            }
        }

        private void SetControlVariables()
        {
            //control the density at each label
            if (this._mainGeometryGraph.Edges.Count >= 15000) {
                this._lgLayoutSettings.MaxNumberOfNodesPerTile = 20;
            } else if (this._mainGeometryGraph.Edges.Count >= 10000) {
                this._lgLayoutSettings.MaxNumberOfNodesPerTile = 30;
            } else {
                this._lgLayoutSettings.MaxNumberOfNodesPerTile = 40;
            }

            if (this._mainGeometryGraph.Nodes.Count >= 1000 && this._mainGeometryGraph.Edges.Count <= 15000) {
                this._lgLayoutSettings.MaxNumberOfNodesPerTile = 40;
            }
            //delta = 1 (higher than 1) will give exact (fast approximate) flow 
            //control speed and approximation
            this._lgLayoutSettings.delta = (this._lgLayoutSettings.MaxNumberOfNodesPerTile / 8) + 1;

        }
             
        public void RunForMsaglFiles(string tileDirectory)
        {
            this._lgLayoutSettings.hugeGraph = true; //PromptUserforGraphSize();
            this._lgLayoutSettings.flow = false;//true;// PromptUserforFlow();

            //set control variables
            this.SetControlVariables();

            Dictionary<Node, int> nodeToId;
            var g = this.TryCompetitionMeshApproach(out nodeToId, tileDirectory);

            var stopwatch = new Stopwatch();
            Tiling[] graphs = this.calculateGraphsForEachZoomLevel(g, nodeToId, this._lgLayoutSettings.hugeGraph);
            
            
            stopwatch.Start();
            this.DrawAtEachLevelQuotaBounded(graphs, nodeToId);
            stopwatch.Stop();
        }

 
        public void DrawAtEachLevelQuotaBounded(Tiling[] g, Dictionary<Node, int> nodeToId)
        {

            //until all edges are added create a layer and add vertices one after another
            int layer = 0;
            //int plottedNodeCount = 0;
            List<Node> nodes = new List<Node>();
            this._lgData.Levels.Clear();
             
            foreach (var node in this._mainGeometryGraph.Nodes) {
                this._lgData.GeometryNodesToLgNodeInfos[node].ZoomLevel = this.maxdepth;
            }

            //foreach (var node in _mainGeometryGraph.Nodes)
            //    _lgData.GeometryNodesToLgNodeInfos[node].ZoomLevel = Math.Log(_lgData.GeometryNodesToLgNodeInfos[node].ZoomLevel, 2);


            for (int i = 0; i < g.Length; i++)
            {

                
                var level = this.CreateLevel(layer);
                

                var count = 0;
                //set the zoomlevel of the nodes
                for (int j = 0; j < g[i].N; j++)
                {
                    if (g[i].DegList[j] > 0)
                    //if (_lgData.GeometryNodesToLgNodeInfos[idToNode[j]].ZoomLevel<= layer)
                    {
                        //count++
                        Node nd = this.idToNode[g[i].VList[j].Id];
                        if (this._lgData.GeometryNodesToLgNodeInfos[nd].ZoomLevel == this.maxdepth)
                        {
                            count++;
                            this._lgData.GeometryNodesToLgNodeInfos[nd].ZoomLevel = layer;
                            this._lgLayoutSettings.GeometryNodesToLgNodeInfos[nd].ZoomLevel = layer;                            
                        }
                        else {
                            count++;
                        }
                    }
                    //else  //jyoti - added this in case all top level nodes are of degree 0 - bipartite graph
                      //  if (g[i].VList[j].ZoomLevel <= i+1) count++;
                }


                this._lgData.LevelNodeCounts[layer] = count;
               
                
                //add the edges to the current level for this graph
                Dictionary<Node, int> nodeId = new Dictionary<Node, int>();//-remove later - not needed

                Set<Rail> railsOfEdge = new Set<Rail>();
                foreach (Edge edge in this._mainGeometryGraph.Edges)
                {
                    //if (!level._railsOfEdges.ContainsKey(edge))
                    {
                        railsOfEdge = this.MsaglAddRailsOfEdgeQuotaBounded(level, g[i], edge, nodeId);
                        if (railsOfEdge.Count > 0)
                        {
                            level._railsOfEdges[edge] = railsOfEdge;
                        }
                    }
                }

                layer++;
            }

            this._lgLayoutSettings.maximumNumOfLayers = layer;
            
        }
  
         
        public void DrawAtEachLevelQuotaSatisfied(Tiling[] g, Dictionary<Node, int> nodeToId)
        {
       
            //until all edges are added create a layer and add vertices one after another
            int layer = 0;
            int plottedNodeCount = 0;
            List<Node> nodes = new List<Node>();
            this._lgData.Levels.Clear();
            var level = this.CreateLevel(layer);

            foreach (var node in this._mainGeometryGraph.Nodes) {
                this._lgData.GeometryNodesToLgNodeInfos[node].ZoomLevel = 100;
            }

            foreach (LgNodeInfo node in this._lgData.SortedLgNodeInfos)
            {
                this._lgData.SortedLgNodeInfos[plottedNodeCount].ZoomLevel = layer;
                this._lgData.GeometryNodesToLgNodeInfos[node.GeometryNode].ZoomLevel = layer;
                
                nodes.Add(node.GeometryNode);
                

                int graphLayer = layer;
                if (layer >= g.Length) {
                    graphLayer = g.Length - 1;
                }

                double nodeOnlyZoomLevel = Math.Log(g[graphLayer].VList[nodeToId[node.GeometryNode]].ZoomLevel,2);


                while (true)
                {
                    if (nodeOnlyZoomLevel <= layer)
                    {
                        //try to insert the node
                        if (this.MsaglNodePlottedandQuotaSatisfied(g[graphLayer], level, graphLayer,
                            plottedNodeCount,nodes, nodeToId))
                        {
                            //if the node is successfully inserted, break the loop to proceed with a new node
                            break;
                        }
                    }

                    layer++;
                    level = this.CreateLevel(layer);
                    this._lgData.SortedLgNodeInfos[plottedNodeCount].ZoomLevel = layer;
                    this._lgData.GeometryNodesToLgNodeInfos[node.GeometryNode].ZoomLevel = layer;
                    this._lgLayoutSettings.GeometryNodesToLgNodeInfos[node.GeometryNode].ZoomLevel = layer;
                    node.ZoomLevel = layer;
                    
                    if (layer >= g.Length) {
                        graphLayer = g.Length - 1;
                    } else {
                        graphLayer = layer;
                    }

                    this._lgData.LevelNodeCounts[layer] = plottedNodeCount;
                }
                plottedNodeCount++;
                this._lgData.LevelNodeCounts[layer] = plottedNodeCount;
            }

            this._lgLayoutSettings.maximumNumOfLayers = layer;           
             
        }

        private Dictionary<int, List<int>> tileNodes = new Dictionary<int, List<int>>();
        //(7,5460,21845) (5,341,1365) // of course. makes perfect sense. Now I understand everything about those magic numbers.
        private int maxdepth = 7;
        private int maxtiles = 21845;
        private int immediatemaxtiles = 5461;
        private int[] tileNodeCount = new int[21845];
        private int[] tileEdgeFlow = new int[21845];
        private int[] tileDepth = new int[21845];
        private Rectangle[] tiles = new Rectangle[21845];


        private void buildTiles( double l, double t,  double r, double b, int depth, int pid, int kid)
        {
            if (depth > this.maxdepth) {
                return;
            }

            int index = 4*pid;
            this.tiles[pid] = new Rectangle(l,t,r,b);
            this.tileDepth[pid] = depth;
            
            double centerx = l + (r - l) / 2;
            double centery = t +  (b - t) / 2;
            this.buildTiles(l,          t,       centerx,    centery    , depth + 1, index+1 , 1);
            this.buildTiles(centerx,    t,             r,    centery, depth + 1, index+2, 2);
            this.buildTiles(l,      centery,     centerx,          b, depth + 1,index+3 , 3);
            this.buildTiles(centerx, centery,       r,              b, depth + 1,index+4, 4);

        }

        private bool computeEdgeFlow(int root, int alreadyVisible, int[,] costTree, Dictionary<IntPair, List<int>> resultTree)
        {
            if (this.immediatemaxtiles <= root && root <= this.maxtiles) {
                return true;
            }

            IntPair p = new IntPair(root,alreadyVisible);            
            int[] result = resultTree[p].ToArray();
            if (result.Length == 0) //no feasible solution
{
                return false;
            }

            this.tileEdgeFlow[4 * root + 1] = result[1];
            this.tileEdgeFlow[4 * root + 2] = result[2];
            this.tileEdgeFlow[4 * root + 3] = result[3];
            this.tileEdgeFlow[4 * root + 4] = result[4];

            bool a = this.computeEdgeFlow(4 * root + 1, result[1], costTree, resultTree);
            bool b = this.computeEdgeFlow(4 * root + 2, result[2], costTree, resultTree);
            bool c = this.computeEdgeFlow(4 * root + 3, result[3], costTree, resultTree);
            bool d = this.computeEdgeFlow(4 * root + 4, result[4], costTree, resultTree);
            return a & b & c & d;
        }

        private int dynamicProgram(int root, int alreadyVisible, int[,] costTree, Dictionary<IntPair, List<int>> resultTree)
        {

            //processing for leafnode
            if (this.immediatemaxtiles <= root && root <= this.maxtiles)
            {
                if (this.tileNodeCount[root] < alreadyVisible) {
                    return 0;//int.MaxValue;
                }

                return (this.tileNodeCount[root] - alreadyVisible) * (this.tileNodeCount[root] - alreadyVisible);
            }

            //process non-leaf nodes 
            int cost;
            int mink1=0, mink2=0, mink3=0, mink4=0;
            int mincost = int.MaxValue;
            int minrootsum = int.MaxValue;
            //I can choose the rootsome anything between current visible and nodequota
            for (int rootsum = alreadyVisible; rootsum <= this._lgLayoutSettings.MaxNumberOfNodesPerTile; rootsum += this._lgLayoutSettings.delta)
            {                
            
                //distribute the rootsum among the kids approximately
                for (int k1 = 0; k1 <= rootsum; k1 += this._lgLayoutSettings.delta)
                {
                    for (int k2 = 0; k2 <= rootsum; k2 += this._lgLayoutSettings.delta)
                    {
                        if (rootsum < (k1 + k2)) {
                            break;
                        }

                        for (int k3 = 0; k3 <= rootsum; k3 += this._lgLayoutSettings.delta)
                        {
                            int k4 = rootsum - (k1 + k2 + k3);
                            if (k4 < 0) {
                                break;
                            }
                            //computation of a distribution ends here

                            int sum = (k1 + k2 + k3 + k4);
                            //total visible in this layer                                                                   
                            if (sum != rootsum) {
                                continue; //distribution is not correct
                            }

                            int newvis = rootsum - alreadyVisible; //newly visible nodes                            

                            bool solutionexists = true;
                            int []c = new int[5];
                            int[] k = new int[5];
                            k[1] = k1;
                            k[2] = k2;
                            k[3] = k3;
                            k[4] = k4;
                            for (int i = 1; i <= 4; i++)
                            {
                                c[i] = costTree[4*root + i, k[i]];
                                if (c[i] == -1)
                                {
                                    c[i] = this.dynamicProgram(4*root + i, k[i], costTree, resultTree);
                                    costTree[4*root + i, k[i]] = c[i];
                                }
                                if (c[i] == int.MaxValue) {
                                    solutionexists = false; //no solution 
                                }
                            }
                            if (solutionexists == false) {
                                continue;
                            }

                            cost = (int) (1+Math.Log(root+1,4)) * newvis*newvis; //cost for newly visible nodes
                            if (root == 0) {
                                cost = 0;
                            }

                            cost += (c[1] + c[2] + c[3] + c[4]);
                            if (mincost > cost)
                            {
                                mincost = cost;
                                minrootsum = rootsum;
                                mink1 = k1; mink2 = k2; mink3 = k3; mink4 = k4;
                            }
                        }                        
                    }
                }                
            }

            costTree[root, alreadyVisible] = mincost;           
            var p = new IntPair(root, alreadyVisible);
            if (!resultTree.ContainsKey(p)) {
                resultTree[p] = new List<int>();
            }

            resultTree[p] = new List<int>() { minrootsum, mink1, mink2, mink3, mink4 };
            return mincost;
        }

        private void distributeNodes(int root, int[,] costTree, Dictionary<IntPair, List<int>> resultTree, Dictionary<int, Node> idToNode)
        {
            //processing for a leafnode
            if (this.immediatemaxtiles <= root && root <= this.maxtiles) {
                return;
            }

            this.distributeNodes(4 * root + 1, costTree, resultTree, idToNode);
            this.distributeNodes(4 * root + 2, costTree, resultTree, idToNode);
            this.distributeNodes(4 * root + 3, costTree, resultTree, idToNode);
            this.distributeNodes(4 * root + 4, costTree, resultTree, idToNode);


            List<int> L = new List<int>();
            List<double> R = new List<double>();
            foreach (var x in this.tileNodes[4 * root + 1])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 2])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 3])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 4])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }

            int[] LA = L.ToArray();
            double[] RA = R.ToArray();
            Array.Sort(RA, LA);
            this.tileNodes[root] = new List<int>();

            //assign values of ks
            int k1 = this.tileEdgeFlow[4 * root + 1];
            int k2 = this.tileEdgeFlow[4 * root + 2];
            int k3 = this.tileEdgeFlow[4 * root + 3];
            int k4 = this.tileEdgeFlow[4 * root + 4];

            //if (k1 + k2 + k3 + k4 == 0) return;

            //shift nodes into layers
            for (int i = 0; i < LA.Length; i++)
            {
                if (RA[i] <= Math.Pow(2, this.tileDepth[root]))
                {
                    this.tileNodes[root].Add(LA[i]);
                    if (this.tileNodes[4*root + 1].Remove(LA[i])) {
                        k1--;
                    }

                    if (this.tileNodes[4*root + 2].Remove(LA[i])) {
                        k2--;
                    }

                    if (this.tileNodes[4*root + 3].Remove(LA[i])) {
                        k3--;
                    }

                    if (this.tileNodes[4*root + 4].Remove(LA[i])) {
                        k4--;
                    }

                    continue;

                }
            }
            k1 *= 1+(int)Math.Log(root, 4);
            k2 *= 1 + (int)Math.Log(root, 4);
            k3 *= 1 + (int)Math.Log(root, 4);
            k4 *= 1 + (int)Math.Log(root, 4);
            for (int i = 0; i < LA.Length; i++)
            {
                if (this.tileNodes[4 * root + 1].Contains(LA[i]) && k1 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 1].Remove(LA[i]);
                    k1--;
                }
                else if (this.tileNodes[4 * root + 2].Contains(LA[i]) && k2 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 2].Remove(LA[i]);
                    k2--;
                }
                else if (this.tileNodes[4 * root + 3].Contains(LA[i]) && k3 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 3].Remove(LA[i]);
                    k3--;
                }
                else if (this.tileNodes[4 * root + 4].Contains(LA[i]) && k4 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 4].Remove(LA[i]);
                    k4--;
                }
            }
        }

        private void distributeNodesOld(int root, int[,] costTree, Dictionary<IntPair, List<int>> resultTree, Dictionary<int, Node> idToNode)
        {
            //processing for a leafnode
            if (this.immediatemaxtiles <= root && root <= this.maxtiles) {
                return;
            }

            this.distributeNodesOld(4 * root + 1, costTree, resultTree, idToNode);
            this.distributeNodesOld(4 * root + 2, costTree, resultTree, idToNode);
            this.distributeNodesOld(4 * root + 3, costTree, resultTree, idToNode);
            this.distributeNodesOld(4 * root + 4, costTree, resultTree, idToNode);
            

            List<int> L = new List<int>();
            List<double> R = new List<double>();
            foreach (var x in this.tileNodes[4*root + 1])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 2])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 3])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }
            foreach (var x in this.tileNodes[4 * root + 4])
            {
                L.Add(x);
                R.Add(this._lgData.GeometryNodesToLgNodeInfos[idToNode[x]].ZoomLevel);
            }

            int []LA =  L.ToArray();
            double [] RA = R.ToArray();
            Array.Sort(RA,LA);
            this.tileNodes[root] = new List<int>();

            //assign values of ks
            int k1= this.tileEdgeFlow[4 * root + 1];
            int k2 = this.tileEdgeFlow[4 * root + 2];
            int k3 = this.tileEdgeFlow[4 * root + 3];
            int k4 = this.tileEdgeFlow[4 * root + 4];

            if(k1+k2+k3+k4 == 0) {
                return;
            }

            //shift nodes into layers
            for (int i = 0; i < LA.Length; i++)
            {
                if (this.tileNodes[4*root + 1].Contains(LA[i]) && k1 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4*root + 1].Remove(LA[i]);
                    k1--;
                }
                else if (this.tileNodes[4 * root + 2].Contains(LA[i]) && k2 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 2].Remove(LA[i]);
                    k2--;
                }
                else if (this.tileNodes[4 * root + 3].Contains(LA[i]) && k3 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 3].Remove(LA[i]);
                    k3--;
                }
                else if (this.tileNodes[4 * root + 4].Contains(LA[i]) && k4 > 0)
                {
                    this.tileNodes[root].Add(LA[i]);
                    this.tileNodes[4 * root + 4].Remove(LA[i]);
                    k4--;
                }
            }
        }
        private Tiling TryCompetitionMeshApproach(out Dictionary<Node, int> nodeToId, string tileDirectory)
        {
            Boolean loaded = this.LoadNodeLocationsFromFile(tileDirectory);
            this._mainGeometryGraph.UpdateBoundingBox();
            this._lgLayoutSettings.lgGeometryGraph = this._mainGeometryGraph;

            //create a set of nodes and empty edges 
            this.g = new Tiling(this._mainGeometryGraph.Nodes.Count, true);

            this.idToNode = new Dictionary<int, Node>();
            nodeToId = new Dictionary<Node, int>();



            var stopwatch = new Stopwatch();

            stopwatch.Start();
            this.CreateConnectedGraphs();
            this.FillGeometryNodeToLgInfosTables();


            LevelCalculator.RankGraph(this._lgData, this._mainGeometryGraph);
            if (!loaded) {
                this.LayoutTheWholeGraph();
            }

            int maxY;
            var maxX = this.CreateNodePositions(this.g, nodeToId, this.idToNode, out maxY, tileDirectory);

            if (this._lgLayoutSettings.flow) {
                this.ComputeZoomLevelviaFlow(nodeToId, maxX, maxY);
            }

            stopwatch.Stop();

            stopwatch.Start();
            MeshCreator.FastCompetitionMesh(this.g, this.idToNode, maxX, maxY, this.locationtoNode); 
            stopwatch.Stop();

            stopwatch.Start();
            //Create Detour //less than a minute for 1500 vertices and 5000 edges            
            this.g.MsaglDetour(this.idToNode, true); //true : don't create detour 

            this.g.CreateNodeTreeEdgeTree();
            stopwatch.Stop();



            stopwatch.Start();
            this.g = this.ComputeEdgeRoutes(this.g, nodeToId);

            stopwatch.Stop();

            //Remove Deg 2 Nodes when possible //less than a minute for 1500 vertices and 5000 edges
            stopwatch.Start();
            this.g.MsaglRemoveDeg2(this.idToNode);
            stopwatch.Stop();

            stopwatch.Start();
            PlanarGraphUtilities.TransformToGeometricPlanarGraph(this.g);
            stopwatch.Stop();

            stopwatch.Start();
            PlanarGraphUtilities.RemoveLongEdgesFromThinFaces(this.g);
            stopwatch.Stop();


            //Move the points towards median
            stopwatch.Start();
            LocalModifications.MsaglMoveToMedian(this.g, this.idToNode, this._lgLayoutSettings);
            stopwatch.Stop();

            LocalModifications.MsaglShortcutShortEdges(this.g, this.idToNode, this._lgLayoutSettings);

            this.g.MsaglRemoveDeg2(this.idToNode);

            LocalModifications.MsaglMoveToMedian(this.g, this.idToNode, this._lgLayoutSettings);

            return this.g;
        }

        private void ComputeZoomLevelviaFlow(Dictionary<Node, int> nodeToId, int maxX, int maxY)
        {

            this.buildTiles(0, 0, maxX, maxY, 0, 0, 0);

            int count = 0;
            RTree<int,Point> TreeOfNodes = new RTree<int,Point>();
            for (int index = 0; index < this.g.N; index++)
            {
                TreeOfNodes.Add(new Rectangle(new Point(this.g.VList[index].XLoc, this.g.VList[index].YLoc)), index);
            }
            //nodecount on tiles            
            for (int i = this.immediatemaxtiles; i < this.maxtiles; i++)
            {
                if (this.tileDepth[i] < this.maxdepth) {
                    continue;
                }

                this.tileNodes.Add(i, new List<int>());
                int[] candidateList = TreeOfNodes.GetAllIntersecting(this.tiles[i]);
                for (int index = 0; index < candidateList.Length; index++)
                {
                    this.tileNodes[i].Add(candidateList[index]);
                }
                count += candidateList.Length;
                this.tileNodeCount[i] = candidateList.Length;
            }

            int quota = this._lgLayoutSettings.MaxNumberOfNodesPerTile;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new NotImplementedException();
#else
            int[,] costTree = new int[this.maxtiles, quota + 1];
            Dictionary<IntPair, List<int>> resultTree = new Dictionary<IntPair, List<int>>();
            for (int i = 0; i < this.maxtiles; i++) {
                for (int j = 0; j <= quota; j++) {
                    costTree[i, j] = -1;
                }
            }

            this.dynamicProgram(0, Math.Min(this.g.N, quota), costTree, resultTree);
            bool flowfound = this.computeEdgeFlow(0, Math.Min(this.g.N, quota), costTree, resultTree);
            this.distributeNodes(0, costTree, resultTree, this.idToNode);


            this.g.maxTheoreticalZoomLevel = 0;
            for (int i = 0; i < this.maxtiles; i++)
            {
                foreach (var x in this.tileNodes[i])
                {
                    this.g.VList[x].ZoomLevel = (int) Math.Pow(2, this.tileDepth[i]);
                    this._lgData.GeometryNodesToLgNodeInfos[this.idToNode[x]].ZoomLevel = this.g.VList[x].ZoomLevel;
                    if (this.g.maxTheoreticalZoomLevel < this.g.VList[x].ZoomLevel) {
                        this.g.maxTheoreticalZoomLevel = this.g.VList[x].ZoomLevel;
                    }
                }
            }
            LevelCalculator.SetEdgesOnLevels(this._lgData, this._mainGeometryGraph, this._lgLayoutSettings);
            this._lgData.Levels.Clear();
#endif
        }


        public bool loadBipartiteData(string line)
        {
            if (line == null) {
                return false;
            }

            line = line.Replace(".tiles", "");
            try
            {
                System.IO.StreamReader file = new System.IO.StreamReader(line + ".set");
                Dictionary<string, int> nametoset = new Dictionary<string, int>();
                //int count = 0;
                while ((line = file.ReadLine()) != null)
                {
                    string[] words = line.Split(',');
                    nametoset[words[0].Trim()] = int.Parse(words[1]);
                }
                foreach (var n in this._mainGeometryGraph.Nodes)
                {
                    var nodename = n.ToString().Trim();
                    if ( nametoset.ContainsKey(nodename)) {
                        this._lgData.GeometryNodesToLgNodeInfos[n].PartiteSet = nametoset[nodename];
                    }
                }
                file.Close();
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                Console.WriteLine("May not be a Bipartite Graph");
                return false;
            }
        }
        public bool LoadNodeLocationsFromFile(string tileDirectory)
        {
            //OpenFileDialog openFileDialog1 = new OpenFileDialog();
            if (tileDirectory == null) {
                return false;
            }

            string line = tileDirectory.Replace(".tiles", "");


            try
            {
                System.IO.StreamReader file = new System.IO.StreamReader(line + ".loc");
                Dictionary<string, Point> nametopoint = new Dictionary<string, Point>();
                while ((line = file.ReadLine()) != null)
                {
                    string[] words = line.Split(',');
                    //if (count++ > 200) break;
                    Point p = new Point(Double.Parse(words[1]) * 5, Double.Parse(words[2]) * 5);
                    nametopoint[words[0]] = p;
                }
                List<Node> remove = new List<Node>();


                foreach (Node w in this._mainGeometryGraph.Nodes)
                {
                    String s = w.ToString();//.Split('\"')[1];
                    if (!nametopoint.ContainsKey(s))
                    {
                        remove.Add(w);
                    }
                    else {
                        w.Center = nametopoint[s];
                    }
                }
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                Console.WriteLine("exiting LoadNodeLocationsFromFile");
                return false;
            }


        }

        public void RouteByLayers()//int maxNodesPerTile, int maxSegmentsPerTile, double increaseNodeQuota)
        {
            this.FillLevelsWithNodesOnly(int.MaxValue);

            this.InitRailsOfEdgesEmpty();
            //InitNodeLabelWidthToHeightRatios();
            this.AddSkeletonLevels();
            for (int i = 0; i < this._lgData.Levels.Count; i++) {
                this.RemoveOverlapsAndRouteForLayer(int.MaxValue, int.MaxValue, int.MaxValue, i);
            }

            this._lgData.CreateLevelNodeTrees(this.NodeDotWidth(1));
            //LabelingOfOneRun();
#if TEST_MSAGL
            this.TestAllEdgesConsistency();
#endif
        }

        private int CreateNodePositions(Tiling g, Dictionary<Node, int> nodeToId, Dictionary<int, Node> idToNode, out int maxY, string tileDirectory)
        {
            //PointSet ps = new PointSet(_mainGeometryGraph.Nodes.Count);
            //find maxX and maxY
            int maxX = 0;
            maxY = 0;

            //check if negative coordinate
            double minX = 0;
            double minY = 0;
            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                if (node.Center.X < minX) {
                    minX = node.Center.X;
                }

                if (node.Center.Y < minY) {
                    minY = node.Center.Y;
                }
            }

            //shift to positive coordinate
            int nodeIndex = 0;
            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                nodeToId.Add(node, nodeIndex);
                idToNode.Add(nodeIndex, node);
                node.Center = new Point((int)(node.Center.X - minX + 5), (int)(node.Center.Y - minY + 5));
                if (node.Center.X > maxX) {
                    maxX = (int)node.Center.X;
                }

                if (node.Center.Y > maxY) {
                    maxY = (int)node.Center.Y;
                }

                nodeIndex++;
            }

            //bound the graph inside a 1000 by 1000 box
            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                var x = node.Center.X;//(node.Center.X / maxX) * 1000;
                var y = node.Center.Y;//(node.Center.Y / maxY) * 1000;
                Point p = new Point((int)x, (int)y);

                
                x = (int)x;
                y = (int)y;
                while (this.locationtoNode.ContainsKey(p))
                {
                    x++;
                    y++;
                    p = new Point((int)x, (int)y);
                }
                 
                node.Center = p;
                
                if (!this.locationtoNode.ContainsKey(p)) {
                    this.locationtoNode.Add(p, nodeToId[node]);
                }

                g.VList[nodeToId[node]] = new Vertex((int)p.X, (int)p.Y) { Id = nodeToId[node] };
            }
            /*
            //make sure that no two nodes are on the same position
            for (int i = 0; i < g.NumOfnodes;i++ )
            {
                while (g.GetNodeOtherthanThis(i, g.VList[i].XLoc, g.VList[i].YLoc) >= 0)
                {
                    g.VList[i].XLoc++;
                    g.VList[i].YLoc++;
                }
            }
             */

            maxX = 0;
            maxY = 0;
            //scale the node positions to create intermediate gaps
            for (int i = 0; i < g.NumOfnodes; i++)
            {

                g.VList[i].XLoc *= 5;
                g.VList[i].YLoc *= 5;
                idToNode[i].Center = new Point(g.VList[i].XLoc, g.VList[i].YLoc);
                if (g.VList[i].XLoc > maxX) {
                    maxX = g.VList[i].XLoc;
                }

                if (g.VList[i].YLoc > maxY) {
                    maxY = g.VList[i].YLoc;
                }
            }
            this.locationtoNode = new Dictionary<Point, int>();
            for (int i = 0; i < g.NumOfnodes; i++)
            {
                Point p = new Point(g.VList[i].XLoc, g.VList[i].YLoc);
                this.locationtoNode.Add(p, i);
            }
            int index = 0;
            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                Point p = new Point(g.VList[index].XLoc, g.VList[index].YLoc);
                g.nodeToLoc.Add(node, p);
                index++;
            }

            this._lgData.SortedLgNodeInfos = new List<LgNodeInfo>();

            //FillGeometryNodeToLgInfosTables();

            //CreateConnectedGraphs();
            //LevelCalculator.RankGraph(_lgData, _mainGeometryGraph);
            LevelCalculator.SetNodeZoomLevelsAndRouteEdgesOnLevels(this._lgData, this._mainGeometryGraph, this._lgLayoutSettings);
            
            bool bipartite = this.loadBipartiteData(tileDirectory);


            if (bipartite)
            {

                List<LgNodeInfo> listOfinfo = new List<LgNodeInfo>();
                foreach (var node in this._lgData.SortedLgNodeInfos)
                {
                    if (node.PartiteSet == 1)
                    {
                        listOfinfo.Add(node);
                    }
                }
                foreach (var node in this._lgData.SortedLgNodeInfos)
                {
                    if (node.PartiteSet == 0)
                    {
                        listOfinfo.Add(node);
                    }
                }
                this._lgData.SortedLgNodeInfos.Clear();
                
                foreach (var node in listOfinfo) {
                    this._lgData.SortedLgNodeInfos.Add(node);
                }
            }




            foreach (var node in this._lgData.SortedLgNodeInfos)
            {
                int l = (int)node.ZoomLevel;
                g.VList[nodeToId[node.GeometryNode]].ZoomLevel = l;
                if (g.maxTheoreticalZoomLevel < l) {
                    g.maxTheoreticalZoomLevel = l;
                }
            }
            this._lgData.Levels.Clear();


            this._mainGeometryGraph.UpdateBoundingBox();
            return maxX;
        }

        private Tiling ComputeEdgeRoutes(Tiling g, Dictionary<Node, int> nodeId)
        {
            Tiling g1 = new Tiling(g.NumOfnodes, true);
            g1.N = g.N;
            g1.isPlanar = g.isPlanar;
            g1.NumOfnodesBeforeDetour = g.NumOfnodesBeforeDetour;
            g1.nodeTree = g.nodeTree;
            g1.maxTheoreticalZoomLevel = g.maxTheoreticalZoomLevel;
            for (int i = 0; i < g.NumOfnodes; i++) {
                g1.VList[i] = new Vertex(g.VList[i].XLoc, g.VList[i].YLoc) { Id = g.VList[i].Id, ZoomLevel = g.VList[i].ZoomLevel };
            }

            DijkstraAlgo dijkstra = new DijkstraAlgo();
            int counter = 0;
            foreach (var edge in this._mainGeometryGraph.Edges)
            {
                counter++;

                dijkstra.MSAGLGreedy(g.VList, g.EList, g.DegList, nodeId[edge.Source],
                   nodeId[edge.Target],
                   g.NumOfnodes);
                foreach (VertexNeighbor vn in dijkstra.Edgelist) {
                    g1.AddEdge(vn.A, g.EList[vn.A, vn.Neighbor].NodeId, g.EList[vn.A, vn.Neighbor].Selected, g.EList[vn.A, vn.Neighbor].Used);
                }
            }
            for (int i = 0; i < g1.NumOfnodes; i++) {
                if (g1.DegList[i] == 0) {
                    g1.VList[i].Invalid = true;
                }
            }

            return g1;
        }




        public Dictionary<SymmetricSegment, Rail> Segs = new Dictionary<SymmetricSegment, Rail>();

        public int[] graphLayer = new int[100];

        public bool MsaglNodePlottedandQuotaSatisfied(Tiling g, LgLevel level, int currentGraphLayer, int nodeToBePlotted, IEnumerable<Node> nodes, Dictionary<Node, int> nodeId)
        {

            this.graphLayer[level.ZoomLevel] = currentGraphLayer;
            Set<Rail> railsOfEdge = new Set<Rail>();

            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                if (!level._railsOfEdges.ContainsKey(edge))
                {
                    railsOfEdge = this.MsaglAddRailsOfEdge(level, g, edge, nodeId);
                    if (railsOfEdge.Count > 0)
                    {
                        level._railsOfEdges[edge] = railsOfEdge;
                    }
                }
            }



            SymmetricSegment s;
            List<SymmetricSegment> newSegments = new List<SymmetricSegment>();
            foreach (var rail in level.RailDictionary.Values)
            {
                s = new SymmetricSegment(rail.Left, rail.Right);

                if (!this.Segs.ContainsKey(s))
                {
                    this.Segs.Add(s, rail);
                    newSegments.Add(s);
                }
            }


            var allInserted = true;


            if (!level.QuotaSatisfied(nodes,
                this._lgLayoutSettings.MaxNumberOfNodesPerTile,
                this._lgLayoutSettings.MaxNumberOfRailsPerTile)) {
                allInserted = false;
            }

            if (!allInserted && level.ZoomLevel - 1 >= 0)
            {

                //foreach (Edge edge in level._railsOfEdges.Keys)
                level._railsOfEdges.Clear();
                level._railDictionary.Clear();
                level.RailTree.Clear();
                foreach (SymmetricSegment sg in newSegments) {
                    this.Segs.Remove(sg);
                }

                foreach (Edge edge in this._lgData.Levels[level.ZoomLevel - 1]._railsOfEdges.Keys)
                {
                    railsOfEdge = this._lgData.Levels[level.ZoomLevel - 1]._railsOfEdges[edge];

                    Set<Rail> rails = new Set<Rail>();
                    Set<Rail> oldRails = new Set<Rail>();

                    foreach (var r in railsOfEdge)
                    {

                        LineSegment ls = new LineSegment(r.A, r.B);
                        var tuple = new SymmetricSegment(r.A, r.B);

                        Rail rail;
                        if (!level._railDictionary.TryGetValue(tuple, out rail))
                        {
                            rail = new Rail(ls, this._lgData.GeometryEdgesToLgEdgeInfos[edge],
                                                level.ZoomLevel);
                            this._lgData.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel = level.ZoomLevel;

                            rail.A = r.targetA;
                            rail.B = r.targetB;
                            rail.initialA = r.initialA;
                            rail.initialB = r.initialB;
                            rail.targetA = r.targetA;
                            rail.targetB = r.targetB;

                            if (!this.RailToEdges.ContainsKey(rail)) {
                                this.RailToEdges[rail] = new List<Edge>();
                            }

                            if (!this.RailToEdges[rail].Contains(edge)) {
                                this.RailToEdges[rail].Add(edge);
                            }

                            level._railDictionary[tuple] = rail;
                            level._railTree.Add(ls.BoundingBox, rail);
                            rails.Insert(rail);

                        }
                        else
                        {
                            if (!this.RailToEdges[rail].Contains(edge)) {
                                this.RailToEdges[rail].Add(edge);
                            }
                        }

                    }
                    level._railsOfEdges[edge] = rails;
                }

                return false;
            }

            return true;
        }

        public bool MsaglNodeSuccessfullyPlotted(Tiling g, LgLevel level, int currentGraphLayer, int nodeToBePlotted, IEnumerable<Node> nodes, Dictionary<Node, int> nodeId)
        {

            this.graphLayer[level.ZoomLevel] = currentGraphLayer;
            Set<Rail> railsOfEdge = new Set<Rail>();

            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                if (!level._railsOfEdges.ContainsKey(edge))
                {
                    railsOfEdge = this.MsaglAddRailsOfEdge(level, g, edge, nodeId);
                    if (railsOfEdge.Count > 0)
                    {
                        level._railsOfEdges[edge] = railsOfEdge;
                    }
                }
            }


            var bbox = this.GetLargestTile();
            GridTraversal grid = new GridTraversal(bbox, level.ZoomLevel);
            GreedyNodeRailLevelCalculator calc = new GreedyNodeRailLevelCalculator(this._lgData.SortedLgNodeInfos);


            calc.MaxAmountRailsPerTile = this._lgLayoutSettings.MaxNumberOfRailsPerTile;


            SymmetricSegment s;
            List<SymmetricSegment> newSegments = new List<SymmetricSegment>();
            foreach (var rail in level.RailDictionary.Values)
            {
                s = new SymmetricSegment(rail.Left, rail.Right);

                if (!this.Segs.ContainsKey(s))
                {
                    this.Segs.Add(s, rail);
                    newSegments.Add(s);
                }
            }

            Boolean allInserted = true;
            foreach (var seg in newSegments) {
                if (calc.IfCanInsertLooseSegmentUpdateTiles(seg, grid) == false)
                {
                    allInserted = false;
                    //break;
                }
            }

            if (!allInserted && level.ZoomLevel - 1 >= 0)
            {

                foreach (Edge edge in level._railsOfEdges.Keys) {
                    level._railsOfEdges[edge].Clear();
                }

                foreach (SymmetricSegment sg in newSegments) {
                    this.Segs.Remove(sg);
                }

                foreach (Edge edge in this._lgData.Levels[level.ZoomLevel - 1]._railsOfEdges.Keys)
                {
                    railsOfEdge = this._lgData.Levels[level.ZoomLevel - 1]._railsOfEdges[edge];
                    level._railsOfEdges[edge] = railsOfEdge;
                }

                return false;
            }

            return true;//level.QuotaSatisfied(nodes, this._lgLayoutSettings.MaxNumberOfNodesPerTile, this._lgLayoutSettings.MaxNumberOfRailsPerTile);
        }


        public Dictionary<Rail, List<Edge>> RailToEdges = new Dictionary<Rail, List<Edge>>();

        private Set<Rail> MsaglAddRailsOfEdge(LgLevel level, Tiling g, Edge edge, Dictionary<Node, int> nodeId)
        {
            if (this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel > level.ZoomLevel ||
             this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel > level.ZoomLevel) {
                return new Set<Rail>();
            }

            if (!g.pathList.ContainsKey(edge)) {
                return new Set<Rail>();
            }

            this._lgData.GeometryEdgesToLgEdgeInfos[edge] = new LgEdgeInfo(edge)
            {
                Rank = Math.Min(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel),
                ZoomLevel =
                    Math.Max(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel)
            };


            var railsOfEdge = new Set<Rail>();


            int[] pathVertices = g.pathList[edge].ToArray();
            for (int index = 0; index < pathVertices.Length - 1; index++)
            {
                var a = new Point(g.VList[pathVertices[index]].PreciseX, g.VList[pathVertices[index]].PreciseY);
                var b = new Point(g.VList[pathVertices[index + 1]].PreciseX, g.VList[pathVertices[index + 1]].PreciseY);
                var initial_a = new Point(g.VList[pathVertices[index]].XLoc, g.VList[pathVertices[index]].YLoc);
                var initial_b = new Point(g.VList[pathVertices[index + 1]].XLoc, g.VList[pathVertices[index + 1]].YLoc);
                var target_a = new Point(g.VList[pathVertices[index]].TargetX, g.VList[pathVertices[index]].TargetY);
                var target_b = new Point(g.VList[pathVertices[index + 1]].TargetX, g.VList[pathVertices[index + 1]].TargetY);

                var left = new Point(0, 0);
                var right = new Point(0, 0);

                if (g.VList[pathVertices[index]].LeftX != 0 && g.VList[pathVertices[index]].RightX != 0)
                {
                    left = new Point(g.VList[pathVertices[index]].LeftX, g.VList[pathVertices[index]].LeftY);
                    right = new Point(g.VList[pathVertices[index]].RightX, g.VList[pathVertices[index]].RightY);
                }
                else if (g.VList[pathVertices[index + 1]].LeftX != 0 && g.VList[pathVertices[index + 1]].RightX != 0)
                {
                    left = new Point(g.VList[pathVertices[index + 1]].LeftX, g.VList[pathVertices[index + 1]].LeftY);
                    right = new Point(g.VList[pathVertices[index + 1]].RightX, g.VList[pathVertices[index + 1]].RightY);
                }


                var tuple = new SymmetricSegment(a, b);
                Rail rail;
                if (!level._railDictionary.TryGetValue(tuple, out rail))
                {
                    var ls = new LineSegment(a, b);
                    //CubicBezierSegment cb = new CubicBezierSegment(a, a, a, a);
                    rail = new Rail(ls, this._lgData.GeometryEdgesToLgEdgeInfos[edge], level.ZoomLevel);
                        //(int)_lgData.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel);

                    if (!this.RailToEdges.ContainsKey(rail)) {
                        this.RailToEdges[rail] = new List<Edge>();
                    }

                    if (!this.RailToEdges[rail].Contains(edge)) {
                        this.RailToEdges[rail].Add(edge);
                    }

                    rail.A = a;
                    rail.B = b;
                    rail.initialA = initial_a;
                    rail.initialB = initial_b;
                    rail.targetA = target_a;
                    rail.targetB = target_b;

                    if (left.X != 0 && left.Y != 0)
                    { rail.Left = left; rail.Right = right; }
                    else { rail.Left = a; rail.Right = b; }

                    level._railDictionary[tuple] = rail;
                    level._railTree.Add(ls.BoundingBox, rail);
                }
                else
                {
                    rail.ZoomLevel = Math.Max(rail.ZoomLevel, level.ZoomLevel);
                    //rail.ZoomLevel = level.ZoomLevel;
                    if (!this.RailToEdges[rail].Contains(edge)) {
                        this.RailToEdges[rail].Add(edge);
                    }
                }
                railsOfEdge.Insert(rail);
            }

            return railsOfEdge;
        }




        private Set<Rail> MsaglAddRailsOfEdgeQuotaBounded(LgLevel level, Tiling g, Edge edge, Dictionary<Node, int> nodeId)
        {
            if (this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel > level.ZoomLevel ||
             this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel > level.ZoomLevel) {
                return new Set<Rail>();
            }

            if (!g.pathList.ContainsKey(edge)) {
                return new Set<Rail>();
            }

            this._lgData.GeometryEdgesToLgEdgeInfos[edge] = new LgEdgeInfo(edge)
            {
                Rank = Math.Max(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel),
                ZoomLevel =
                    Math.Max(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel)
            };


            var railsOfEdge = new Set<Rail>();


            int[] pathVertices = g.pathList[edge].ToArray();
            for (int index = 0; index < pathVertices.Length - 1; index++)
            {
                var a = new Point(g.VList[pathVertices[index]].PreciseX, g.VList[pathVertices[index]].PreciseY);
                var b = new Point(g.VList[pathVertices[index + 1]].PreciseX, g.VList[pathVertices[index + 1]].PreciseY);
                var initial_a = new Point(g.VList[pathVertices[index]].XLoc, g.VList[pathVertices[index]].YLoc);
                var initial_b = new Point(g.VList[pathVertices[index + 1]].XLoc, g.VList[pathVertices[index + 1]].YLoc);
                var target_a = new Point(g.VList[pathVertices[index]].TargetX, g.VList[pathVertices[index]].TargetY);
                var target_b = new Point(g.VList[pathVertices[index + 1]].TargetX, g.VList[pathVertices[index + 1]].TargetY);

                var left = new Point(0, 0);
                var right = new Point(0, 0);

                if (g.VList[pathVertices[index]].LeftX != 0 && g.VList[pathVertices[index]].RightX != 0)
                {
                    left = new Point(g.VList[pathVertices[index]].LeftX, g.VList[pathVertices[index]].LeftY);
                    right = new Point(g.VList[pathVertices[index]].RightX, g.VList[pathVertices[index]].RightY);
                }
                else if (g.VList[pathVertices[index + 1]].LeftX != 0 && g.VList[pathVertices[index + 1]].RightX != 0)
                {
                    left = new Point(g.VList[pathVertices[index + 1]].LeftX, g.VList[pathVertices[index + 1]].LeftY);
                    right = new Point(g.VList[pathVertices[index + 1]].RightX, g.VList[pathVertices[index + 1]].RightY);
                }


                var tuple = new SymmetricSegment(a, b);
                Rail rail;
                if (!level._railDictionary.TryGetValue(tuple, out rail))
                {
                    var ls = new LineSegment(a, b);
                    //CubicBezierSegment cb = new CubicBezierSegment(a, a, a, a);
                    rail = new Rail(ls, this._lgData.GeometryEdgesToLgEdgeInfos[edge], level.ZoomLevel);
                    //(int)_lgData.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel);

                    if (!this.RailToEdges.ContainsKey(rail)) {
                        this.RailToEdges[rail] = new List<Edge>();
                    }

                    if (!this.RailToEdges[rail].Contains(edge)) {
                        this.RailToEdges[rail].Add(edge);
                    }

                    rail.A = a;
                    rail.B = b;
                    rail.initialA = initial_a;
                    rail.initialB = initial_b;
                    rail.targetA = target_a;
                    rail.targetB = target_b;

                    if (left.X != 0 && left.Y != 0)
                    { rail.Left = left; rail.Right = right; }
                    else { rail.Left = a; rail.Right = b; }

                    level._railDictionary[tuple] = rail;
                    level._railTree.Add(ls.BoundingBox, rail);

                    //rail.TopRankedEdgeInfoOfTheRail = _lgData.GeometryEdgesToLgEdgeInfos[edge];
                    rail.UpdateTopEdgeInfo(this._lgData.GeometryEdgesToLgEdgeInfos[edge]);
                }
                else
                {
                    rail.ZoomLevel = Math.Max(rail.ZoomLevel, level.ZoomLevel);
                    //rail.ZoomLevel = level.ZoomLevel;
                    if (!this.RailToEdges[rail].Contains(edge)) {
                        this.RailToEdges[rail].Add(edge);
                    }
                }
                railsOfEdge.Insert(rail);
            }

            return railsOfEdge;
        }



        private void FillGeometryNodesToLgNodeInfos()
        {
            int rankIndex = int.MaxValue;
            this._lgData.SortedLgNodeInfos = new List<LgNodeInfo>();
            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                var nodeInfo = new LgNodeInfo(node)
                {
                    Rank = rankIndex--,
                    ZoomLevel = 1000,
                    LabelVisibleFromScale = 1,
                    LabelWidthToHeightRatio = 1,
                    LabelOffset = new Point(1, 1)
                };
                this._lgData.SortedLgNodeInfos.Add(nodeInfo);
                this._lgData.GeometryNodesToLgNodeInfos[node] = nodeInfo;
            }
        }

        /// <summary>
        ///     does the initialization
        /// </summary>
        public void Run(string tileDirectory)
        {
            this.RunForMsaglFiles(tileDirectory);
            this._lgData.CreateLevelNodeTrees(this.NodeDotWidth(1));
            this._railGraph = new RailGraph();
    
#if TEST_GRAPHMAPS
#if TEST_MSAGL
            _mainGeometryGraph.SetDebugIds();
#endif

            CreateConnectedGraphs();
            FillGeometryNodeToLgInfosTables();
            LevelCalculator.RankGraph(_lgData, _mainGeometryGraph);
            LayoutTheWholeGraph();
#if !SHARPKIT
            var timer = new Timer();
            timer.Start();
#endif

            LevelCalculator.SetNodeZoomLevelsAndRouteEdgesOnLevels(_lgData, _mainGeometryGraph, _lgLayoutSettings);
            Debug.Assert(ClusterRanksAreNotLessThanChildrens());
            _railGraph = new RailGraph();
            LayoutAndRouteByLayers(_lgLayoutSettings.MaxNumberOfNodesPerTile, _lgLayoutSettings.MaxNumberOfRailsPerTile,
                _lgLayoutSettings.IncreaseNodeQuota);
#if !SHARPKIT
            timer.Stop();
            Console.WriteLine("levels calculated for {0}", timer.Duration);
            if (_lgLayoutSettings.ExitAfterInit)
                Environment.Exit(0);
#endif
#endif
        }

        private bool NodeSuccessfullyPlotted(PointSet points, Dictionary<Node, int> nodeToIndex, LgLevel level, Tiling grid, int nodeToBePlotted, IEnumerable<Node> nodes)
        {
            DijkstraAlgo dijkstra = new DijkstraAlgo();

            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                var railsOfEdge = this.AddRailsOfEdge(points, nodeToIndex, level, grid, edge, dijkstra);
                level._railsOfEdges[edge] = railsOfEdge;

            }

            return level.QuotaSatisfied(nodes, this._lgLayoutSettings.MaxNumberOfNodesPerTile, this._lgLayoutSettings.MaxNumberOfRailsPerTile);
        }

        private Set<Rail> AddRailsOfEdge(PointSet points, Dictionary<Node, int> nodeToIndex, LgLevel level, Tiling grid, Edge edge,
            DijkstraAlgo dijkstra)
        {
            if (this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel > level.ZoomLevel ||
                this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel > level.ZoomLevel) {
                return new Set<Rail>();
            }

            this._lgData.GeometryEdgesToLgEdgeInfos[edge] = new LgEdgeInfo(edge)
            {
                Rank = Math.Min(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel),
                ZoomLevel =
                    Math.Max(this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel,
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel)
            };

            dijkstra.SelectShortestPath(grid.VList, grid.EList, grid.DegList, points.Pt[nodeToIndex[edge.Source]].GridPoint,
                points.Pt[nodeToIndex[edge.Target]].GridPoint, grid.NumOfnodes);

            var railsOfEdge = new Set<Rail>();
            foreach (VertexNeighbor e in dijkstra.Edgelist)
            {
                var a = new Point(grid.VList[e.A].XLoc, grid.VList[e.A].YLoc);
                var b = new Point(grid.VList[grid.EList[e.A, e.Neighbor].NodeId].XLoc,
                    grid.VList[grid.EList[e.A, e.Neighbor].NodeId].YLoc);
                //_grid.EList[e.A, e.Neighbor].Used++;
                var tuple = new SymmetricSegment(a, b);
                Rail rail;
                if (!level._railDictionary.TryGetValue(tuple, out rail))
                {
                    var ls = new LineSegment(a, b);
                    rail = new Rail(ls, this._lgData.GeometryEdgesToLgEdgeInfos[edge],
                        (int)this._lgData.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel);
                    level._railDictionary[tuple] = rail;
                    level._railTree.Add(ls.BoundingBox, rail);
                }
                else {
                    rail.ZoomLevel = Math.Max(rail.ZoomLevel, level.ZoomLevel);
                }

                railsOfEdge.Insert(rail);
            }
            return railsOfEdge;
        }

        private LgLevel CreateLevel(int layer)
        {
            //FillGeometryNodeToLgInfosTables();
            LgLevel level = new LgLevel(layer, this._mainGeometryGraph);

            int levelNodeCount = this._mainGeometryGraph.Nodes.Count;
            if (this._lgData.LevelNodeCounts == null)
            {
                this._lgData.LevelNodeCounts = new List<int>();
            }
            this._lgData.LevelNodeCounts.Add(levelNodeCount);
            this._lgData.Levels.Add(level);
            return level;
        }

        private bool ClusterRanksAreNotLessThanChildrens()
        {
            return
                this._mainGeometryGraph.RootCluster.AllClustersDepthFirstExcludingSelf().
                    All(cluster => cluster.Clusters.Concat(cluster.Nodes).Any(n =>
                        this._lgData.GeometryNodesToLgNodeInfos[cluster].Rank >= this._lgData.GeometryNodesToLgNodeInfos[n].Rank));
        }


        //        void TestVisibleTogether() {
        //            for (int i = 0; i < geometryGraph.Nodes.Count - 1; i++) {
        //                var a = GeometryNodesToLgNodes[geometryGraph.Nodes[i]];
        //                for (int j = i + 1; j < geometryGraph.Nodes.Count; j++) {
        //                    var b = GeometryNodesToLgNodes[geometryGraph.Nodes[j]];
        //                    string color = VisibleTogether(a, b) ? "green" : "red";
        //                    var l = new List<DebugCurve>();
        //                    foreach (var n in geometryGraph.Nodes) {
        //                        if(n!=a.GeometryNode && n!=b.GeometryNode)
        //                        l.Add(new DebugCurve(100, 1, "black", n.BoundaryCurve));
        //                        else
        //                            l.Add(new DebugCurve(3, color, n.BoundaryCurve));
        //
        //                    }
        //
        //                    l.Add(new DebugCurve(5,color, a.DominatedRect.Perimeter()));
        //                    l.Add(new DebugCurve(5, color, b.DominatedRect.Perimeter()));
        //                    LayoutAlgorithmSettings.ShowDebugCurves(l.ToArray());
        //                }
        //            }
        //        }


        private void FillGeometryNodeToLgInfosTables()
        {
            foreach (
                Node node in
                    this._mainGeometryGraph.Nodes.Concat(this._mainGeometryGraph.RootCluster.AllClustersWideFirstExcludingSelf()))
            {

                this._lgData.GeometryNodesToLgNodeInfos[node] = new LgNodeInfo(node);
            }
            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                this._lgData.GeometryEdgesToLgEdgeInfos[edge] = new LgEdgeInfo(edge);
            }
        }

        private void LayoutTheWholeGraph()
        {
            if (this._lgLayoutSettings.NeedToLayout)
            {
                if (this._runInParallel) {
                    Parallel.ForEach(this._lgData.ConnectedGeometryGraphs, new ParallelOptions(), this.LayoutAndPadOneComponent);
                } else {
                    foreach (GeometryGraph connectedGraph in this._lgData.ConnectedGeometryGraphs) {
                        this.LayoutAndPadOneComponent(connectedGraph);
                    }
                }

                Rectangle rect = MdsGraphLayout.PackGraphs(this._lgData.ConnectedGeometryGraphs, this._lgLayoutSettings);
                this._mainGeometryGraph.BoundingBox = rect;
            }
            else {
                foreach (GeometryGraph graph in this._lgData.ConnectedGeometryGraphs) {
                    graph.UpdateBoundingBox();
                }
            }
        }

        private void LayoutAndPadOneComponent(GeometryGraph connectedGraph)
        {
            this.LayoutOneComponent(connectedGraph);
            connectedGraph.BoundingBox.Pad(this._lgLayoutSettings.NodeSeparation / 2);
        }

        private void CreateConnectedGraphs()
        {
            Dictionary<Node, int> nodeToIndex;
            List<Node> listOfNodes = this.CreateNodeListForBasicGraph(out nodeToIndex);
            var basicGraph = new BasicGraphOnEdges<SimpleIntEdge>(this.GetSimpleIntEdges(nodeToIndex), listOfNodes.Count);
            IEnumerable<IEnumerable<int>> comps = ConnectedComponentCalculator<SimpleIntEdge>.GetComponents(basicGraph);
            foreach (var comp in comps) {
                this._lgData.AddConnectedGeomGraph(this.GetConnectedSubgraph(comp, listOfNodes));
            }
        }

        private GeometryGraph GetConnectedSubgraph(IEnumerable<int> comp, List<Node> nodeList)
        {
            var edges = new List<Edge>();
            var nodes = new List<Node>();
            var geomGraph = new GeometryGraph();
            foreach (int i in comp)
            {
                Node node = nodeList[i];
                var cluster = node as Cluster;
                if (cluster != null)
                {
                    if (cluster.ClusterParent == this._mainGeometryGraph.RootCluster)
                    {
                        //MainGeometryGraph.RootCluster.RemoveCluster(cluster);
                        geomGraph.RootCluster.AddCluster(cluster);
                    }
                }
                else
                {
                    nodes.Add(node);
                }

                foreach (Edge edge in node.OutEdges.Concat(node.SelfEdges))
                {
                    Debug.Assert(!edges.Contains(edge));
                    edges.Add(edge);
                }
            }
            geomGraph.Edges = new SimpleEdgeCollection(edges);
            geomGraph.Nodes = new SimpleNodeCollection(nodes);
            return geomGraph;
        }

        private List<Node> CreateNodeListForBasicGraph(out Dictionary<Node, int> nodeToIndex)
        {
            var list = new List<Node>();
            nodeToIndex = new Dictionary<Node, int>();

            foreach (Node node in this._mainGeometryGraph.Nodes)
            {
                nodeToIndex[node] = list.Count;
                list.Add(node);
            }
            foreach (Cluster node in this._mainGeometryGraph.RootCluster.AllClustersDepthFirstExcludingSelf())
            {
                nodeToIndex[node] = list.Count;
                list.Add(node);
            }
            return list;
        }

        private List<SimpleIntEdge> GetSimpleIntEdges(Dictionary<Node, int> nodeToIndex)
        {
            var list =
                this._mainGeometryGraph.Edges.Select(
                    edge => new SimpleIntEdge { Source = nodeToIndex[edge.Source], Target = nodeToIndex[edge.Target] })
                    .ToList();

            foreach (Cluster cluster in this._mainGeometryGraph.RootCluster.AllClustersDepthFirstExcludingSelf())
            {
                list.AddRange(
                    cluster.Clusters.Select(
                        child => new SimpleIntEdge { Source = nodeToIndex[cluster], Target = nodeToIndex[child] }));
                list.AddRange(
                    cluster.Nodes.Select(
                        child => new SimpleIntEdge { Source = nodeToIndex[cluster], Target = nodeToIndex[child] }));
            }
            return list;
        }

        private void LayoutOneComponent(GeometryGraph component)
        {
            PrepareGraphForLayout(component);
            if (component.RootCluster.Clusters.Any())
            {
                // todo: do we really use layered layout here?
                var layoutSettings = new SugiyamaLayoutSettings
                {
                    FallbackLayoutSettings =
                        new FastIncrementalLayoutSettings
                        {
                            AvoidOverlaps = true
                        },
                    NodeSeparation = this._lgLayoutSettings.NodeSeparation,
                    LayerSeparation = this._lgLayoutSettings.NodeSeparation,
                    EdgeRoutingSettings = this._lgLayoutSettings.EdgeRoutingSettings,
                    LayeringOnly = true
                };
                var initialBc = new InitialLayoutByCluster(component, a => layoutSettings);
                initialBc.Run();
            }
            else
            {
                // todo: currently implemented as if there is only one component
                LayoutHelpers.CalculateLayout(component, this.GetMdsLayoutSettings(), this._cancelToken);
                this.RemoveOverlapsForLgLayout(component);
            }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=369 there are no structs in js
            Rectangle box = component.BoundingBox.Clone();
#else
            Rectangle box = component.BoundingBox;
#endif
            box.Pad(this._lgLayoutSettings.NodeSeparation / 2);
            component.BoundingBox = box;
        }

        private void RemoveOverlapsForLgLayout(GeometryGraph component)
        {
            Node[] componentNodes = component.Nodes.ToArray();
            Debug.Assert(this.RankingIsDefined(componentNodes));
            this.SortComponentNodesByRanking(componentNodes);
            List<int> approxLayerCounts = this.GetApproximateLayerCounts(componentNodes);
            Size[] sizes = this.GetApproxSizesForOverlapRemoval(approxLayerCounts, this._lgLayoutSettings.NodeSeparation * 3,
                componentNodes);
            GTreeOverlapRemoval.RemoveOverlapsForLayers(componentNodes, sizes);
        }

        private Size[] GetApproxSizesForOverlapRemoval(List<int> approxLayerCounts, double w, Node[] componentNodes)
        {
            var ret = new Size[componentNodes.Length];
            int layer = 0;

            for (int i = 0; i < componentNodes.Length; i++)
            {
                ret[i] = new Size(w, w);
                if (i == approxLayerCounts[layer])
                {
                    w /= 2;
                    layer++;
                }
            }
            return ret;
        }

        /// <summary>
        ///     trying to approximately evaluate what the number of nodes will be in each layer
        /// </summary>
        /// <param name="componentNodes"></param>
        /// <returns></returns>
        private List<int> GetApproximateLayerCounts(Node[] componentNodes)
        {
            int total = componentNodes.Length;
            int nodesPerTile = this._lgLayoutSettings.MaxNumberOfNodesPerTile;
            int numberOfTiles = 1;
            var ret = new List<int>();
            while (true)
            {
                int nodesOnLayer = Math.Min(nodesPerTile * numberOfTiles, total);
                ret.Add(nodesOnLayer);
                if (nodesOnLayer == total) {
                    break;
                }

                numberOfTiles *= 3; // thinking about those empty tiles!
            }
            return ret;
        }

        private void SortComponentNodesByRanking(Node[] componentNodes)
        {
            var comparer = new RankComparer(this._lgData.GeometryNodesToLgNodeInfos);
            Array.Sort(componentNodes, comparer);
        }

        private bool RankingIsDefined(Node[] componentNodes)
        {
            if (componentNodes.Length == 0) {
                return true;
            }

            return this._lgData.GeometryNodesToLgNodeInfos[componentNodes[0]].Rank > 0;
        }

        private static void PrepareGraphForLayout(GeometryGraph connectedGraph)
        {
            foreach (Cluster cluster in connectedGraph.RootCluster.AllClustersDepthFirst())
            {
                if (cluster.RectangularBoundary == null) {
                    cluster.RectangularBoundary = new RectangularClusterBoundary();
                }
            }

            foreach (Edge edge in connectedGraph.Edges)
            {
                if (edge.SourcePort == null)
                {
                    Edge e = edge;
                    edge.SourcePort = new RelativeFloatingPort(() => e.Source.BoundaryCurve,
                        () => e.Source.Center);
                }
                if (edge.TargetPort == null)
                {
                    Edge e = edge;
                    edge.TargetPort = new RelativeFloatingPort(() => e.Target.BoundaryCurve,
                        () => e.Target.Center);
                }
            }
        }

        private MdsLayoutSettings GetMdsLayoutSettings()
        {
            var settings = new MdsLayoutSettings
            {
                EdgeRoutingSettings = { KeepOriginalSpline = true, EdgeRoutingMode = EdgeRoutingMode.None },
                RemoveOverlaps = false
            };
            return settings;
        }

        private bool IsEdgeVisibleAtCurrentLevel(Edge e)
        {
            LgEdgeInfo ei;
            if (!this.GeometryEdgesToLgEdgeInfos.TryGetValue(e, out ei)) {
                return false;
            }

            double curLevel = this._lgData.Levels[this._lgData.GetLevelIndexByScale(this.CurrentZoomLevel)].ZoomLevel;
            return ei.ZoomLevel <= curLevel;
        }

        private void AddHighlightedEdgesAndNodesToRailGraph()
        {
            this._railGraph.Edges.InsertRange(this._lgData.SelectedEdges);
            this._railGraph.Nodes.InsertRange(this._lgData.SelectedNodeInfos.Select(n => n.GeometryNode));
        }

        public void AddLabelsOfHighlightedNodes(double scale)
        {
            this.SelectedNodeLabels.Clear();

            var selNodesSet = new Set<LgNodeInfo>();
            foreach (var edge in this._lgData.SelectedEdges)
            {
                var ni = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                if (this._visibleRectangle.Intersects(this.GetNodeDotRect(ni, scale)))
                {
                    selNodesSet.Insert(ni);
                }
                ni = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                if (this._visibleRectangle.Intersects(this.GetNodeDotRect(ni, scale)))
                {
                    selNodesSet.Insert(ni);
                }
            }
            selNodesSet.InsertRange(this._lgData.SelectedNodeInfos.Where(ni => this._visibleRectangle.Intersects(this.GetNodeDotRect(ni, scale))));

            if (!selNodesSet.Any())
            {
                return;
            }
            foreach (var n in selNodesSet)
            {
                
            }
            var selNodes = selNodesSet.ToList();
            selNodes = selNodes.OrderByDescending(ni => ni.Rank).ToList();

            this.InsertCandidateLabelsGreedily(selNodes, scale);


        }

        public bool forthefirsttime = true;

        private void AddVisibleRailsAndNodes()
        {
            this._railGraph.Rails.Clear();
            var level = this._lgData.GetCurrentLevelByScale(this.CurrentZoomLevel);
            this._railGraph.Rails.InsertRange(level.GetRailsIntersectingRect(this._visibleRectangle));

            //jyoti: only show the top level nodes for the first time   
            if (this.forthefirsttime) {
                this._railGraph.Nodes.InsertRange(level.GetNodesIntersectingRectLabelzero(this._visibleRectangle, 1));
            } else {
                this._railGraph.Nodes.InsertRange(level.GetNodesIntersectingRectLabelzero(this._visibleRectangle, Math.Pow(2, level.ZoomLevel)));
            }

            this._railGraph.Edges.Clear();
            this._railGraph.Edges.InsertRange(
                this._railGraph.Rails.Select(r => r.TopRankedEdgeInfoOfTheRail.Edge).Where(this.IsEdgeVisibleAtCurrentLevel));
            this.AddHighlightedEdgesAndNodesToRailGraph();
        }

        internal static GeometryGraph CreateClusteredSubgraphFromFlatGraph(GeometryGraph subgraph,
            GeometryGraph mainGeometryGraph)
        {
            if (mainGeometryGraph.RootCluster.Clusters.Any() == false) {
                return subgraph;
            }

            var ret = new GeometryGraph();
            Dictionary<Node, Node> originalNodesToNewNodes = MapSubgraphNodesToNewNodesForRouting(subgraph);
            ReplicateClusterStructure(subgraph, originalNodesToNewNodes);
            AddNewNodeAndClustersToTheNewGraph(originalNodesToNewNodes, ret);


            foreach (Edge edge in subgraph.Edges)
            {
                Node ns = originalNodesToNewNodes[edge.Source];
                Node nt = originalNodesToNewNodes[edge.Target];
                ret.Edges.Add(new Edge(ns, nt)
                {
                    EdgeGeometry = edge.EdgeGeometry,
                    SourcePort = null,
                    TargetPort = null
                });
            }

            foreach (var kv in originalNodesToNewNodes)
            {
                Node newNode = kv.Value;
                var cluster = newNode as Cluster;
                if (cluster != null)
                {
                    Node oldNode = kv.Key;
                    if (oldNode.BoundaryCurve != newNode.BoundaryCurve)
                    {
                        oldNode.BoundaryCurve = newNode.BoundaryCurve;
                        oldNode.RaiseLayoutChangeEvent(null);
                    }
                }
            }
            return ret;
            //LayoutAlgorithmSettings.ShowGraph(ret);
        }

        private static void AddNewNodeAndClustersToTheNewGraph(Dictionary<Node, Node> onodesToNewNodes, GeometryGraph ret)
        {
            foreach (Node newNode in onodesToNewNodes.Values)
            {
                var cl = newNode as Cluster;
                if (cl == null) {
                    ret.Nodes.Add(newNode);
                } else
                {
                    if (cl.ClusterParent == null) {
                        ret.RootCluster.AddCluster(cl);
                    }
                }
            }
        }

        private static void ReplicateClusterStructure(GeometryGraph geometryGraph, Dictionary<Node, Node> onodesToNewNodes) {
            foreach (Node onode in geometryGraph.Nodes) {
                Cluster oclparent = onode.ClusterParent;
                if (oclparent != null) {
                    Node newParent;
                    if (onodesToNewNodes.TryGetValue(oclparent, out newParent)) {
                        ((Cluster)newParent).AddNode(onodesToNewNodes[onode]);
                    }
                }
            }
        }

        /*
                bool IsRootCluster(Cluster oclparent) {
                    return !oclparent.ClusterParents.Any();
                }
        */

        private static Dictionary<Node, Node> MapSubgraphNodesToNewNodesForRouting(GeometryGraph geometryGraph)
        {
            var onodesToNewNodes = new Dictionary<Node, Node>();
            foreach (Node oNode in geometryGraph.Nodes)
            {
                var cluster = oNode as Cluster;

                onodesToNewNodes[oNode] = cluster != null
                    ? new Cluster
                    {
                        CollapsedBoundary = cluster.CollapsedBoundary,
                        BoundaryCurve = oNode.BoundaryCurve,
#if TEST_MSAGL
                        DebugId = oNode.DebugId
#endif
                    }
                    : new Node(oNode.BoundaryCurve);
            }
            return onodesToNewNodes;
        }



        /// <summary>
        /// </summary>
        public void RunOnViewChange()
        {
            this._visibleRectangle = this._lgLayoutSettings.ClientViewportMappedToGraph();
            //Rectangle.Intersect(lgLayoutSettings.ClientViewportMappedToGraph, mainGeometryGraph.BoundingBox);

            //            if (MainGeometryGraph.Edges.Count == 33) {
            //                LayoutAlgorithmSettings.ShowDebugCurves(
            //                    new DebugCurve("red", MainGeometryGraph.BoundingBox.Perimeter()),
            //                    new DebugCurve("blue", visibleRectangle.Perimeter()));
            //                LayoutAlgorithmSettings.ShowGraph(clusterOGraph);
            //            }
            if (this._visibleRectangle.IsEmpty) {
                return; //probably we should avoid this situation
            }

            this.CurrentZoomLevel = this.GetZoomFactorToTheGraph();//jyoti added -1
            //CurrentZoomLevel = Math.Floor(GetZoomFactorToTheGraph());
            this.FillRailGraph();
            this._lgLayoutSettings.OnViewerChangeTransformAndInvalidateGraph();
        }

        public double GetZoomFactorToTheGraph()
        {
            return this._lgLayoutSettings.TransformFromGraphToScreen()[0, 0] / this.FitFactor();
        }

        public double FitFactor()
        {
            Rectangle vp = this._lgLayoutSettings.ClientViewportFunc();
            this._lgLayoutSettings.mainGeometryGraphWidth = this._mainGeometryGraph.Width;
            this._lgLayoutSettings.mainGeometryGraphHeight = this._mainGeometryGraph.Height;
            return Math.Min(vp.Width / this._mainGeometryGraph.Width, vp.Height / this._mainGeometryGraph.Height);
        }

        private void FillRailGraph()
        {
            this._railGraph.Nodes.Clear();
            this.AddVisibleRailsAndNodes();
            this.RegisterEdgeSourceTargets();
        }

        private void RegisterEdgeSourceTargets()
        {
            foreach (Edge e in this._railGraph.Edges) {
                this.AddEdgeSourceAndTarget(e);
            }
        }

        private void AddEdgeSourceAndTarget(Edge edge)
        {
            this._railGraph.Nodes.Insert(edge.Source); //
            this._railGraph.Nodes.Insert(edge.Target);
        }


        /// <summary>
        /// </summary>
        /// <returns></returns>
        public double GetMaximalZoomLevel()
        {
            if (this._lgData == null) {
                return 1;
            }

            return this._lgData.GetMaximalZoomLevel();
        }

        /// <summary>
        ///     gets all edges passing through rail from the rail's level.
        /// </summary>
        /// <param name="rail"></param>
        /// <returns></returns>
        public List<Edge> GetEdgesPassingThroughRail(Rail rail)
        {
            int i = (int)Math.Log(rail.ZoomLevel, 2);
            LgLevel railLevel = this._lgData.Levels[i];
            List<Edge> passingEdges = railLevel.GetEdgesPassingThroughRail(rail);
            return passingEdges;
        }

        private int GetIndexByZoomLevel(double z)
        {
            return (int)Math.Log(z, 2);
        }

        //
        //        void GetCellInfosOfLevel(int iLevel, List<LgCellInfo> cellInfos) {
        //            Rectangle biggestTile = _lgData.BiggestTile.IsEmpty ? _mainGeometryGraph.BoundingBox : _lgData.BiggestTile;
        //
        //            var levelCellInfos = new List<LgCellInfo>();
        //            Rectangle rect1 = _visibleRectangle.Intersection(biggestTile);
        //            Rectangle rect2 = biggestTile;
        //            var d = (int) Math.Pow(2, iLevel);
        //            double cW = rect2.Width/d;
        //            double cH = rect2.Height/d;
        //            var iL = (int) ((rect1.Left - rect2.Left)/rect2.Width*d);
        //            double iR = Math.Ceiling((rect1.Right - rect2.Left)/rect2.Width*d);
        //            var iB = (int) ((rect1.Bottom - rect2.Bottom)/rect2.Height*d);
        //            double iT = Math.Ceiling((rect1.Top - rect2.Bottom)/rect2.Height*d);
        //            iL = Math.Min(d, Math.Max(iL, 0));
        //            iR = Math.Min(d, Math.Max(iR, 0));
        //            iT = Math.Min(d, Math.Max(iT, 0));
        //            iB = Math.Min(d, Math.Max(iB, 0));
        //
        //            for (int ix = iL; ix < iR; ix++) {
        //                for (int iy = iB; iy < iT; iy++) {
        //                    Point pBottomLeft = biggestTile.LeftBottom + new Point(ix*cW, iy*cH);
        //                    var r = new Rectangle(pBottomLeft, pBottomLeft + new Point(cW, cH));
        //                    var ci = new LgCellInfo
        //                    {
        //                        CellRectangle = r,
        //                        ZoomLevel = d,
        //                        MaxNumberNodesPerTile = _lgLayoutSettings.MaxNumberOfNodesPerTile
        //                    };
        //                    levelCellInfos.Add(ci);
        //                }
        //            }
        //
        //            CountContainingNodesOfLevelLeq(d, levelCellInfos);
        //            cellInfos.AddRange(levelCellInfos);
        //        }

        //        void CountContainingNodesOfLevelLeq(double zoomLevel, IEnumerable<LgCellInfo> cellInfos) {
        //            throw new NotImplementedException();
        ////            var ciRtree = new RTree<LgCellInfo,Point>();
        ////            var visCellsRect = new Rectangle();
        ////            visCellsRect.SetToEmpty();
        ////            foreach (LgCellInfo ci in cellInfos) {
        ////                ciRtree.Add(ci.CellRectangle, ci);
        ////                visCellsRect.Add(ci.CellRectangle.LeftBottom);
        ////                visCellsRect.Add(ci.CellRectangle.RightTop);
        ////            }
        ////            IEnumerable<LgNodeInfo> nodeInfosInCells = _lgNodeHierarchy.GetNodeItemsIntersectingRectangle(visCellsRect);
        ////            foreach (LgNodeInfo nodeInfo in nodeInfosInCells) {
        ////                if (nodeInfo.ZoomLevel <= zoomLevel) {
        ////                    LgCellInfo[] toCount = ciRtree.GetAllIntersecting(nodeInfo.GeometryNode.BoundingBox);
        ////                    foreach (LgCellInfo ci in toCount) {
        ////                        if (ci.ZoomLevel <= zoomLevel)
        ////                            ci.NumberNodesLeqItsLevelInside++;
        ////                    }
        ////                }
        ////            }
        //        }



        public List<LgNodeInfo> GetAllNodesOnVisibleLayers()
        {
            var nodesToSelect = new List<LgNodeInfo>();
            for (int i = 0; i < this._lgData.Levels.Count; i++)
            {
                var nodes = this.GetNodeInfosOnLevel(i);
                nodesToSelect.AddRange(nodes);
            }
            return nodesToSelect;
        }

        private IEnumerable<LgNodeInfo> GetNodeInfosOnLevel(int i)
        {
            int iStart = (i == 0 ? 0 : this._lgData.LevelNodeCounts[i - 1]);
            int iEnd = this._lgData.LevelNodeCounts[i];
            for (int j = iStart; j < iEnd; j++) {
                yield return this._lgData.SortedLgNodeInfos[j];
            }
        }

        private IEnumerable<LgNodeInfo> GetNodeInfosOnLevelLeq(int i)
        {
            int iEnd = this._lgData.LevelNodeCounts[i];
            for (int j = 0; j < iEnd; j++) {
                yield return this._lgData.SortedLgNodeInfos[j];
            }
        }

        public void InitEdgesOfLevels()
        {
            foreach (Edge edge in this.GeometryEdgesToLgEdgeInfos.Keys)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                int iLevel = this.GetIndexByZoomLevel(Math.Max(s.ZoomLevel, t.ZoomLevel));
                for (int j = iLevel; j < this._lgData.Levels.Count; j++)
                {
                    this._lgData.Levels[j]._railsOfEdges[edge] = new Set<Rail>();
                }
            }
        }

        private List<LgNodeInfo> GetNeighborsOnLevel(LgNodeInfo ni, int i)
        {
            int zoomLevel = this._lgData.Levels[i].ZoomLevel;
            Debug.Assert(ni.ZoomLevel <= zoomLevel);
            List<LgNodeInfo> neighb = (from e in ni.GeometryNode.OutEdges
                                       let nit = this._lgData.GeometryNodesToLgNodeInfos[e.Target]
                                       where nit.ZoomLevel <= zoomLevel
                                       select nit).ToList();
            neighb.AddRange(from e in ni.GeometryNode.InEdges
                            let nis = this._lgData.GeometryNodesToLgNodeInfos[e.Source]
                            where nis.ZoomLevel <= zoomLevel
                            select nis);
            return neighb;
        }



        //adapting lev's shortest path - jyoti
        public void RouteEdgesOnZeroLayer()
        {

            Tiling g1 = new Tiling(this.g.NumOfnodes, true);
            g1.N = this.g.N;
            g1.isPlanar = this.g.isPlanar;
            g1.NumOfnodesBeforeDetour = this.g.NumOfnodesBeforeDetour;
            g1.nodeTree = this.g.nodeTree;
            g1.maxTheoreticalZoomLevel = this.g.maxTheoreticalZoomLevel;
            g1.nodeToLoc = this.g.nodeToLoc;
            for (int i = 0; i < this.g.NumOfnodes; i++) {
                g1.VList[i] = new Vertex(this.g.VList[i].XLoc, this.g.VList[i].YLoc) { Id = this.g.VList[i].Id, ZoomLevel = this.g.VList[i].ZoomLevel };
            }

            var skeletonLevel = this._lgData.SkeletonLevels[0];
            skeletonLevel.ClearSavedTrajectoriesAndUsedEdges();

            foreach (LgNodeInfo ni in this.GetNodeInfosOnLevelLeq(0))
            {
                skeletonLevel.PathRouter.SetAllEdgeLengthMultipliersMin(0.8);

                this.DecreaseWeightsAlongOldTrajectoriesFromSource(0, skeletonLevel, ni);

                var neighb = this.GetNeighborsOnLevel(ni, 0);
                foreach (LgNodeInfo t in neighb)
                {
                    var path = this._lgData.SkeletonLevels[0].HasSavedTrajectory(ni, t)
                        ? skeletonLevel.GetTrajectory(ni, t)
                        : skeletonLevel.PathRouter.GetPath(ni, t, ShrinkEdgeLengths, this.g);
                    skeletonLevel.SetTrajectoryAndAddEdgesToUsed(ni, t, path);

                    //collect the path

                    int node1Id = this.PointToId[path.First()];
                    int lastnode = this.PointToId[path.Last()];
                    var zoomlevel = Math.Max(this.g.VList[node1Id].ZoomLevel, this.g.VList[lastnode].ZoomLevel);

                    int node2Id = -1;

                    foreach (Point p in path)
                    {
                        node2Id = this.PointToId[p];
                        if (node1Id == node2Id) {
                            continue;
                        }

                        int j = -1;
                        //find the neighbor where node2 is located
                        for (j = 0; j < this.g.DegList[node1Id]; j++) {
                            if (this.g.EList[node1Id, j].NodeId == node2Id) {
                                break;
                            }
                        }

                        g1.AddEdge(node1Id, node2Id, 1, zoomlevel);
                        node1Id = node2Id;
                    }




                    // decrease weights
                    skeletonLevel.PathRouter.DecreaseWeightOfEdgesAlongPath(path, 0.5);
                }
            }

            //clear unused junctions
            for (int i = 0; i < g1.NumOfnodes; i++) {
                if (g1.DegList[i] == 0) {
                    g1.VList[i].Invalid = true;
                }
            }

            this.g = g1;

        }

        private bool EdgeIsNew(LgNodeInfo s, LgNodeInfo t, int i)
        {
            return Math.Max(s.ZoomLevel, t.ZoomLevel) >= this._lgData.Levels[i].ZoomLevel;
        }

        public void RouteEdges(int i)
        {
            this.AddAllPortEdges(i);
            if (i == 0) {
                this.RouteEdgesOnZeroLayer();
            } else {
                this.RouteOnHigherLevel(i);
            }
        }

        private void RouteOnHigherLevel(int iLevel)
        {
            IEnumerable<LgNodeInfo> nodes;
            var skeletonLevel = this.DealWithPathsFromPreviousLevels(iLevel, out nodes);
            this.ComputeNewTrajectories(iLevel, nodes, skeletonLevel);
        }

        private LgSkeletonLevel DealWithPathsFromPreviousLevels(int i, out IEnumerable<LgNodeInfo> nodes)
        {
            var skeletonLevel = this._lgData.SkeletonLevels[i];
            var prevSkeletonLevel = this._lgData.SkeletonLevels[i - 1];
            nodes = this.GetNodeInfosOnLevelLeq(i);

            skeletonLevel.ClearSavedTrajectoriesAndUsedEdges();
            skeletonLevel.PathRouter.ResetAllEdgeLengthMultipliers();

            // reset multipliers
            skeletonLevel.PathRouter.ResetAllEdgeLengthMultipliers();

            this.UpdatePrevLayerTrajectories(i, nodes, prevSkeletonLevel, skeletonLevel);
            //DecreaseWeightsAlongOldTrajectories(i, skeletonLevel);

            return skeletonLevel;
        }

        /// <summary>
        /// find a better name
        /// </summary>
        /// <param name="i"></param>
        /// <param name="nodes"></param>
        /// <param name="prevSkeletonLevel"></param>
        /// <param name="skeletonLevel"></param>
        private void UpdatePrevLayerTrajectories(int i, IEnumerable<LgNodeInfo> nodes,
            LgSkeletonLevel prevSkeletonLevel, LgSkeletonLevel skeletonLevel
            )
        {
            foreach (LgNodeInfo s in nodes)
            {
                var neighb = this.GetNeighborsOnLevel(s, i).OrderBy(n => n.ZoomLevel);
                foreach (LgNodeInfo t in neighb)
                {
                    var oldTrajectory = prevSkeletonLevel.GetTrajectory(s, t);
                    if (oldTrajectory == null) {
                        continue;
                    }

                    var trajectory = skeletonLevel.PathRouter.GetPathOnSavedTrajectory(s, t,
                        oldTrajectory, ShrinkEdgeLengths);
                    skeletonLevel.SetTrajectoryAndAddEdgesToUsed(s, t, trajectory);

                    skeletonLevel.MarkEdgesAlongPathAsEdgesOnOldTrajectories(trajectory);
                }
            }
        }

        private void DecreaseWeightsAlongOldTrajectoriesFromSource(int i, LgSkeletonLevel skeletonLevel, LgNodeInfo s)
        {

            IOrderedEnumerable<LgNodeInfo> neighb = this.GetNeighborsOnLevel(s, i).OrderBy(n => n.ZoomLevel);

            foreach (var t in neighb)
            {
                var tuple = new SymmetricTuple<LgNodeInfo>(s, t);
                List<Point> path;
                skeletonLevel.EdgeTrajectories.TryGetValue(tuple, out path);
                if (path != null)
                {
                    double iEdgeLevel = Math.Log(Math.Max(s.ZoomLevel, t.ZoomLevel), 2);

                    double newWeight = (iEdgeLevel < 1 ? 0.4 : (iEdgeLevel < 2 ? 0.5 : 0.6));
                    newWeight = Math.Max(0.4, newWeight);
                    newWeight = Math.Min(0.6, newWeight);

                    skeletonLevel.PathRouter.DecreaseWeightOfEdgesAlongPath(path, newWeight);
                    //#if TEST_MSAGL
                    //                    skeletonLevel.PathRouter.AssertEdgesPresentAndPassable(path);
                    //#endif

                }

            }
        }

        private void SetWeightsAlongOldTrajectoriesFromSourceToMin(int i, LgSkeletonLevel skeletonLevel, LgNodeInfo s, double wmin)
        {
            IOrderedEnumerable<LgNodeInfo> neighb = this.GetNeighborsOnLevel(s, i).OrderBy(n => n.ZoomLevel);

            foreach (var t in neighb)
            {
                var tuple = new SymmetricTuple<LgNodeInfo>(s, t);
                List<Point> path;
                skeletonLevel.EdgeTrajectories.TryGetValue(tuple, out path);
                if (path != null)
                {
                    skeletonLevel.PathRouter.SetWeightOfEdgesAlongPathToMin(path, wmin);
                }

            }
        }

        private void ComputeNewTrajectories(int i, IEnumerable<LgNodeInfo> nodes, LgSkeletonLevel skeletonLevel)
        {
            int numRouted = 0;

            // reset multipliers
            skeletonLevel.PathRouter.SetAllEdgeLengthMultipliersMin(0.8);

            foreach (LgNodeInfo s in nodes)
            {

                // decrease weights along old trajectories
                this.DecreaseWeightsAlongOldTrajectoriesFromSource(i, skeletonLevel, s);

                IOrderedEnumerable<LgNodeInfo> neighb = this.GetNeighborsOnLevel(s, i).OrderBy(n => n.ZoomLevel);

                foreach (LgNodeInfo t in neighb)
                {

                    List<Point> path = skeletonLevel.GetTrajectory(s, t);
                    if (path == null)
                    {
                        path = skeletonLevel.PathRouter.GetPath(s, t, ShrinkEdgeLengths);
                        skeletonLevel.SetTrajectoryAndAddEdgesToUsed(s, t, path);
                    }
                    numRouted++;
                    if (numRouted % 100 == 1) {
                        Console.Write(".");
                    }
                }

                // reset multipliers
                this.SetWeightsAlongOldTrajectoriesFromSourceToMin(i, skeletonLevel, s, 0.8);
            }
        }

        private void ClearLevels(int num)
        {
            this._lgData.Levels.Clear();
            this._lgData.SkeletonLevels.Clear();
            int zoomLevel = 1;
            for (int i = 0; i < num; i++)
            {
                var level = new LgLevel(zoomLevel, this._mainGeometryGraph);
                level.ClearRailTree();

                this._lgData.Levels.Add(level);
                this._lgData.SkeletonLevels.Add(new LgSkeletonLevel { ZoomLevel = zoomLevel });

                zoomLevel *= 2;
            }
        }

        private void UpdateNodeInfoZoomLayers()
        {
            int level = 1;
            int j = 0;
            for (int i = 0; i < this._lgData.SortedLgNodeInfos.Count; i++)
            {
                while (j < this._lgData.LevelNodeCounts.Count && i == this._lgData.LevelNodeCounts[j])
                {
                    j++;
                    level *= 2;
                }
                this._lgData.SortedLgNodeInfos[i].ZoomLevel = level;
            }
        }

        private void InitRailsOfEdgesEmpty()
        {
            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                double zoomLevel = Math.Max(s.ZoomLevel, t.ZoomLevel);

                this.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel = zoomLevel;
                int iLevel = this.GetIndexByZoomLevel(zoomLevel); //(int) Math.Log(zoomLevel, 2);
                for (int j = iLevel; j < this._lgData.Levels.Count; j++) {
                    this._lgData.Levels[j]._railsOfEdges[edge] = new Set<Rail>();
                }
            }
        }

        private void InitRailsOfEdgesEmpty(int iMinLevel)
        {
            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                double zoomLevel = Math.Max(s.ZoomLevel, t.ZoomLevel);
                this.GeometryEdgesToLgEdgeInfos[edge].ZoomLevel = zoomLevel;

                int iLevel = Math.Max(this.GetIndexByZoomLevel(zoomLevel), iMinLevel);
                for (int j = iLevel; j < this._lgData.Levels.Count; j++) {
                    this._lgData.Levels[j]._railsOfEdges[edge] = new Set<Rail>();
                }
            }
        }

        public void AddAllPortEdges(int iLevel)
        {
            var nodes = this.GetNodeInfosOnLevelLeq(iLevel);
            var skeletonLevel = this._lgData.SkeletonLevels[iLevel];
            skeletonLevel.AddGraphEdgesFromCentersToPointsOnBorders(nodes);
            //SplineRouter.ShowVisGraph(skeletonLevel.PathRouter.VisGraph, null,null,null);
        }

        private void CreateGraphFromSteinerCdt(int iLevel)
        {
            var nodes = this.GetNodeInfosOnLevelLeq(iLevel);
            var cdt = new SteinerCdt(this._lgData.SkeletonLevels[iLevel].PathRouter.VisGraph, nodes);
            //cdt.ReadTriangleOutputAndPopulateTheLevelVisibilityGraphFromTriangulation();
            for (int index = 0; index < this.g.NumOfnodes; index++)
            {
                var p = new Point(this.g.VList[index].XLoc, this.g.VList[index].YLoc);
                var w = cdt._visGraph.AddVertex(p);
                if (index < this.g.NumOfnodesBeforeDetour) {
                    w.isReal = true;
                } else {
                    w.isReal = false;
                }

                cdt._outPoints[index] = w;
                cdt._visGraph.PointToVertexMap[p] = w;
                cdt._visGraph.visVertexToId[w] = index;
                this.PointToId[p] = index;
            }

            for (int index = 0; index < this.g.NumOfnodes; index++)
            {
                for (int neighbor = 0; neighbor < this.g.DegList[index]; neighbor++)
                {
                    var a = cdt._outPoints[index];
                    var b = cdt._outPoints[this.g.EList[index, neighbor].NodeId];
                    cdt.AddVisEdge(a, b);
                }
            }
        }
        /*
                void CreateGraphFromSteinerCdt(int iLevel)
                {
                    var nodes = GetNodeInfosOnLevelLeq(iLevel);
                    var cdt = new SteinerCdt(_lgData.SkeletonLevels[iLevel].PathRouter.VisGraph, nodes);
                    cdt.ReadTriangleOutputAndPopulateTheLevelVisibilityGraphFromTriangulation();
                }
                */
        public bool SimplifyRoutes(int iLevel)
        {
            var nodes = this.GetNodeInfosOnLevelLeq(iLevel);
            var skeletonLevel = this._lgData.SkeletonLevels[iLevel];
            var fixedVertices = new Set<Point>(nodes.Select(n => n.Center));
            if (iLevel > 0) {
                fixedVertices.InsertRange(this._lgData.SkeletonLevels[iLevel - 1].GetPointsOnSavedTrajectories());
            }
#if TEST_MSAGL
            //            SplineRouter.ShowVisGraph(skeletonLevel.PathRouter.VisGraph, nodes.Select(n=>n.BoundaryOnLayer), null, null);
#endif

            var routeSimplifier = new RouteSimplifier(skeletonLevel.PathRouter, nodes, fixedVertices);
            return routeSimplifier.Run();
        }


#if TEST_MSAGL && !SHARPKIT
        private static void ShowOldNewIntersected(Set<Rail> oldIntersected, Set<Rail> newIntersected, Point a, Point shortcutted,
            Point b, LgSkeletonLevel skeletonLevel)
        {
            var ll = new List<DebugCurve>();
            ll.Add(new DebugCurve(new LineSegment(a, shortcutted)));
            ll.Add(new DebugCurve(new LineSegment(shortcutted, b)));
            ll.AddRange(oldIntersected.Select(r => new DebugCurve(100, 0.1, "blue", LineFromRail(r))));
            ll.AddRange(newIntersected.Select(r => new DebugCurve(100, 0.1, "red", LineFromRail(r))));
            ll.Add(new DebugCurve(CurveFactory.CreateCircle(3, shortcutted)));
            ll.Add(new DebugCurve("red", CurveFactory.CreateCircle(3, a)));
            ll.Add(new DebugCurve("green", CurveFactory.CreateCircle(3, b)));

            foreach (var rail in newIntersected)
            {
                var e = FindRailInVisGraph(rail, skeletonLevel);
                if (e == null)
                {
                    Console.WriteLine("rail {0} is not found in the vis graph", rail);
                }
            }


            LayoutAlgorithmSettings.ShowDebugCurves(ll.ToArray());
        }
#endif

#if TEST_MSAGL && !SHARPKIT
        private static void ShowOldTrajectories(LgSkeletonLevel skeletonLevel)
        {
            if (skeletonLevel.ZoomLevel <= 1.0) {
                return;
            }

            var segs = skeletonLevel.PathRouter.EdgesOnOldTrajectories();
            var ll = new List<DebugCurve>();
            foreach (var seg in segs)
            {
                ll.Add(new DebugCurve(new LineSegment(seg.A, seg.B)));
            }
            LayoutAlgorithmSettings.ShowDebugCurves(ll.ToArray());
        }
#endif

        private static VisibilityEdge FindRailInVisGraph(Rail rail, LgSkeletonLevel skeletonLevel)
        {
            Point s, t;
            rail.GetStartEnd(out s, out t);
            return skeletonLevel.PathRouter.FindEdge(s, t);
        }

        private static ICurve LineFromRail(Rail rail)
        {
            Point s, t;
            rail.GetStartEnd(out s, out t);
            return new LineSegment(s, t);
        }

        /*
        var rect = new Rectangle(a, b);
            foreach (var railNode in skeletonLevel._railTree.GetAllLeavesIntersectingRectangle(rect)) {
                var rail = railNode.UserData;
                if (NontrivialIntersection(a, b, rail)) {
                    return false;
                }
            }
            return true;
        }
        */

        public void FillLevelsWithNodesOnly(int maxNodesPerTile)
        {
            var calc = new GreedyNodeRailLevelCalculator(this._lgData.SortedLgNodeInfos)
            {
                GeometryNodesToLgNodeInfos = this._lgData.GeometryNodesToLgNodeInfos
            };
            calc.MaxAmountNodesPerTile = maxNodesPerTile;
            calc.IncreaseNodeQuota = this._lgLayoutSettings.IncreaseNodeQuota;

            var bbox = this.GetLargestTile();

            calc.PlaceNodesOnly(bbox);
            this._lgData.LevelNodeCounts = calc.GetLevelNodeCounts();

            this.ClearLevels(this._lgData.LevelNodeCounts.Count);
        }

        public void FillLevelWithNodesRoutesTryRerouting(int iLevel, int maxNodesPerTile, int maxSegmentsPerTile,
            double increaseNodeQuota)
        {
            var calc = new GreedyNodeRailLevelCalculator(this._lgData.SortedLgNodeInfos)
            {
                GeometryNodesToLgNodeInfos = this._lgData.GeometryNodesToLgNodeInfos
            };
            calc.InitBoundingBox();

            calc.MaxAmountNodesPerTile = maxNodesPerTile;
            calc.MaxAmountRailsPerTile = maxSegmentsPerTile;
            calc.IncreaseNodeQuota = increaseNodeQuota;

            var trajectories =
                this._lgData.SkeletonLevels[iLevel].EdgeTrajectories;
            int numToInsert = this._lgData.LevelNodeCounts[iLevel];

            int zoomLevel = this._lgData.Levels[iLevel].ZoomLevel;
            int nodesOnPrevLevel = iLevel > 0 ? this._lgData.LevelNodeCounts[iLevel - 1] : 0;

            var bbox = this.GetLargestTile();

            var oldSegs = this.GetSegmentsOnOldTrajectories(iLevel);

            //int numInserted = calc.TryInsertingNodesAndRoutesTryRerouting(numToInsert, trajectories, zoomLevel,
            //    nodesOnPrevLevel, new GridTraversal(bbox, iLevel), _lgData.SkeletonLevels[iLevel].PathRouter);

            int numInserted = calc.TryInsertingNodesAndRoutes(numToInsert, trajectories, oldSegs, zoomLevel,
    nodesOnPrevLevel, new GridTraversal(bbox, iLevel), this._lgData.SkeletonLevels[iLevel].PathRouter);

            if (iLevel > 0 && numInserted < this._lgData.LevelNodeCounts[iLevel - 1]) {
                // couldn't even get to current level
                numInserted = this._lgData.LevelNodeCounts[iLevel - 1];
            }

            if (numInserted < numToInsert) {
                this.PushNodesToNextLevel(iLevel, numInserted);
            }

            // update level node counts
            this._lgData.LevelNodeCounts[iLevel] = numInserted;
        }

        public List<SymmetricSegment> GetSegmentsOnOldTrajectories(int iLevel)
        {
            var segs = new List<SymmetricSegment>();
            for (int i = 0; i < iLevel; i++)
            {
                segs.AddRange(this._lgData.SkeletonLevels[i].PathRouter.SegmentsNotOnOldTrajectories());
            }
            return segs;
        }

        private void AssignRailsByTrajectories(int iLevel)
        {
            var edges = new List<Edge>(this._lgData.Levels[iLevel]._railsOfEdges.Keys);
            this._lgData.Levels[iLevel]._railTree = new RTree<Rail,Point>();
            foreach (Edge edge in edges)
            {
                LgEdgeInfo ei = this._lgData.GeometryEdgesToLgEdgeInfos[edge];
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                if (this._lgData.SkeletonLevels[iLevel].HasSavedTrajectory(s, t))
                {
                    List<Point> trajectory = this._lgData.SkeletonLevels[iLevel].GetTrajectory(s, t);
                    List<Rail> rails = this._lgData.Levels[iLevel].FetchOrCreateRailSequence(trajectory);
                    this._lgData.Levels[iLevel]._orderedRailsOfEdges[edge] = rails;

                    this._lgData.AssembleEdgeAtLevel(ei, iLevel, new Set<Rail>(rails));
                }
            }
        }

        private void PushNodesToNextLevel(int iLevel, int numInserted)
        {
            if (iLevel == this._lgData.Levels.Count - 1)
            {
                var level = new LgLevel(this._lgData.Levels[iLevel].ZoomLevel * 2, this._mainGeometryGraph);
                level.CreateEmptyRailTree();
                this._lgData.Levels.Add(level);
                this._lgData.SkeletonLevels.Add(new LgSkeletonLevel { ZoomLevel = level.ZoomLevel });
                this._lgData.LevelNodeCounts.Add(this._lgData.LevelNodeCounts.Last());
            }

            int numToInsert = this._lgData.LevelNodeCounts[iLevel];
            if (numToInsert <= numInserted) {
                return;
            }

            int newZoomLevel = this._lgData.Levels[iLevel + 1].ZoomLevel;
            for (int i = numInserted; i < numToInsert; i++)
            {
                this._lgData.SortedLgNodeInfos[i].ZoomLevel = newZoomLevel;
            }

            this._lgData.LevelNodeCounts[iLevel] = numInserted;

            // need to update assigned edge routes
            this._lgData.Levels[iLevel]._railsOfEdges.Clear();
            this._lgData.Levels[iLevel + 1]._railsOfEdges.Clear();
            this.InitRailsOfEdgesEmpty(iLevel);
        }


        /// <summary>
        /// calculates all info for LG Browsing
        /// </summary>
        /// <param name="maxNodesPerTile"></param>
        /// <param name="maxSegmentsPerTile"></param>
        /// <param name="increaseNodeQuota"></param>
        public void LayoutAndRouteByLayers(int maxNodesPerTile, int maxSegmentsPerTile, double increaseNodeQuota)
        {
            this.FillLevelsWithNodesOnly(maxNodesPerTile);

            this.InitRailsOfEdgesEmpty();
            //InitNodeLabelWidthToHeightRatios();
            this.AddSkeletonLevels();
            for (int i = 0; i < this._lgData.Levels.Count; i++) {
                this.RemoveOverlapsAndRouteForLayer(maxNodesPerTile, maxSegmentsPerTile, increaseNodeQuota, i);
            }

            this._lgData.CreateLevelNodeTrees(this.NodeDotWidth(1));
            //LabelingOfOneRun();
#if TEST_MSAGL
            this.TestAllEdgesConsistency();
#endif
        }

        public Rectangle GetLargestTile()
        {
            var bbox = this._mainGeometryGraph.boundingBox.Clone();
            var p = this.NodeDotWidth(1) * 0.5;
            bbox.Pad(p, p, p, p);
            return bbox;
        }

        public void LabelingOfOneRun()
        {
            foreach (LgNodeInfo node in this._lgData.SortedLgNodeInfos)
            {
                //if (node.ZoomLevel > 1.0)
                //    node.LabelVisibleFromScale = node.ZoomLevel;
                //else
                node.LabelVisibleFromScale = 0.0;
            }

            double scale = 1.0 / 16.0;

            double delta = Math.Pow(2, 1.0 / 8.0);

            int numberOfLabeledNodes = 0;
            double hugeScale = Math.Pow(2, this._lgData.Levels.Last().ZoomLevel + 1);
            while (numberOfLabeledNodes < this._lgData.GeometryNodesToLgNodeInfos.Count && scale <= hugeScale)
            {
                numberOfLabeledNodes = this.InsertLabelsGreedily(scale);
                scale *= delta;
            }

            this.CleanUpRails();

            // make labels appear slightly earlier
            foreach (LgNodeInfo node in this._lgData.SortedLgNodeInfos)
            {
                node.LabelVisibleFromScale -= 0.0001;
            }
        }

        private void RemoveOverlapsAndRouteForLayer(int maxNodesPerTile, int maxSegmentsPerTile, double increaseNodeQuota,
            int iLayer)
        {
            this.PrepareNodeBoundariesAndSkeletonOnLayer(iLayer);

            var skeletonLevel = this._lgData.SkeletonLevels[iLayer];

            this.CreateGraphFromSteinerCdt(iLayer);
            var bbox = this.GetLargestTile();

            var gt = new GridTraversal(bbox, iLayer);

            this.RouteEdges(iLayer);
            this.RemoveUnusedVisibilityGraphEdgesAndNodes(iLayer);
        }

        private bool HigherTrajectoriesPreserved(int iLayer)
        {
            for (int j = 0; j <= iLayer; j++) {
                if (!this.TestTrajectoriesPreserved(iLayer)) {
                    return false;
                }
            }

            return true;
        }

        private void PrepareNodeBoundariesAndSkeletonOnLayer(int iLayer)
        {
            var scale = Math.Pow(2, iLayer);
            var rad = this.NodeDotWidth(scale); // make the BoundaryOnLayer twice larger than the node
            for (int i = 0; i < this._lgData.LevelNodeCounts[iLayer]; i++)
            {
                LgNodeInfo ni = this._lgData.SortedLgNodeInfos[i];
                ni.BoundaryOnLayer = CurveFactory.CreateRegularPolygon(this._lgLayoutSettings.NumberOfNodeShapeSegs,
                    ni.Center, rad);
            }
            if (iLayer > 0)
            {
                this.ModifySkeletonWithNewBoundariesOnLayer(iLayer);
            }
        }

        private void ModifySkeletonWithNewBoundariesOnLayer(int iLayer)
        {
            for (int i = 0; i < this._lgData.LevelNodeCounts[iLayer]; i++) {
                this._lgData.SkeletonLevels[iLayer].PathRouter.ModifySkeletonWithNewBoundaryOnLayer(
                    this._lgData.SortedLgNodeInfos[i]);
            }
        }

        private void RemoveUnusedVisibilityGraphEdgesAndNodes(int i)
        {
            this._lgData.SkeletonLevels[i].RemoveUnusedGraphEdgesAndNodes();
        }

        private void SimplifyRoutesOnLevelUntilDone(int i)
        {
#if TEST_MSAGL
            //            ShowOldTrajectories(_lgData.SkeletonLevels[i]);
#endif

            do
            {
                if (!this.SimplifyRoutes(i)) {
                    break;
                }

                this.UpdateRoutesAfterSimplification(i);
            } while (true);

#if TEST_MSAGL
            //            ShowOldTrajectories(_lgData.SkeletonLevels[i]);
#endif
        }

        public void InitNodeLabelWidthToHeightRatios(List<double> noldeLabelRatios)
        {
            for (int i = 0; i < this._mainGeometryGraph.Nodes.Count; i++)
            {
                var n = this._mainGeometryGraph.Nodes[i];
                var ni = this._lgData.GeometryNodesToLgNodeInfos[n];
                if (ni != null)
                {
                    ni.LabelWidthToHeightRatio = noldeLabelRatios[i];
                }
            }
        }

        private void RemoveTrajectoriesForEdgesWithHighZoom(int iLevel)
        {
            // some edges are no longer used since their adjacent nodes moved to the next level.
            // need to delete them from the level EdgeTrajectories
            var edgeTrajectories = this._lgData.SkeletonLevels[iLevel].EdgeTrajectories;
            int zoomLevel = this._lgData.Levels[iLevel].ZoomLevel;
            var removeList =
                new List<SymmetricTuple<LgNodeInfo>>(
                    edgeTrajectories.Keys.Where(p => p.A.ZoomLevel > zoomLevel || p.B.ZoomLevel > zoomLevel));
            if (removeList.Count > 0)
            {
                this._lgData.SkeletonLevels[iLevel].RemoveSomeEdgeTrajectories(removeList);
                this._lgData.Levels[iLevel].RemoveFromRailEdges(removeList);
            }
        }

        private void TestRailsForTrajectories(int iLevel)
        {
            foreach (var path in this._lgData.SkeletonLevels[iLevel].EdgeTrajectories.Values)
            {
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Rail rail = this._lgData.Levels[iLevel].FindRail(path[i], path[i + 1]);
                    if (rail == null)
                    {
                        Console.WriteLine("Rail not found for trajectory!");
                    }
                }
            }
        }

        private void UpdateRoutesAfterSimplification(int i)
        {
            var skeletonLevel = this._lgData.SkeletonLevels[i];
            foreach (Edge edge in this._lgData.Levels[i]._railsOfEdges.Keys)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                List<Point> path = skeletonLevel.GetTrajectory(s, t);
                if (path == null) {
                    continue;
                }

                int startedToSkip = 0;
                for (int j = 1; j < path.Count - 1; j++)
                {
                    if (skeletonLevel.PathRouter.ContainsVertex(path[j]))
                    {
                        if (startedToSkip > 0)
                        {
                            skeletonLevel.PathRouter.MarkEdgeUsed(path[startedToSkip - 1], path[j]);
                            startedToSkip = 0;
                        }
                        continue;
                    }
                    path.RemoveAt(j);
                    if (startedToSkip == 0) {
                        startedToSkip = j;
                    }

                    j--;
                }
                if (startedToSkip > 0) {
                    skeletonLevel.PathRouter.MarkEdgeUsed(path[startedToSkip - 1], path.Last());
                }

                if (!this.EdgeIsNew(s, t, i))
                {
                    skeletonLevel.MarkEdgesAlongPathAsEdgesOnOldTrajectories(path);
                }
            }
        }

        private bool TestTrajectoriesPreserved(int iLevel)
        {
            if (iLevel == 0) {
                return true;
            }

            foreach (var st in this._lgData.SkeletonLevels[iLevel - 1].EdgeTrajectories)
            {
                LgNodeInfo s = st.Key.A;
                LgNodeInfo t = st.Key.B;
                List<Point> oldPath = st.Value;
                Point ps = oldPath.First();
                List<Point> newPath = this._lgData.SkeletonLevels[iLevel].GetTrajectory(s, t);
                Debug.Assert(newPath.Count > 1);
                if (!newPath.First().Equals(ps)) {
                    newPath.Reverse();
                }

                if (!newPath.First().Equals(ps))
                {
                    // Endpoint of old path not found on new path!
                    return false;
                }

                int i;
                int j;
                for (i = 1, j = 1; i < oldPath.Count; i++, j++)
                {
                    var subdivEdge = new List<Point> { oldPath[i - 1] };
                    while (j < newPath.Count && !oldPath[i].Equals(newPath[j]))
                    {
                        subdivEdge.Add(newPath[j]);
                        j++;
                    }
                    if (j == newPath.Count || !oldPath[i].Equals(newPath[j]))
                    {
                        // Point of old path not found on new path!
                        return false;
                    }
                    subdivEdge.Add(newPath[j]);

                    bool subdivEdgeIsLine = RectSegIntersection.ArePointsOnLine(subdivEdge);
                    if (subdivEdgeIsLine) {
                        continue;
                    }

                    return false;
                }
            }
            return true;
        }

        private void CleanUpRails()
        {
            foreach (LgLevel level in this._lgData.Levels)
            {
                var usedRails = new Set<Rail>();
                foreach (var rails in level._railsOfEdges.Values)
                {
                    usedRails.InsertRange(rails);
                }

                var unusedRails = new Set<Rail>(level._railDictionary.Values.Where(r => !usedRails.Contains(r)));
                if (!unusedRails.Any()) {
                    continue;
                }

                foreach (Rail rail in unusedRails)
                {
                    level.RemoveRailFromDictionary(rail);
                    level.RemoveRailFromRtree(rail);
                }
            }
        }

        private void TestAllEdgesConsistency()
        {
            foreach (Edge edge in this._mainGeometryGraph.Edges)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[edge.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[edge.Target];
                double zoomLevel = Math.Max(s.ZoomLevel, t.ZoomLevel);
                int iLevel = this.GetIndexByZoomLevel(zoomLevel);
                for (int i = iLevel; i < this._lgData.Levels.Count; i++)
                {
                    Set<Rail> rails = this._lgData.Levels[i]._railsOfEdges[edge];
                    foreach (Rail rail in rails)
                    {
                        var rc = rail.Geometry as ICurve;
                        if (rc != null)
                        {
                            var ss = new SymmetricSegment(rc.Start, rc.End);
                            if (!this._lgData.Levels[i]._railDictionary.ContainsKey(ss)) {
                                Debug.Assert(false, string.Format("rail {0} is not in the dictionary", rail));
                            }
                        }
                    }
                }
            }
        }

        public void SelectAllColoredEdgesIncidentTo(LgNodeInfo nodeInfo, object color)
        {
            List<Edge> edges = nodeInfo.GeometryNode.Edges.ToList();

            //START-jyoti to select only the neighbors within the current zoom level
            List<Edge> filteredEdges = new List<Edge>();
            Dictionary<Edge,object> reselectEdges = new Dictionary<Edge, object>();

            foreach (Edge edge in edges)
            {
                //jyoti - user selects a and then the neighbor b of a is selected
                // if the user deselect b, then we still need the edge (a,b) to be there.
                // so reselect these edges.
                if (this._lgData.GeometryNodesToLgNodeInfos[edge.Source].Color != null &&
                    this._lgData.GeometryNodesToLgNodeInfos[edge.Target].Color != null)
                {
                    if (nodeInfo.GeometryNode.Center == edge.Source.Center) {
                        reselectEdges[edge] = this._lgData.GeometryNodesToLgNodeInfos[edge.Target].Color;
                    } else {
                        reselectEdges[edge] = this._lgData.GeometryNodesToLgNodeInfos[edge.Source].Color;
                    }
                }
                
                 


                filteredEdges.Add(edge);

                if (!nodeInfo.Selected)
                {
                    edge.Color = nodeInfo.Color;
                    if(nodeInfo.GeometryNode.Center == edge.Source.Center) {
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].SelectedNeighbor++; // = !nodeInfo.Selected;
                    } else {
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Source].SelectedNeighbor++; //= !nodeInfo.Selected;                    
                    }
                }
                else
                {
                    if (nodeInfo.GeometryNode.Center == edge.Source.Center) {
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Target].SelectedNeighbor--; // = !nodeInfo.Selected;
                    } else {
                        this._lgData.GeometryNodesToLgNodeInfos[edge.Source].SelectedNeighbor--; //= !nodeInfo.Selected;
                    }
                }
            }
            edges = filteredEdges;
            //END-jyoti to select only the neighbors within the current zoom level


            if (!nodeInfo.Selected) {
                this._lgData.SelectEdges(edges);
            } else
            {
                this._lgData.UnselectColoredEdges(edges, color);

                //reselect the edges that have been deselected by the complcated case of "both endvertices selection"                
                foreach (Edge edge in reselectEdges.Keys) {
                    edge.Color = reselectEdges[edge];
                }

                this._lgData.SelectEdges(reselectEdges.Keys.ToList());
 
            }

            

        }

        public void SelectAllEdgesIncidentTo(LgNodeInfo nodeInfo)
        {
            List<Edge> edges = nodeInfo.GeometryNode.Edges.ToList();

            //START-jyoti to select only the neighbors within the current zoom level
            List<Edge> filteredEdges = new List<Edge>();
            foreach (Edge edge in edges)
            {
                //if (_lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel > CurrentZoomLevel ||
                //    _lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel > CurrentZoomLevel) continue;
                filteredEdges.Add(edge); 
                if (!nodeInfo.Selected)
                {
                    edge.Color = nodeInfo.Color;
                   // _lgData.GeometryNodesToLgNodeInfos[edge.Source].SelectedNeighbor++; //= !nodeInfo.Selected;
                  //  _lgData.GeometryNodesToLgNodeInfos[edge.Target].SelectedNeighbor++; // = !nodeInfo.Selected;
                }
                else
                {
                   // _lgData.GeometryNodesToLgNodeInfos[edge.Source].SelectedNeighbor--; //= !nodeInfo.Selected;
                   // _lgData.GeometryNodesToLgNodeInfos[edge.Target].SelectedNeighbor--; // = !nodeInfo.Selected;
                }
            }
            edges = filteredEdges;
            //END-jyoti to select only the neighbors within the current zoom level


            if (!nodeInfo.Selected) {
                this._lgData.SelectEdges(edges);
            } else
            {
                this._lgData.UnselectEdges(edges);                
            }
                
        }

        public void UpdateVisibleEdgesIncidentTo(LgNodeInfo nodeInfo, int currentLayer)
        {
            List<Edge> edges = nodeInfo.GeometryNode.Edges.ToList();
            int currentZoomFromLayer = Math.Max(0, currentLayer - 1);

            //START-jyoti to select only the neighbors within the current zoom level
            List<Edge> filteredEdges = new List<Edge>();

            foreach (Edge edge in edges)
            {
                if (this._lgData.GeometryNodesToLgNodeInfos[edge.Source].ZoomLevel <= currentZoomFromLayer &&
                    this._lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel <= currentZoomFromLayer) {
                    filteredEdges.Add(edge);
                }
            }

            edges = filteredEdges;
            //END-jyoti to select only the neighbors within the current zoom level


            if (nodeInfo.Selected) {
                this._lgData.SelectEdges(edges, Math.Min(currentLayer, this._lgLayoutSettings.maximumNumOfLayers));
            }
        }

        public void SelectEdge(LgEdgeInfo ei)
        {
            var edges = new List<Edge> { ei.Edge };
            this._lgData.SelectEdges(edges);
        }

        public void DeselectAllEdges()
        {
            this._lgData.PutOffAllEdges();
        }


        public void RunMds()
        {
            //DistributeUniformlyRandom();
            //return;

            var multipliers = new Dictionary<Tuple<Node, Node>, double>();

            //foreach (var s in lgData.SortedLgNodeInfos)
            //{
            //    foreach (var t in lgData.SortedLgNodeInfos)
            //    {
            //        if (s == t) continue;
            //        if (s.ZoomLevel < 2 && t.ZoomLevel < 2)
            //            multipliers.Add(new Tuple<Node, Node>(s.GeometryNode, t.GeometryNode), 10);
            //    }
            //}

            foreach (Edge e in this.GeometryEdgesToLgEdgeInfos.Keys)
            {
                LgNodeInfo s = this._lgData.GeometryNodesToLgNodeInfos[e.Source];
                LgNodeInfo t = this._lgData.GeometryNodesToLgNodeInfos[e.Target];
                double zoomLevel = Math.Max(s.ZoomLevel, t.ZoomLevel);

                if (zoomLevel < 5)
                {
                    double mult = 10; // = Math.Pow(10, lgData.Levels.Last().ZoomLevel - zoomLevel);
                    multipliers.Add(new Tuple<Node, Node>(s.GeometryNode, t.GeometryNode), mult);

                    e.Length = 3;
                    //Math.Pow(1.5, lgData.Levels.Last().ZoomLevel - zoomLevel); //lgData.Levels.Last().ZoomLevel - zoomLevel + 1;
                    //Math.Pow(2, lgData.Levels.Last().ZoomLevel - zoomLevel);
                }
                //else if(Math.Min(s.ZoomLevel, t.ZoomLevel) < 2)
                //{
                //    e.Length = 0.1;
                //}
            }


            var settings = new MdsLayoutSettings { ScaleX = 600, ScaleY = 600, RunInParallel = false, PivotNumber = 100 };

            //var mds = new MDSGraphLayoutWeighted(settings, mainGeometryGraph, multipliers);
            var mds = new MdsGraphLayout(settings, this._mainGeometryGraph);
            mds.Run();
        }

        public void SelectTopEdgePassingThroughRailWithEndpoint(Rail rail, Set<LgNodeInfo> selectedVnodes)
        {
            List<Edge> edges =
                this.GetEdgesPassingThroughRail(rail)
                    .Where(e => selectedVnodes.Contains(this._lgData.GeometryNodesToLgNodeInfos[e.Source])
                                || selectedVnodes.Contains(this._lgData.GeometryNodesToLgNodeInfos[e.Target])).ToList();
            if (!edges.Any()) {
                return;
            }

            this.DeselectAllEdges();
            Edge edge = edges.OrderByDescending(e => this._lgData.GeometryEdgesToLgEdgeInfos[e].Rank).First();
            this.SelectEdge(this._lgData.GeometryEdgesToLgEdgeInfos[edge]);
        }

        private Rectangle GetLabelRectForScale(LgNodeInfo nodeInfo, double scale)
        {
            double labelHeight = this._lgLayoutSettings.NodeLabelHeightInInches * this._lgLayoutSettings.DpiX / scale /
                                 this.FitFactor();
            double labelWidth = labelHeight * nodeInfo.LabelWidthToHeightRatio;

            double nodeDotWidth = this.GetNodeDotRect(nodeInfo, scale).Width;
            //_lgLayoutSettings.NodeDotWidthInInches * _lgLayoutSettings.DpiX / currentScale;

            Point offset = Point.Scale(labelWidth + nodeDotWidth * 1.01, labelHeight + nodeDotWidth * 1.01,
                nodeInfo.LabelOffset);

            double overlapScale = 0.7;
            var d = new Point(0.5 * labelWidth, 0.5 * labelHeight) * overlapScale;
return new Rectangle(nodeInfo.Center + offset - d, nodeInfo.Center + offset + d);
        }

        private Rectangle GetLabelRectForScale(LgNodeInfo nodeInfo, LabelPlacement placement, double scale)
        {
            double labelHeight = this._lgLayoutSettings.NodeLabelHeightInInches * this._lgLayoutSettings.DpiX / scale /
                                 this.FitFactor();
            double labelWidth = labelHeight * nodeInfo.LabelWidthToHeightRatio;

            double nodeDotWidth = this.GetNodeDotRect(nodeInfo, scale).Width;
            //_lgLayoutSettings.NodeDotWidthInInches * _lgLayoutSettings.DpiX / currentScale;            

            Point offset = Point.Scale(labelWidth + nodeDotWidth * 1.01, labelHeight + nodeDotWidth * 1.01,
                LgNodeInfo.GetLabelOffset(placement));

            var d = new Point(0.5 * labelWidth, 0.5 * labelHeight);

            return new Rectangle(nodeInfo.Center + offset - d, nodeInfo.Center + offset + d);
        }

        private Rectangle GetNodeDotRect(LgNodeInfo node, double scale)
        {
            var rect = new Rectangle(node.Center);
            rect.Pad(this.NodeDotWidth(scale) / 2);
            return rect;
        }

        private double NodeDotWidth(double scale)
        {
            return this._lgLayoutSettings.NodeDotWidthInInches * this._lgLayoutSettings.DpiX / scale / this.FitFactor();
        }

        private void InsertCandidateLabelsGreedily(List<LgNodeInfo> candidates, double _scale)
        {
            var scale = this.CurrentZoomLevel;

            var labelRTree = new RTree<LgNodeInfo, Point>();

            // insert all nodes inserted before
            foreach (var node in this._railGraph.Nodes)
            {

                var ni = this._lgData.GeometryNodesToLgNodeInfos[node];

                // add all node dots
                labelRTree.Add(this.GetNodeDotRect(ni, scale), ni);

                if (ni.LabelVisibleFromScale <= scale)
                {
                    Rectangle labelRect = this.GetLabelRectForScale(ni, scale);
                    labelRTree.Add(labelRect, ni);
                }
            }

            foreach (LgNodeInfo node in candidates)
            {
                if (node.LabelVisibleFromScale <= scale)
                {
                    // already inserted before
                    continue;
                }

                LabelPlacement[] positions =
                {
                    LabelPlacement.Bottom,
                    LabelPlacement.Right,
                    LabelPlacement.Left,
                    LabelPlacement.Top
                };

                bool couldPlace = false;
                var labelRect = new Rectangle();

                LabelPlacement pl = LabelPlacement.Bottom;

                foreach (LabelPlacement placement in positions)
                {
                    pl = placement;
                    labelRect = this.GetLabelRectForScale(node, pl, scale);
                    if (!labelRTree.IsIntersecting(labelRect))
                    {
                        couldPlace = true;
                        break;
                    }
                }


                if (couldPlace)
                {
                    labelRTree.Add(labelRect, node);
                    this.SelectedNodeLabels[node] = pl;
                }
            }

        }

        private int InsertLabelsGreedily(double scale)
        {
            int labeledNodes = 0;

            int iLevel = 0;
            while (iLevel < this._lgData.Levels.Count - 1 && scale >= this._lgData.Levels[iLevel].ZoomLevel)
            {
                iLevel++;
            }

            var nodes = this.GetNodeInfosOnLevelLeq(iLevel);

            var labelRTree = new RTree<LgNodeInfo, Point>();

            var skippedRTree = new RTree<LgNodeInfo, Point>();


            // insert all nodes inserted before
            foreach (LgNodeInfo node in nodes)
            {
                // add all node dots
                labelRTree.Add(this.GetNodeDotRect(node, scale), node);

                if (node.LabelVisibleFromScale > 0 && node.LabelVisibleFromScale < scale)
                {
                    Rectangle labelRect = this.GetLabelRectForScale(node, scale);
                    labelRTree.Add(labelRect, node);
                    labeledNodes++;
                }
            }

            foreach (LgNodeInfo node in nodes)
            {
                if (node.LabelVisibleFromScale > 0 && node.LabelVisibleFromScale < scale)
                {
                    // already inserted before
                    continue;
                }

                LabelPlacement[] positions =
                {
                    LabelPlacement.Bottom,
                    LabelPlacement.Right,
                    LabelPlacement.Left,
                    LabelPlacement.Top
                };

                bool couldPlace = false;
                var labelRect = new Rectangle();
                foreach (LabelPlacement placement in positions)
                {
                    node.LabelPosition = placement;
                    labelRect = this.GetLabelRectForScale(node, scale);
                    if (!labelRTree.IsIntersecting(labelRect))
                    {
                        couldPlace = true;
                        break;
                    }
                }

                // uncomment: insert no labels until Johann Sebastian Bach can be labeled
                if (!couldPlace) {
                    break;
                }

                // finally, test if inserting would overlap too much important skipped nodes
                if (couldPlace)
                {
                    var overlappedSkipped = skippedRTree.GetAllIntersecting(labelRect);
                    if (overlappedSkipped.Any())
                    {
                        if (this.DoesOverlapTooManyImportant(node, overlappedSkipped))
                        {
                            couldPlace = false;
                        }
                    }                    
                }

                if (!couldPlace)
                {
                    node.LabelVisibleFromScale = 0;

                    skippedRTree.Add(labelRect, node);
                }
                else
                {
                    if (node.LabelVisibleFromScale <= 0) {
                        node.LabelVisibleFromScale = scale;
                    }

                    labelRTree.Add(labelRect, node);
                    labeledNodes++;
                }
            }

            return labeledNodes;
        }

        private bool DoesOverlapTooManyImportant(LgNodeInfo node, IEnumerable<LgNodeInfo> overlappedNodes)
        {
            // rank high == important

            double nodeWeight = node.Rank;
            double overlappedWeight = overlappedNodes.Sum(n => n.Rank);

            return nodeWeight * 20.0 < overlappedWeight;

        }

        private class RankComparer : IComparer<Node> {
            private readonly Dictionary<Node, LgNodeInfo> _table;

            public RankComparer(Dictionary<Node, LgNodeInfo> geometryNodesToLgNodeInfos)
            {
                this._table = geometryNodesToLgNodeInfos;
            }

            public int Compare(Node a, Node b)
            {
                return this._table[b].Rank.CompareTo(this._table[a].Rank);
            }
        }

        public bool RectIsEmptyStartingFromLevel(Rectangle tileBox, int iLevel)
        {
            for (int i = iLevel; i < this._lgData.Levels.Count; i++)
            {
                if (!this._lgData.Levels[i].RectIsEmptyOnLevel(tileBox)) {
                    return false;
                }
            }
            return true;
        }

        public int GetNumberOfLevels()
        {
            return this._lgData.Levels.Count;
        }

        private LgNodeInfo FindClosestNodeInfoForMouseClickOnLevel(Point mouseDownPositionInGraph, int iLevel)
        {
            if (iLevel >= this._lgData.Levels.Count) {
                return null;
            }

            var level = this._lgData.Levels[iLevel];
            var hitRectWidth = this.NodeDotWidth(this.CurrentZoomLevel) / 2;
            var rect = new Rectangle(new Size(hitRectWidth, hitRectWidth), mouseDownPositionInGraph);
            var intersected = level.NodeInfoTree.GetAllIntersecting(rect);
            if (!intersected.Any()) {
                return null;
            }

            double dist = double.PositiveInfinity;
            LgNodeInfo closest = null;
            foreach (var ni in intersected)
            {
                var t = (ni.Center - mouseDownPositionInGraph).LengthSquared;
                if (t < dist)
                {
                    dist = t;
                    closest = ni;
                }
            }
            return closest;
        }

        private LgNodeInfo FindClosestNodeInfoForMouseClickBelowCurrentLevel(Point mouseDownPositionInGraph)
        {
            var iLevel = this._lgData.GetLevelIndexByScale(this.CurrentZoomLevel);
            for (int i = iLevel + 1; i < this._lgData.LevelNodeCounts.Count; i++)
            {
                LgNodeInfo closest = this.FindClosestNodeInfoForMouseClickOnLevel(mouseDownPositionInGraph, i);
                if (closest != null) {
                    return closest;
                }
            }
            return null;
        }

        public void AnalyzeClick(Point mouseDownPositionInGraph, int downCount)
        {
            var closest = this.FindClosestNodeInfoForMouseClickBelowCurrentLevel(mouseDownPositionInGraph);
            if (closest == null) {
                return;
            }

            this.SelectAllEdgesIncidentTo(closest);
            var edges = closest.GeometryNode.Edges.ToList();
            this._railGraph.Edges.InsertRange(edges);
            closest.Selected = true;
            this._lgData.SelectedNodeInfos.Insert(closest);
            this._railGraph.Nodes.Insert(closest.GeometryNode);
            this.RunOnViewChange();
        }

        public Node AnalyzeClickForInvisibleNode(Point mouseDownPositionInGraph, int downCount)
        {
            var closest = this.FindClosestNodeInfoForMouseClickBelowCurrentLevel(mouseDownPositionInGraph);
            if (closest == null) {
                return null;
            }

            this.SelectAllEdgesIncidentTo(closest);
            var edges = closest.GeometryNode.Edges.ToList();
            this._railGraph.Edges.InsertRange(edges);
            closest.Selected = true;
            this._lgData.SelectedNodeInfos.Insert(closest);
            this._railGraph.Nodes.Insert(closest.GeometryNode);
            this.RunOnViewChange();
            return closest.GeometryNode;
        }

        public bool NumberOfNodesOfLastLayerIntersectedRectIsLessThanBound(int iLevel, Rectangle rect, int bound)
        {
            int lastLevel = this.GetNumberOfLevels() - 1;
            var level = this._lgData.Levels[lastLevel];
            var zoom = Math.Pow(2, iLevel);
            return level.NodeInfoTree.NumberOfIntersectedIsLessThanBound(rect, bound, node => node.ZoomLevel > zoom);
        }

        public bool TileIsEmpty(Rectangle rectangle)
        {
            int lastLevel = this.GetNumberOfLevels() - 1;
            var level = this._lgData.Levels[lastLevel];
            LgNodeInfo t;
            return !level.NodeInfoTree.OneIntersecting(rectangle, out t);
        }

        public IEnumerable<Node> GetTileNodes(Tuple<int, int, int> tile)
        {
            var grid = new GridTraversal(this._mainGeometryGraph.BoundingBox, tile.Item1);
            var rect = grid.GetTileRect(tile.Item2, tile.Item3);
            return
                this._lgData.Levels[this.GetNumberOfLevels() - 1].NodeInfoTree.GetAllIntersecting(rect)
                    .Select(nodeInfo => nodeInfo.GeometryNode);
        }
    }

}
