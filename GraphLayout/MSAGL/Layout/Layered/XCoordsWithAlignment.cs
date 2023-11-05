using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.GraphAlgorithms;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// The implementation follows "Fast and Simple Horizontal Coordinate Assignment" of Ulrik Brandes and Boris K¨opf
    /// The paper has two serious bugs that this code resolves.
    /// </summary>
    internal partial class XCoordsWithAlignment {
        private LayerArrays la;
        private ProperLayeredGraph graph;
        private int nOfOriginalVertices;
        private int[] root;
        private int[] align;

        //int[] sink;
        //double[] shift;
        private int nOfVertices;
        private Anchor[] anchors;
        private double nodeSep;
        private object[] lowMedians;
        private object[] upperMedians; //each element or int or IntPair
        private Set<IntPair> markedEdges = new Set<IntPair>();
        private int h;//number of layers

        //We pretend, when calculating the alignment, that we traverse layers from left to right and from bottom to top.
        //The actual directions are defined by variables "LR" and "BT". 

        /// <summary>
        /// from left to right
        /// </summary>
        private bool LR;

        /// <summary>
        /// from bottom to top
        /// </summary>
        private bool BT;

        private int EnumRightUp { get { return (this.LR ? 0 : 1) + 2 * (this.BT ? 0 : 1); } }


        /// <summary>
        /// Returns true if v is a virtual vertex
        /// </summary>
        /// <param name="v"></param>GG
        /// <returns></returns>
        private bool IsVirtual(int v) { return v >= this.nOfOriginalVertices; }


        //four arrays for four different direction combinations
        private double[][] xCoords = new double[4][];
        private double[] x;

        private int Source(LayerEdge edge) { return this.BT ? edge.Source : edge.Target; }

        private int Target(LayerEdge edge) { return this.BT ? edge.Target : edge.Source; }

        /// <summary>
        /// 
        /// </summary>
        static internal void CalculateXCoordinates(LayerArrays layerArrays, ProperLayeredGraph layeredGraph, int nOfOriginalVs, Anchor[] anchors, double nodeSeparation) {
            XCoordsWithAlignment x = new XCoordsWithAlignment(layerArrays, layeredGraph, nOfOriginalVs, anchors, nodeSeparation);
            x.Calculate();
        }

        private void Calculate() {
            this.SortInAndOutEdges();

            this.RightUpSetup();
            this.CalcBiasedAlignment();

            this.LeftUpSetup();
            this.CalcBiasedAlignment();

            this.RightDownSetup();
            this.CalcBiasedAlignment();

            this.LeftDownSetup();
            this.CalcBiasedAlignment();
            this.HorizontalBalancing();

        }

        //We need to find a median of a vertex neighbors from a specific layer. That is, if we have a vertex v and edges (v,coeff), (v,side1), (v,cornerC) 
        // going down, and X[coeff]<X[side1]<X[cornerC], then side1 is the median.
        //There is an algorithm that finds the median with expected linear number of steps,
        //see for example http://www.ics.uci.edu/~eppstein/161/960125.html. However, I think we are better off 
        //with sorting, since we are taking median at least twice. 
        //Notice, that the sorting should be done only for original vertices since dummy vertices 
        //have only one incoming edge and one outcoming edge.
        //Consider here reusing the sorting that comes from the ordering step,
        //if it is not broken by layer insertions.
        private void SortInAndOutEdges() {
            this.FillLowMedians();
            this.FillUpperMedins();
            //Microsoft.Msagl.Ordering.EdgeComparerBySource edgeComparerBySource = new Ordering.EdgeComparerBySource(this.la.X);
            //Microsoft.Msagl.Ordering.EdgeComparerByTarget edgeComparerByTarget = new Ordering.EdgeComparerByTarget(this.la.X);
            //for (int i = 0; i < this.nOfOriginalVertices; i++) {
            //    Array.Sort<LayerEdge>(this.graph.InEdges(i) as LayerEdge[], edgeComparerBySource);
            //    Array.Sort<LayerEdge>(this.graph.OutEdges(i) as LayerEdge[], edgeComparerByTarget);
            //}
        }

        private void FillUpperMedins() {
            this.upperMedians = new object[this.graph.NodeCount];
            for (int i = 0; i < this.graph.NodeCount; i++) {
                this.FillUpperMediansForNode(i);
            }
        }

        private int CompareByX(int a, int b) { return this.la.X[a] - this.la.X[b]; }

        private void FillUpperMediansForNode(int i) {
            int count = this.graph.InEdgesCount(i);
            if (count > 0) {
                int[] predecessors = new int[count];
                count = 0;
                foreach (LayerEdge e in this.graph.InEdges(i)) {
                    predecessors[count++] = e.Source;
                }

                Array.Sort(predecessors, new System.Comparison<int>(this.CompareByX));
                int m = count / 2;
                if (m * 2 == count) {
                    this.upperMedians[i] = new IntPair(predecessors[m - 1], predecessors[m]);
                } else {
                    this.upperMedians[i] = predecessors[m];
                }
            } else {
                this.upperMedians[i] = -1;
            }
        }

        private void FillLowMedians() {
            this.lowMedians = new object[this.graph.NodeCount];
            for (int i = 0; i < this.graph.NodeCount; i++) {
                this.FillLowMediansForNode(i);
            }
        }

        private void FillLowMediansForNode(int i) {
            int count = this.graph.OutEdgesCount(i);
            if (count > 0) {
                int[] successors = new int[count];
                count = 0;
                foreach (LayerEdge e in this.graph.OutEdges(i)) {
                    successors[count++] = e.Target;
                }

                Array.Sort(successors, new System.Comparison<int>(this.CompareByX));
                int m = count / 2;
                if (m * 2 == count) {
                    this.lowMedians[i] = new IntPair(successors[m - 1], successors[m]);
                } else {
                    this.lowMedians[i] = successors[m];
                }
            } else {
                this.lowMedians[i] = -1;
            }
        }

        private void HorizontalBalancing() {

            int leastWidthAssignment = -1;
            double[] a = new double[4];
            double[] b = new double[4];

            double leastWidth = Double.MaxValue;
            for (int i = 0; i < 4; i++) {
                this.AssignmentBounds(i, out a[i], out b[i]);
                double w = b[i] - a[i];
                if (w < leastWidth) {
                    leastWidthAssignment = i;
                    leastWidth = w;
                }
            }

            for (int i = 0; i < 4; i++) {
                double delta;
                if (IsLeftMostAssignment(i)) {
                    //need to align left ends according to the paper
                    delta = a[leastWidthAssignment] - a[i];
                } else {
                    delta = b[leastWidthAssignment] - b[i];
                }

                this.x = this.xCoords[i];
                if (delta != 0) {
                    for (int j = 0; j < this.nOfVertices; j++) {
                        this.x[j] += delta;
                    }
                }
            }



            double[] arr = new double[4];
            for (int v = 0; v < this.nOfVertices; v++) {
                arr[0] = this.xCoords[0][v];
                arr[1] = this.xCoords[1][v];
                arr[2] = this.xCoords[2][v];
                arr[3] = this.xCoords[3][v];
                Array.Sort(arr);
                this.anchors[v].X = (arr[1] + arr[2]) / 2;
            }

            //    Layout.ShowDataBase(dataBase);

        }

        private static bool IsLeftMostAssignment(int i) {
            return i == 0 || i == 2;
        }

        private void AssignmentBounds(int i, out double a, out double b) {
            if (this.nOfVertices == 0) {
                a = 0;
                b = 0;
            } else {
                this.x = this.xCoords[i];
                a = b = this.x[0];
                for (int j = 1; j < this.nOfVertices; j++) {
                    double r = this.x[j];
                    if (r < a) {
                        a = r;
                    } else if (r > b) {
                        b = r;
                    }
                }
            }
        }

        private void CalcBiasedAlignment() {
            this.ConflictElimination();
            this.Align();
#if TEST_MSAGL
            //for (int i = 0; i < nOfVertices; i++)
            //    anchors[i].X = x[i];
            //Layout.ShowDataBase(dataBase);
#endif
        }

        private void LeftUpSetup() {
            this.LR = false;
            this.BT = true;
        }

        private void LeftDownSetup() {
            this.LR = false;
            this.BT = false;
        }

        private void RightDownSetup() {
            this.LR = true;
            this.BT = false;
        }

        private void RightUpSetup() {
            this.LR = true;
            this.BT = true;
        }


        /// <summary>
        /// The code is written as if we go left up, but in fact the settings define the directions.
        /// 
        /// We need to create a subgraph for alignment:
        /// where no edge segments intersect, and every vertex has
        /// at most one incoming and at most one outcoming edge.
        /// This function marks edges to resolve conflicts with only one inner segment.  
        /// An inner segment is a segment between two dummy nodes.
        /// We mark edges that later will not participate in the alignment. 
        /// Inner segments are preferred to other ones. So, in a conflict with one inner and one
        /// non-inner edges we leave the inner edge to participate in the alignment. 
        /// At the moment we mark as not participating both of the two intersecting inner segments
        /// </summary>
        private void ConflictElimination() {
            this.RemoveMarksFromEdges();
            this.MarkConflictingEdges();
        }

        /*
         * Type 0 conflicts are those where inner edges do not participate. 
         * They are resolved not by marking but just when we calculate the alignment in CreateBlocks.
         * A quote from "Fast and ..." with some typo corrections:
         * Type 0 conflicts are resolved greedily in a leftmost fashion, 
         * i.e. in every layer we process the vertices from left to right and 
         * for each vertex we consider its median upper neighbor (its left and right 
         * median upper neighbor, in this order, if there are two). 
         * The pair is aligned, if no conflicting alignment is to the left of this one.
         * The resulting bias is mediated by the fact that the symmetric bias is applied 
         * in one of the other three assignments.
         */




        private IEnumerable<int> UpperEdgeMedians(int target) {
            object medians = this.BT ? this.upperMedians[target] : this.lowMedians[target];
            if (medians is IntPair) {
                IntPair ip = medians as IntPair;
                if (this.LR) {
                    yield return ip.First; yield return ip.Second;
                } else {
                    yield return ip.Second; yield return ip.First;
                }
            } else {
                int i = (int)medians;
                if (i >= 0) {
                    yield return i;
                }
            }
        }



        /// <summary>
        /// here we eliminate all constraints 
        /// </summary>
        private void MarkConflictingEdges() {
            int i = this.LowerOf(0, this.h - 1);
            int lowest = i;
            int upperBound = this.UpperOf(0, this.h - 1);
            int nextBelowUpperBound = this.NextLower(upperBound);

            //our top layer has index h-1, our bottom layer has index 0
            //inner segments can appear only between layers with indices i+1 and i where i>0 and i<h-1
            for (; this.IsBelow(i, upperBound); i = this.NextUpper(i)) {
                if (this.IsBelow(lowest, i) && this.IsBelow(i, nextBelowUpperBound)) {
                    this.ConflictsWithAtLeastOneInnerEdgeForALayer(i);
                }
            }
        }




        /// <summary>
        /// parameterized next upper 
        /// </summary>
        /// <param name="i"></param>
        /// <returns></returns>
        private int NextUpper(int i) { return this.BT ? i + 1 : i - 1; }

        /// <summary>
        /// parameterized next lower
        /// </summary>
        /// <param name="i"></param>
        /// <returns></returns>
        private int NextLower(int i) { return this.BT ? i - 1 : i + 1; }

        /// <summary>
        /// parameterize highest of two numbers
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        private int UpperOf(int i, int j) { return this.BT ? Math.Max(i, j) : Math.Min(i, j); }

        /// <summary>
        /// parameterized lowest of a pair
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        private int LowerOf(int i, int j) { return this.BT ? Math.Min(i, j) : Math.Max(i, j); }


        /// <summary>
        /// returns parameterized below
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        private bool IsBelow(int i, int j) { return this.BT ? i < j : j < i; }

        /// <summary>
        /// returns the "parameterized" left of the two positions
        /// </summary>
        /// <param name="pos0"></param>
        /// <param name="pos1"></param>
        /// <returns></returns>
        private int LeftMost(int pos0, int pos1) { return this.LR ? Math.Min(pos0, pos1) : Math.Max(pos0, pos1); }

        private double LeftMost(double pos0, double pos1) { return this.LR ? Math.Min(pos0, pos1) : Math.Max(pos0, pos1); }

        /// <summary>
        /// returns the "parameterized" right of the two positions
        /// </summary>
        /// <param name="pos0"></param>
        /// <param name="pos1"></param>
        /// <returns></returns>
        private int RightMost(int pos0, int pos1) { return this.LR ? Math.Max(pos0, pos1) : Math.Min(pos0, pos1); }

        /// <summary>
        /// returns the "parameterized" right of the two positions
        /// </summary>
        /// <param name="pos0"></param>
        /// <param name="pos1"></param>
        /// <returns></returns>
        private double RightMost(double pos0, double pos1) { return this.LR ? Math.Max(pos0, pos1) : Math.Min(pos0, pos1); }

        /// <summary>
        /// Return true if i is to the left or equal to pos in a "parameterized" fasion
        /// </summary>
        /// <param name="i"></param>
        /// <param name="pos"></param>
        /// <returns></returns>
        private bool IsNotRightFrom(int i, int pos) { return this.LR ? i <= pos : pos <= i; }

        /// <summary>
        /// Parameterized left relation
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns></returns>
        private bool IsLeftFrom(int i, int j) { return this.LR ? i < j : j < i; }

        /// <summary>
        /// parameterized next right
        /// </summary>
        /// <param name="i"></param>
        /// <returns></returns>
        private int NextRight(int i) { return this.LR ? i + 1 : i - 1; }

        /// <summary>
        /// parameterized next left
        /// </summary>
        /// <param name="i"></param>
        /// <returns></returns>
        private int NextLeft(int i) { return this.LR ? i - 1 : i + 1; }

        ///// <summary>
        ///// Eliminates conflicts with at least one inner edge inside of one layer
        ///// </summary>
        ///// <param name="i"></param>
        private void ConflictsWithAtLeastOneInnerEdgeForALayer(int layerIndex) {
            if (layerIndex >= 0 && layerIndex < this.la.Layers.Length) {
                int[] lowerLayer = this.la.Layers[layerIndex];
                LayerEdge innerEdge = null;

                //start looking for the first inner edge from the left of lowerLayer
                int targetPos = this.LeftMost(0, lowerLayer.Length - 1);
                int lastTargetPos = this.RightMost(0, lowerLayer.Length - 1); ;

                for (; this.IsNotRightFrom(targetPos, lastTargetPos) && innerEdge == null;
                  targetPos = this.NextRight(targetPos)) {
                    innerEdge = this.InnerEdgeByTarget(lowerLayer[targetPos]);
                }

                //now targetPos points to the right of the innerEdge target at lowerLayer
                if (innerEdge != null) {
                    int positionOfInnerEdgeSource = this.Pos(this.Source(innerEdge));
                    //We are still not in the main loop.
                    //We mark conflicting edges with targets to the left of targetPos,
                    //That of course means 
                    //that the sources of conflicting edges lie to the right of Source(innerEdge)
                    for (int j = this.LeftMost(0, lowerLayer.Length - 1);
                      this.IsLeftFrom(j, targetPos);
                      j = this.NextRight(j)) {
                        foreach (LayerEdge ie in this.InEdges(lowerLayer[j])) {
                            if (this.IsLeftFrom(positionOfInnerEdgeSource, this.Pos(this.Source(ie)))) {
                                this.MarkEdge(ie);
                            }
                        }
                    }

                    int innerSourcePos = this.Pos(this.Source(innerEdge));
                    //starting the main loop
                    while (this.IsNotRightFrom(targetPos, lastTargetPos)) {
                        //Now we look for the next inner edge in the alignment to the right of the current innerEdge,
                        //and we mark the conflicts later. Marking the conflicts later makes sense. 
                        //We would have to go through positions between innerEdge and newInnerEdge targets 
                        //again anyway to resolve conflicts with not inner edges and newInnerEdge
                        LayerEdge newInnerEdge = this.AlignmentToTheRightOfInner(lowerLayer,
                            targetPos, positionOfInnerEdgeSource);

                        targetPos = this.NextRight(targetPos);
                        if (newInnerEdge != null) {
                            int newInnerSourcePos = this.Pos(this.Source(newInnerEdge));
                            this.MarkEdgesBetweenInnerAndNewInnerEdges(lowerLayer, innerEdge,
                              newInnerEdge, innerSourcePos, newInnerSourcePos);
                            innerEdge = newInnerEdge;
                            innerSourcePos = newInnerSourcePos;
                        }
                    }

                    //look for conflicting edges with targets to the right from the target of innerEdge
                    for (int k = this.NextRight(this.Pos(this.Target(innerEdge))); this.IsNotRightFrom(k, lastTargetPos); k = this.NextRight(k)) {

                        foreach (LayerEdge ie in this.InEdges(lowerLayer[k])) {
                            if (this.IsLeftFrom(this.Pos(this.Source(ie)), this.Pos(this.Source(innerEdge)))) {
                                this.MarkEdge(ie);
                            }
                        }
                    }
                }
            }
        }

        private LayerEdge InEdgeOfVirtualNode(int v) { return (this.BT ? this.graph.InEdgeOfVirtualNode(v) : this.graph.OutEdgeOfVirtualNode(v)); }

        private IEnumerable<LayerEdge> InEdges(int v) { return this.BT ? this.graph.InEdges(v) : this.graph.OutEdges(v); }

        ///// <summary>
        ///// This function marks conflicting edges with targets positioned between innerEdge and newInnerEdge targets.
        ///// </summary>
        ///// <param name="lowerLayer"></param>
        ///// <param name="innerEdge"></param>
        ///// <param name="newInnerEdge"></param>
        ///// <param name="posInnerEdgeTarget"></param>
        ///// <param name="posNewInnerEdgeTarget"></param>
        private void MarkEdgesBetweenInnerAndNewInnerEdges(int[] lowerLayer, LayerEdge innerEdge, LayerEdge newInnerEdge,
          int innerEdgeSourcePos, int newInnerEdgeSourcePos) {
            int u = this.NextRight(this.Pos(this.Target(innerEdge)));


            for (; this.IsLeftFrom(u, this.Pos(this.Target(newInnerEdge))); u = this.NextRight(u)) {
                foreach (LayerEdge ie in this.InEdges(lowerLayer[u])) {
                    int ieSourcePos = this.Pos(this.Source(ie));
                    if (this.IsLeftFrom(ieSourcePos, innerEdgeSourcePos))//the equality is not possible
{
                        this.MarkEdge(ie);
                    } else if (this.IsLeftFrom(newInnerEdgeSourcePos, ieSourcePos)) {
                        this.MarkEdge(ie);
                    }
                }
            }
        }
        ///// <summary>
        ///// Returns the inner non-conflicting edge incoming into i-th position 
        ///// of the layer or null if there is no such edge
        ///// </summary>
        ///// <param name="layer"></param>
        ///// <param name="innerEdge"></param>
        ///// <param name="i"></param>
        ///// <returns></returns>
        private LayerEdge AlignmentToTheRightOfInner(int[] lowLayer,
          int i,
          int posInnerSource
          ) {
            int numOfInEdges = this.NumberOfInEdges(lowLayer[i]);
            if(numOfInEdges==1){

                LayerEdge ie = null;
                foreach(LayerEdge e in this.InEdges(lowLayer[i])) {
                    ie =e;
                }

                if (this.IsInnerEdge(ie) && this.IsLeftFrom(posInnerSource, this.Pos(ie.Source))) {
                    return ie;
                }

                return null;
            }

            return null;
        }

        private int NumberOfInEdges(int v) {
            return this.BT ? this.graph.InEdgesCount(v) : this.graph.OutEdgesCount(v);
        }

        private int Pos(int v) {
            return this.la.X[v];
        }

        private LayerEdge InnerEdgeByTarget(int v) {
            if (this.IsVirtual(v)) {
                LayerEdge ie = this.InEdgeOfVirtualNode(v);//there is exactly one edge entering in to the dummy node
                if (this.IsVirtual(this.Source(ie))) {
                    return ie;
                }
            }
            return null;
        }

        private bool IsInnerEdge(LayerEdge e) {
            return this.IsVirtual(e.Source) && this.IsVirtual(e.Target);
        }

        private void RemoveMarksFromEdges() {
            this.markedEdges.Clear();
        }

        ///// <summary>
        ///// private constructor
        ///// </summary>
        ///// <param name="layerArrays"></param>
        ///// <param name="anchs"></param>
        ///// <param name="layeredGraph"></param>
        ///// <param name="nOfOriginalVs"></param>
        private XCoordsWithAlignment(LayerArrays layerArrays, ProperLayeredGraph layeredGraph, 
            int nOfOriginalVs, Anchor[] anchorsP, double ns) {
            this.la = layerArrays;
            this.graph = layeredGraph;
            this.nOfOriginalVertices = nOfOriginalVs;
            this.nOfVertices = this.graph.NodeCount;
            this.h = this.la.Layers.Length;
            this.root = new int[this.nOfVertices];
            this.align = new int[this.nOfVertices];
            // this.sink = new int[nOfVertices];
            // this.shift = new double[nOfVertices];
            this.anchors = anchorsP;
            this.nodeSep = ns;
        }

        /// <summary>
        ///Calculate the alignment based on the marked edges and greedily resolving the remaining conflicts on the fly, without marking
        /// </summary>
        private void Align() {
            this.CreateBlocks();
            this.AssignCoordinatesByLongestPath();
        }

        private void AssignCoordinatesByLongestPath() {
            this.x = this.xCoords[this.EnumRightUp] = new double[this.nOfVertices];
            /*
             * We create a graph of blocks or rather of block roots. There is an edge
             * from u-block to v-block  if some of elements of u-block is to the left of v 
             * on the same layer. Then we topologically sort the graph and assign coordinates 
             * taking into account separation between the blocks.
             */
            //create the graph first
            List<PolyIntEdge> edges = new List<PolyIntEdge>();
            for (int v = 0; v < this.nOfVertices; v++) {
                if (v == this.root[v])//v is a root
                {
                    int w = v;//w will be running over the block
                    do {
                        int rightNeighbor;
                        if (this.TryToGetRightNeighbor(w, out rightNeighbor)) {
                            edges.Add(new PolyIntEdge(v, this.root[rightNeighbor]));
                        }

                        w = this.align[w];
                    }
                    while (w != v);
                }
            }

            BasicGraphOnEdges<PolyIntEdge> blockGraph = new BasicGraphOnEdges<PolyIntEdge>(edges, this.nOfVertices);
            //sort the graph in the topological order
            int[] topoSort = PolyIntEdge.GetOrder(blockGraph);
            //start placing the blocks according to the order

            foreach (int v in topoSort) {
                if (v == this.root[v])//not every element of topoSort is a root!
                {
                    double vx = 0;
                    bool vIsLeftMost = true;
                    int w = v;//w is running over the block
                    do {
                        int wLeftNeighbor;
                        if (this.TryToGetLeftNeighbor(w, out wLeftNeighbor)) {
                            if (vIsLeftMost) {
                                vx = this.x[this.root[wLeftNeighbor]] + this.DeltaBetweenVertices(wLeftNeighbor, w);
                                vIsLeftMost = false;
                            } else {
                                vx = this.RightMost(vx, this.x[this.root[wLeftNeighbor]] + this.DeltaBetweenVertices(wLeftNeighbor, w));
                            }
                        }
                        w = this.align[w];
                    }
                    while (w != v);

                    this.x[v] = vx;
                }
            }

            //push the roots of the graph maximally to the right 
            foreach (int v in topoSort) {
                if (v == this.root[v]) {
                    if (blockGraph.InEdges(v).Count == 0) {
                        int w = v;//w runs over the block
                        double xLeftMost = this.RightMost(-infinity, infinity);
                        double xl = xLeftMost;
                        do {
                            int wRightNeigbor;
                            if (this.TryToGetRightNeighbor(w, out wRightNeigbor)) {
                                xLeftMost = this.LeftMost(xLeftMost,
                                    this.x[this.root[wRightNeigbor]] - this.DeltaBetweenVertices(w, wRightNeigbor));
                            }

                            w = this.align[w];
                        } while (w != v);

                        //leave the value zero if there are no right neighbours
                        if (xl != xLeftMost) {
                            this.x[v] = xLeftMost;
                        }
                    }
                }
            }

            for (int v = 0; v < this.nOfVertices; v++) {
                if (v != this.root[v]) {
                    this.x[v] = this.x[this.root[v]];
                }
            }

        }

        /// <summary>
        /// returns true is u has a right neighbor on its layer
        /// </summary>
        /// <param name="u"></param>
        /// <param name="neighbor"></param>
        /// <returns></returns>
        private bool TryToGetRightNeighbor(int u, out int neighbor) {
            int neighborPos = this.NextRight(this.Pos(u));
            int[] layer = this.la.Layers[this.la.Y[u]];
            if (neighborPos >= 0 && neighborPos < layer.Length) {
                neighbor = layer[neighborPos];
                return true;
            } else {
                neighbor = 0;
                return false;
            }
        }

        /// <summary>
        /// returns true is u has a right neighbor on its layer
        /// </summary>
        /// <param name="u"></param>
        /// <param name="neighbor"></param>
        /// <returns></returns>
        private bool TryToGetLeftNeighbor(int u, out int neighbor) {
            int neighborPos = this.NextLeft(this.Pos(u));
            int[] layer = this.la.Layers[this.la.Y[u]];
            if (neighborPos >= 0 && neighborPos < layer.Length) {
                neighbor = layer[neighborPos];
                return true;
            } else {
                neighbor = 0;
                return false;
            }
        }


        /// <summary>
        /// Organizes the vertices into blocks. A block is a maximal path in the alignment subgraph. 
        /// The alignment is defined by array align. Every vertex is connected to the top vertex of 
        /// the block by using root array. The alignment is cyclic. If we start from a root vertex v and 
        /// apply align then we return to v at some point.
        /// </summary>
        private void CreateBlocks() {

            for (int v = 0; v < this.nOfVertices; v++) {
                this.root[v] = this.align[v] = v;
            }

            int lowBound = this.LowerOf(0, this.h - 1);

            //i points to the last layer before the highest one

            for (int i = this.NextLower(this.UpperOf(0, this.h - 1)); !this.IsBelow(i, lowBound); i = this.NextLower(i)) {
                int[] layer = this.la.Layers[i];

                int r = this.LeftMost(-1, this.la.Layers[this.NextUpper(i)].Length);
                //We align vertices of the layer above the i-th one only if their positions are
                //to the right of r. This moves us forward on the layer above the current and resolves the conflicts.

                int rightBound = this.RightMost(0, layer.Length - 1);
                for (int k = this.LeftMost(0, layer.Length - 1);
                    this.IsNotRightFrom(k, rightBound); k = this.NextRight(k)) {
                    int vk = layer[k];
                    foreach (int upperNeighborOfVk in this.UpperEdgeMedians(vk)) {
                        if (!this.IsMarked(vk, upperNeighborOfVk)) {
                            if (this.IsLeftFrom(r, this.Pos(upperNeighborOfVk))) {
                                this.align[upperNeighborOfVk] = vk;
                                this.align[vk] = this.root[vk] = this.root[upperNeighborOfVk];
                                r = this.Pos(upperNeighborOfVk);
                                break;// done with the alignement for vk
                            }
                        }
                    }
                }
            }
        }
        

        private bool IsMarked(int source, int target) {
            if (this.BT) {
                return this.markedEdges.Contains(new IntPair(target, source));
            } else {
                return this.markedEdges.Contains(new IntPair(source, target));
            }
        }

        private void MarkEdge(LayerEdge ie) {
            this.markedEdges.Insert(new IntPair(ie.Source, ie.Target));
        }

        /// <summary>
        /// Assigning xcoords starting from roots
        /// </summary>

        private const double infinity = Double.MaxValue;

        /// <summary>
        /// Calculates the minimum separation between two neighboring vertices: if u is to the left of v on the same layer return positive
        /// number, otherwise negative.
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        private double DeltaBetweenVertices(int u, int v) {
            int sign = 1;
            if (this.Pos(u) > this.Pos(v)) { //swap u and v
                int t = u;
                u = v;
                v = t;
                sign = -1;
            }

            double anchorSepar = this.anchors[u].RightAnchor + this.anchors[v].LeftAnchor;
            return (anchorSepar + this.nodeSep) * sign;

        }

    }
}
