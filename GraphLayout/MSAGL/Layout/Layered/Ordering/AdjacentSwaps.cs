using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Following "A technique for Drawing Directed Graphs" of Gansner, Koutsofios, North and Vo
    /// Works on the layered graph. 
    /// For explanations of the algorithm here see https://www.researchgate.net/profile/Lev_Nachmanson/publication/30509007_Drawing_graphs_with_GLEE/links/54b6b2930cf2e68eb27edf71/Drawing-graphs-with-GLEE.pdf
    /// 
    /// </summary>
    internal partial class Ordering {
        /// <summary>
        /// for each vertex v let P[v] be the array of predeccessors of v
        /// </summary>
        private int[][] predecessors;


        /// <summary>
        /// The array contains a dictionary per vertex
        /// The value POrder[v][u] gives the offset of u in the array P[v]
        /// </summary>
        private Dictionary<int, int>[] pOrder;

        /// <summary>
        /// for each vertex v let S[v] be the array of successors of v
        /// </summary>
        private int[][] successors;

        /// <summary>
        /// The array contains a dictionary per vertex
        /// The value SOrder[v][u] gives the offset of u in the array S[v]
        /// </summary>
        private Dictionary<int, int>[] sOrder;
        private Dictionary<int, int>[] inCrossingCount;

        /// <summary>
        /// Gets or sets the number of of passes over all layers to rung adjacent exchanges, where every pass goes '
        /// all way up to the top layer and down to the lowest layer
        /// </summary>
        private const int MaxNumberOfAdjacentExchanges = 50;
        private Dictionary<int, int>[] outCrossingCount;

        private bool HeadOfTheCoin() {
            return this.random.Next(2) == 0;
        }

        private void AdjacentExchange() {
            this.InitArrays();
            int count = 0;
            bool progress = true;
            while (progress && count++ < MaxNumberOfAdjacentExchanges) {
                progress = false;
                for (int i = 0; i < this.layers.Length; i++) {
                    progress = this.AdjExchangeLayer(i) || progress;
                }

                for (int i = this.layers.Length - 2; i >= 0; i--) {
                    progress = this.AdjExchangeLayer(i) || progress;
                }
            }
        }

        private void AllocArrays() {
            int n = this.properLayeredGraph.NodeCount;
            this.predecessors = new int[n][];
            this.successors = new int[n][];


            this.pOrder = new Dictionary<int, int>[n];
            this.sOrder = new Dictionary<int, int>[n];
            if (this.hasCrossWeights) {
                this.outCrossingCount = new Dictionary<int, int>[n];
                this.inCrossingCount = new Dictionary<int, int>[n];
            }
            for (int i = 0; i < n; i++) {
                int count = this.properLayeredGraph.InEdgesCount(i);
                this.predecessors[i] = new int[count];
                if (this.hasCrossWeights) {
                    Dictionary<int, int> inCounts = this.inCrossingCount[i] = new Dictionary<int, int>(count);
                    foreach (LayerEdge le in this.properLayeredGraph.InEdges(i)) {
                        inCounts[le.Source] = le.CrossingWeight;
                    }
                }
                this.pOrder[i] = new Dictionary<int, int>(count);
                count = this.properLayeredGraph.OutEdgesCount(i);
                this.successors[i] = new int[count];
                this.sOrder[i] = new Dictionary<int, int>(count);
                if (this.hasCrossWeights) {
                    Dictionary<int, int> outCounts = this.outCrossingCount[i] = new Dictionary<int, int>(count);
                    foreach (LayerEdge le in this.properLayeredGraph.OutEdges(i)) {
                        outCounts[le.Target] = le.CrossingWeight;
                    }
                }
            }
        }

        /// <summary>
        /// Is called just after median layer swap is done
        /// </summary>
        private void InitArrays() {
            if (this.successors == null) {
                this.AllocArrays();
            }

            for (int i = 0; i < this.properLayeredGraph.NodeCount; i++) {
                this.pOrder[i].Clear();
                this.sOrder[i].Clear();
            }


            foreach (int[] t in this.layers) {
                this.InitPsArraysForLayer(t);
            }
        }


        /// <summary>
        /// calculates the number of intersections between edges adjacent to u and v
        /// </summary>
        /// <param name="u">a vertex</param>
        /// <param name="v">a vertex</param>
        /// <param name="cuv">the result when u is to the left of v</param>
        /// <param name="cvu">the result when v is to the left of u</param>
        private void CalcPair(int u, int v, out int cuv, out int cvu) {
            int[] su = this.successors[u], sv = this.successors[v], pu = this.predecessors[u], pv = this.predecessors[v];
            if (!this.hasCrossWeights) {
                cuv = this.CountOnArrays(su, sv) +
                      this.CountOnArrays(pu, pv);
                cvu = this.CountOnArrays(sv, su) +
                      this.CountOnArrays(pv, pu);
            } else {
                Dictionary<int, int> uOutCrossCounts = this.outCrossingCount[u];
                Dictionary<int, int> vOutCrossCounts = this.outCrossingCount[v];
                Dictionary<int, int> uInCrossCounts = this.inCrossingCount[u];
                Dictionary<int, int> vInCrossCounts = this.inCrossingCount[v];
                cuv = this.CountOnArrays(su, sv, uOutCrossCounts, vOutCrossCounts) +
                      this.CountOnArrays(pu, pv, uInCrossCounts, vInCrossCounts);
                cvu = this.CountOnArrays(sv, su, vOutCrossCounts, uOutCrossCounts) +
                      this.CountOnArrays(pv, pu, vInCrossCounts, uInCrossCounts);
            }
        }

        /// <summary>
        /// Sweep layer from left to right and fill S,P arrays as we go.
        /// The arrays P and S will be sorted according to X. Note that we will not keep them sorted
        /// as we doing adjacent swaps. Initial sorting only needed to calculate initial clr,crl values.
        /// </summary>
        /// <param name="layer"></param>
        private void InitPsArraysForLayer(int[] layer) {
            this.ProgressStep();

            foreach (int l in layer) {
                foreach (int p in this.properLayeredGraph.Pred(l)) {
                    Dictionary<int, int> so = this.sOrder[p];
                    int sHasNow = so.Count;
                    this.successors[p][sHasNow] = l; //l takes the first available slot in S[p]
                    so[l] = sHasNow;
                }
                foreach (int s in this.properLayeredGraph.Succ(l)) {
                    Dictionary<int, int> po = this.pOrder[s];
                    int pHasNow = po.Count;
                    this.predecessors[s][pHasNow] = l; //l take the first available slot in P[s]
                    po[l] = pHasNow;
                }
            }
        }

        private int CountOnArrays(int[] unbs, int[] vnbs) {
            int ret = 0;
            int vl = vnbs.Length - 1;
            int j = -1; //the right most position of vnbs to the left from the current u neighbor 
            int vnbsSeenAlready = 0;
            foreach (int uNeighbor in unbs) {
                int xu = this.X[uNeighbor];
                for (; j < vl && this.X[vnbs[j + 1]] < xu; j++) {
                    vnbsSeenAlready++;
                }

                ret += vnbsSeenAlready;
            }
            return ret;
        }


        /// <summary>
        /// every inversion between unbs and vnbs gives an intersecton
        /// </summary>
        /// <param name="unbs">neighbors of u but only from one layer</param>
        /// <param name="vnbs">neighbors of v from the same layers</param>
        /// <returns>number of intersections when u is to the left of v</returns>
        /// <param name="uCrossingCounts"></param>
        /// <param name="vCrossingCount"></param>
        private int CountOnArrays(int[] unbs, int[] vnbs, Dictionary<int, int> uCrossingCounts,
                          Dictionary<int, int> vCrossingCount) {
            int ret = 0;
            int vl = vnbs.Length - 1;
            int j = -1; //the right most position of vnbs to the left from the current u neighbor 

            int vCrossingNumberSeenAlready = 0;
            foreach (int uNeib in unbs) {
                int xu = this.X[uNeib];
                int vnb;
                for (; j < vl && this.X[vnb = vnbs[j + 1]] < xu; j++) {
                    vCrossingNumberSeenAlready += vCrossingCount[vnb];
                }

                ret += vCrossingNumberSeenAlready*uCrossingCounts[uNeib];
            }
            return ret;
        }

        private bool AdjExchangeLayer(int i) {
            this.ProgressStep();

            int[] layer = this.layers[i];
            bool gain = this.ExchangeWithGainWithNoDisturbance(layer);

            if (gain) {
                return true;
            }

            this.DisturbLayer(layer);

            return this.ExchangeWithGainWithNoDisturbance(layer);
        }

        //in this routine u and v are adjacent, and u is to the left of v before the swap
        private void Swap(int u, int v) {
            int left = this.X[u];
            int right = this.X[v];
            int ln = this.layering[u]; //layer number
            int[] layer = this.layers[ln];

            layer[left] = v;
            layer[right] = u;

            this.X[u] = right;
            this.X[v] = left;

            //update sorted arrays POrders and SOrders
            //an array should be updated only in case it contains both u and v.
            // More than that, v has to follow u in an the array.

            this.UpdateSsContainingUv(u, v);

            this.UpdatePsContainingUv(u, v);
        }

        private void UpdatePsContainingUv(int u, int v) {
            if (this.successors[u].Length <= this.successors[v].Length) {
                foreach (int a in this.successors[u]) {
                    Dictionary<int, int> porder = this.pOrder[a];
                    //of course porder contains u, let us see if it contains v
                    if (porder.ContainsKey(v)) {
                        int vOffset = porder[v];
                        //swap u and v in the array P[coeff]
                        int[] p = this.predecessors[a];
                        p[vOffset - 1] = v;
                        p[vOffset] = u;
                        //update sorder itself
                        porder[v] = vOffset - 1;
                        porder[u] = vOffset;
                    }
                }
            } else {
                foreach (int a in this.successors[v]) {
                    Dictionary<int, int> porder = this.pOrder[a];
                    //of course porder contains u, let us see if it contains v
                    if (porder.ContainsKey(u)) {
                        int vOffset = porder[v];
                        //swap u and v in the array P[coeff]
                        int[] p = this.predecessors[a];
                        p[vOffset - 1] = v;
                        p[vOffset] = u;
                        //update sorder itself
                        porder[v] = vOffset - 1;
                        porder[u] = vOffset;
                    }
                }
            }
        }

        private void UpdateSsContainingUv(int u, int v) {
            if (this.predecessors[u].Length <= this.predecessors[v].Length) {
                foreach (int a in this.predecessors[u]) {
                    Dictionary<int, int> sorder = this.sOrder[a];
                    //of course sorder contains u, let us see if it contains v
                    if (sorder.ContainsKey(v)) {
                        int vOffset = sorder[v];
                        //swap u and v in the array S[coeff]
                        int[] s = this.successors[a];
                        s[vOffset - 1] = v;
                        s[vOffset] = u;
                        //update sorder itself
                        sorder[v] = vOffset - 1;
                        sorder[u] = vOffset;
                    }
                }
            } else {
                foreach (int a in this.predecessors[v]) {
                    Dictionary<int, int> sorder = this.sOrder[a];
                    //of course sorder contains u, let us see if it contains v
                    if (sorder.ContainsKey(u)) {
                        int vOffset = sorder[v];
                        //swap u and v in the array S[coeff]
                        int[] s = this.successors[a];
                        s[vOffset - 1] = v;
                        s[vOffset] = u;
                        //update sorder itself
                        sorder[v] = vOffset - 1;
                        sorder[u] = vOffset;
                    }
                }
            }
        }

        private void DisturbLayer(int[] layer) {
            for (int i = 0; i < layer.Length - 1; i++) {
                this.AdjacentSwapToTheRight(layer, i);
            }
        }

        private bool ExchangeWithGainWithNoDisturbance(int[] layer) {
            bool wasGain = false;

            bool gain;
            do {
                gain = this.ExchangeWithGain(layer);
                wasGain = wasGain || gain;
            } while (gain);

            return wasGain;
        }

        private bool ExchangeWithGain(int[] layer) {
            //find a first pair giving some gain
            for (int i = 0; i < layer.Length - 1; i++) {
                if (this.SwapWithGain(layer[i], layer[i + 1])) {
                    this.SwapToTheLeft(layer, i);
                    this.SwapToTheRight(layer, i + 1);
                    return true;
                }
            }

            return false;
        }

        private void SwapToTheLeft(int[] layer, int i) {
            for (int j = i - 1; j >= 0; j--) {
                this.AdjacentSwapToTheRight(layer, j);
            }
        }

        private void SwapToTheRight(int[] layer, int i) {
            for (int j = i; j < layer.Length - 1; j++) {
                this.AdjacentSwapToTheRight(layer, j);
            }
        }

        /// <summary>
        /// swaps i-th element with i+1
        /// </summary>
        /// <param name="layer">the layer to work on</param>
        /// <param name="i">the position to start</param>
        private void AdjacentSwapToTheRight(int[] layer, int i) {
            int u = layer[i], v = layer[i + 1];

            int gain = this.SwapGain(u, v);

            if (gain > 0 || (gain == 0 && this.HeadOfTheCoin())) {
                this.Swap(u, v);
            }
        }

        private int SwapGain(int u, int v) {
            int cuv;
            int cvu;
            this.CalcPair(u, v, out cuv, out cvu);
            return cuv - cvu;
        }

        private bool UvAreOfSameKind(int u, int v) {
            return u < this.startOfVirtNodes && v < this.startOfVirtNodes || u >= this.startOfVirtNodes && v >= this.startOfVirtNodes;
        }

        private int SwapGroupGain(int u, int v) {
            int layerIndex = this.layerArrays.Y[u];
            int[] layer = this.layers[layerIndex];

            if (this.NeighborsForbidTheSwap(u, v)) {
                return -1;
            }

            int uPosition = this.X[u];
            bool uIsSeparator;
            if (this.IsOriginal(u)) {
                uIsSeparator = this.optimalOriginalGroupSize[layerIndex] == 1;
            } else {
                uIsSeparator = this.optimalVirtualGroupSize[layerIndex] == 1;
            }

            int delta = this.CalcDeltaBetweenGroupsToTheLeftAndToTheRightOfTheSeparator(layer,
                                                                                   uIsSeparator
                                                                                       ? uPosition
                                                                                       : uPosition + 1,
                                                                                   uIsSeparator ? u : v);

            if (uIsSeparator) {
                if (delta < -1) {
                    return 1;
                }

                if (delta == -1) {
                    return 0;
                }

                return -1;
            }
            if (delta > 1) {
                return 1;
            }

            if (delta == 1) {
                return 0;
            }

            return -1;
        }

        private bool NeighborsForbidTheSwap(int u, int v) {
            return this.UpperNeighborsForbidTheSwap(u, v) || this.LowerNeighborsForbidTheSwap(u, v);
        }

        private bool LowerNeighborsForbidTheSwap(int u, int v) {
            int uCount, vCount;
            if (((uCount = this.properLayeredGraph.OutEdgesCount(u)) == 0) ||
                ((vCount = this.properLayeredGraph.OutEdgesCount(v)) == 0)) {
                return false;
            }

            return this.X[this.successors[u][uCount >> 1]] < this.X[this.successors[v][vCount >> 1]];
        }

        private bool UpperNeighborsForbidTheSwap(int u, int v) {
            int uCount = this.properLayeredGraph.InEdgesCount(u);
            int vCount = this.properLayeredGraph.InEdgesCount(v);
            if (uCount == 0 || vCount == 0) {
                return false;
            }

            return this.X[this.predecessors[u][uCount >> 1]] < this.X[this.predecessors[v][vCount >> 1]];
        }

        private int CalcDeltaBetweenGroupsToTheLeftAndToTheRightOfTheSeparator(int[] layer, int separatorPosition, int separator) {
            Func<int, bool> kind = this.GetKindDelegate(separator);
            int leftGroupSize = 0;
            for (int i = separatorPosition - 1; i >= 0 && !kind(layer[i]); i--) {
                leftGroupSize++;
            }

            int rightGroupSize = 0;
            for (int i = separatorPosition + 1; i < layer.Length && !kind(layer[i]); i++) {
                rightGroupSize++;
            }

            return leftGroupSize - rightGroupSize;
        }

        private bool IsOriginal(int v) {
            return v < this.startOfVirtNodes;
        }

        private bool IsVirtual(int v) {
            return v >= this.startOfVirtNodes;
        }

        private Func<int, bool> GetKindDelegate(int v) {
            Func<int, bool> kind = this.IsVirtual(v) ? this.IsVirtual : new Func<int, bool>(this.IsOriginal);
            return kind;
        }


        ///// <summary>
        ///// swaps two vertices only if reduces the number of intersections
        ///// </summary>
        ///// <param name="layer">the layer to work on</param>
        ///// <param name="u">left vertex</param>
        ///// <param name="v">right vertex</param>
        ///// <returns></returns>
        private bool SwapWithGain(int u, int v) {
            int gain = this.SwapGain(u, v);

            if (gain > 0) {
                this.Swap(u, v);
                return true;
            }
            return false;
        }
    }
}