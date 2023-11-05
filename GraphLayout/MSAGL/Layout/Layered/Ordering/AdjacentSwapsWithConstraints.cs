using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Microsoft.Msagl.Core.DataStructures;
using System.Diagnostics;

namespace Microsoft.Msagl.Layout.Layered {
    internal class AdjacentSwapsWithConstraints {
        private const int maxNumberOfAdjacentExchanges = 50;
        private readonly bool hasCrossWeights;
        private readonly LayerInfo[] layerInfos;
        private readonly int[] layering;
        private readonly int[][] layers;
        private readonly ProperLayeredGraph properLayeredGraph;
        private readonly Random random = new Random(1);
        private readonly int[] X;
        private Dictionary<int, int>[] inCrossingCount;
        private Dictionary<int, int>[] outCrossingCount;

        /// <summary>
        /// for each vertex v let P[v] be the array of predeccessors of v
        /// </summary>
        private List<int>[] P;


        /// <summary>
        /// The array contains a dictionary per vertex
        /// The value POrder[v][u] gives the offset of u in the array P[v]
        /// </summary>
        private Dictionary<int, int>[] POrder;

        /// <summary>
        /// for each vertex v let S[v] be the array of successors of v
        /// </summary>
        private List<int>[] S;

        /// <summary>
        /// The array contains a dictionary per vertex
        /// The value SOrder[v][u] gives the offset of u in the array S[v]
        /// </summary>
        private Dictionary<int, int>[] SOrder;

        internal AdjacentSwapsWithConstraints(LayerArrays layerArray,
                                              bool hasCrossWeights,
                                              ProperLayeredGraph properLayeredGraph,
                                              LayerInfo[] layerInfos) {
            this.X = layerArray.X;
            this.layering = layerArray.Y;
            this.layers = layerArray.Layers;
            this.properLayeredGraph = properLayeredGraph;
            this.hasCrossWeights = hasCrossWeights;
            this.layerInfos = layerInfos;
        }

        /// <summary>
        /// Gets or sets the number of of passes over all layers to run
        /// adjacent exchanges, where every pass goes
        /// all way up to the top layer and down to the lowest layer
        /// </summary>
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static int MaxNumberOfAdjacentExchanges {
            get { return maxNumberOfAdjacentExchanges; }
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

        private bool CanSwap(int i, int j) {
            if (this.IsVirtualNode(i) || this.IsVirtualNode(j)) {
                return true;
            }

            LayerInfo layerInfo = this.layerInfos[this.layering[i]];
            if (layerInfo == null) {
                return true;
            }

            if (ConstrainedOrdering.BelongsToNeighbBlock(i, layerInfo)
                ||
                ConstrainedOrdering.BelongsToNeighbBlock(j, layerInfo)
                ||
                layerInfo.constrainedFromAbove.ContainsKey(i)
                ||
                layerInfo.constrainedFromBelow.ContainsKey(j)
                ) {
                return false;
            }

            if (layerInfo.leftRight.Contains(new Tuple<int, int>(i, j))) {
                return false;
            }

            return true;
        }

        private bool IsVirtualNode(int v) {
            return this.properLayeredGraph.IsVirtualNode(v);
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

        private int SwapGain(int u, int v) {
            if (!this.CanSwap(u, v)) {
                return -1;
            }

            int cuv;
            int cvu;
            this.CalcPair(u, v, out cuv, out cvu);
            return cuv - cvu;
        }

        /// <summary>
        /// calculates the number of intersections between edges adjacent to u and v
        /// </summary>
        /// <param name="u">a vertex</param>
        /// <param name="v">a vertex</param>
        /// <param name="cuv">the result when u is to the left of v</param>
        /// <param name="cvu">the result when v is to the left of u</param>
        private void CalcPair(int u, int v, out int cuv, out int cvu) {
            List<int> su = this.S[u], sv = this.S[v], pu = this.P[u], pv = this.P[v];
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

        private int CountOnArrays(List<int> unbs, List<int> vnbs) {
            int ret = 0;
            int vl = vnbs.Count - 1;
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
        private int CountOnArrays(List<int> unbs, List<int> vnbs, Dictionary<int, int> uCrossingCounts,
                          Dictionary<int, int> vCrossingCount) {
            int ret = 0;
            int vl = vnbs.Count - 1;
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


        //in this routine u and v are adjacent, and u is to the left of v before the swap
        private void Swap(int u, int v) {
            Debug.Assert(this.UAndVAreOnSameLayer(u, v));
            Debug.Assert(this.UIsToTheLeftOfV(u, v));
            Debug.Assert(this.CanSwap(u, v));

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

            this.UpdateSsContainingUV(u, v);

            this.UpdatePsContainingUV(u, v);
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

        private bool HeadOfTheCoin() {
            return this.random.Next(2) == 0;
        }

        
        internal void DoSwaps() {
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
            Debug.Assert(this.SPAreCorrect());
        }

        private bool SPAreCorrect()
        {
            int n = this.properLayeredGraph.NodeCount;
            for (int i = 0; i < n; i++) {
                if (!this.SIsCorrect(i)) {
                    return false;
                }
            }

            return true;
        }

        private bool SIsCorrect(int i)
        {
            var s = this.S[i];
            Dictionary<int, int> so = this.SOrder[i];
            for (int k = 0; k < s.Count; k++)
            {
                int u = s[k];
                int uPosition = 0;
                if (so.TryGetValue(u, out uPosition) == false) {
                    return false;
                }

                if (uPosition != k) {
                    return false;
                }
            }

            for (int k = 0; k < s.Count - 1; k++)
            {
                int u = s[k];
                int v = s[k + 1];
                if (!this.UIsToTheLeftOfV(u, v)) {
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Is called just after median layer swap is done
        /// </summary>
        private void InitArrays() {
            if (this.S == null) {
                this.AllocArrays();
            }

            for (int i = 0; i < this.properLayeredGraph.NodeCount; i++) {
                this.POrder[i].Clear();
                this.SOrder[i].Clear();
                this.S[i].Clear();
                this.P[i].Clear();
            }


            for (int i = 0; i < this.layers.Length; i++) {
                this.InitPSArraysForLayer(this.layers[i]);
            }
        }

        private void DisturbLayer(int[] layer) {
            for (int i = 0; i < layer.Length - 1; i++) {
                this.AdjacentSwapToTheRight(layer, i);
            }
        }

        private bool AdjExchangeLayer(int i) {
            int[] layer = this.layers[i];
            bool gain = this.ExchangeWithGainWithNoDisturbance(layer);

            if (gain) {
                return true;
            }

            this.DisturbLayer(layer);

            return this.ExchangeWithGainWithNoDisturbance(layer);
        }

        private void AllocArrays() {
            int n = this.properLayeredGraph.NodeCount;
            this.P = new List<int>[n];
            this.S = new List<int>[n];


            this.POrder = new Dictionary<int, int>[n];
            this.SOrder = new Dictionary<int, int>[n];
            if (this.hasCrossWeights) {
                this.outCrossingCount = new Dictionary<int, int>[n];
                this.inCrossingCount = new Dictionary<int, int>[n];
            }
            for (int i = 0; i < n; i++) {
                int count = this.properLayeredGraph.InEdgesCount(i);
                this.P[i] = new List<int>();
                if (this.hasCrossWeights) {
                    Dictionary<int, int> inCounts = this.inCrossingCount[i] = new Dictionary<int, int>(count);
                    foreach (LayerEdge le in this.properLayeredGraph.InEdges(i)) {
                        inCounts[le.Source] = le.CrossingWeight;
                    }
                }
                this.POrder[i] = new Dictionary<int, int>(count);
                count = this.properLayeredGraph.OutEdgesCount(i);
                this.S[i] = new List<int>();
                this.SOrder[i] = new Dictionary<int, int>(count);
                if (this.hasCrossWeights) {
                    Dictionary<int, int> outCounts = this.outCrossingCount[i] = new Dictionary<int, int>(count);
                    foreach (LayerEdge le in this.properLayeredGraph.OutEdges(i)) {
                        outCounts[le.Target] = le.CrossingWeight;
                    }
                }
            }
        }

        private void UpdatePsContainingUV(int u, int v) {
            if (this.S[u].Count <= this.S[v].Count) {
                foreach (int a in this.S[u]) {
                    Dictionary<int, int> porder = this.POrder[a];
                    //of course porder contains u, let us see if it contains v
                    if (porder.ContainsKey(v)) {
                        int vOffset = porder[v];
                        //swap u and v in the array P[coeff]
                        var p = this.P[a];
                        p[vOffset - 1] = v;
                        p[vOffset] = u;
                        //update sorder itself
                        porder[v] = vOffset - 1;
                        porder[u] = vOffset;
                    }
                }
            } else {
                foreach (int a in this.S[v]) {
                    Dictionary<int, int> porder = this.POrder[a];
                    //of course porder contains u, let us see if it contains v
                    if (porder.ContainsKey(u)) {
                        int vOffset = porder[v];
                        //swap u and v in the array P[coeff]
                        var p = this.P[a];
                        p[vOffset - 1] = v;
                        p[vOffset] = u;
                        //update sorder itself
                        porder[v] = vOffset - 1;
                        porder[u] = vOffset;
                    }
                }
            }
        }

        private void SwapToTheRight(int[] layer, int i) {
            for (int j = i; j < layer.Length - 1; j++) {
                this.AdjacentSwapToTheRight(layer, j);
            }
        }

        private void SwapToTheLeft(int[] layer, int i) {
            for (int j = i - 1; j >= 0; j--) {
                this.AdjacentSwapToTheRight(layer, j);
            }
        }

        /// <summary>
        /// swaps i-th element with i+1
        /// </summary>
        /// <param name="layer">the layer to work on</param>
        /// <param name="i">the position to start</param>
        /// <returns></returns>
        private void AdjacentSwapToTheRight(int[] layer, int i) {
            int u = layer[i], v = layer[i + 1];

            int gain = this.SwapGain(u, v);

            if (gain > 0 || (gain == 0 && this.HeadOfTheCoin())) {
                this.Swap(u, v);
                return;
            }
        }

        /// <summary>
        /// Sweep layer from left to right and fill S,P arrays as we go.
        /// The arrays P and S will be sorted according to X. Note that we will not keep them sorted
        /// as we doing adjacent swaps. Initial sorting only needed to calculate initial clr,crl values.
        /// </summary>
        /// <param name="layer"></param>
        private void InitPSArraysForLayer(int[] layer) {
            foreach (int l in layer)
            {
                foreach (int p in this.properLayeredGraph.Pred(l))
                {
                    Dictionary<int, int> so = this.SOrder[p];
                    if (so.ContainsKey(l)) {
                        continue;
                    }

                    int sHasNow = so.Count;
                    this.S[p].Add(l); //l takes the first available slot in S[p]
                    so[l] = sHasNow;
                }
                foreach (int s in this.properLayeredGraph.Succ(l))
                {
                    Dictionary<int, int> po = this.POrder[s];
                    if (po.ContainsKey(l)) {
                        continue;
                    }

                    int pHasNow = po.Count;
                    this.P[s].Add(l); //l take the first available slot in P[s]
                    po[l] = pHasNow;
                }
            }
        }

        private void UpdateSsContainingUV(int u, int v) {
            if (this.P[u].Count <= this.P[v].Count) {
                foreach (int a in this.P[u]) {
                    Dictionary<int, int> sorder = this.SOrder[a];
                    //of course sorder contains u, let us see if it contains v
                    if (sorder.ContainsKey(v)) {
                        int vOffset = sorder[v];
                        //swap u and v in the array S[coeff]
                        var s = this.S[a];
                        s[vOffset - 1] = v;
                        s[vOffset] = u;
                        //update sorder itself
                        sorder[v] = vOffset - 1;
                        sorder[u] = vOffset;
                    }
                }
            } else {
                foreach (int a in this.P[v]) {
                    Dictionary<int, int> sorder = this.SOrder[a];
                    //of course sorder contains u, let us see if it contains v
                    if (sorder.ContainsKey(u)) {
                        int vOffset = sorder[v];
                        //swap u and v in the array S[coeff]
                        var s = this.S[a];
                        s[vOffset - 1] = v;
                        s[vOffset] = u;
                        //update sorder itself
                        sorder[v] = vOffset - 1;
                        sorder[u] = vOffset;
                    }
                }
            }
        }

        private bool UAndVAreOnSameLayer(int u, int v)
        {
            return this.layering[u] == this.layering[v];
        }

        private bool UIsToTheLeftOfV(int u, int v)
        {
            return this.X[u] < this.X[v];
        }
    }
}