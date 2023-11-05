using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.GraphAlgorithms;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Following "Improving Layered Graph Layouts with Edge Bundling" and
    /// "Two polynomial time algorithms for the bundle-Line crossing minimization problem"
    /// Postprocessing minimizing crossings step that works on the layered graph
    /// </summary>
    internal class MetroMapOrdering {
        private LayerArrays layerArrays;
        private Dictionary<int, IntPair> nodePositions;
        private ProperLayeredGraph properLayeredGraph;

        private MetroMapOrdering(ProperLayeredGraph properLayeredGraph, LayerArrays layerArrays,
                         Dictionary<int, IntPair> nodePositions) {
            this.properLayeredGraph = properLayeredGraph;
            this.layerArrays = layerArrays;
            this.nodePositions = nodePositions;
        }

        /// <summary>
        /// Reorder only points having identical nodePositions
        /// </summary>
        internal static void UpdateLayerArrays(ProperLayeredGraph properLayeredGraph, LayerArrays layerArrays,
                                               Dictionary<int, IntPair> nodePositions) {
            new MetroMapOrdering(properLayeredGraph, layerArrays, nodePositions).UpdateLayerArrays();
        }

        /// <summary>
        /// Reorder virtual nodes between the same pair of real nodes
        /// </summary>
        internal static void UpdateLayerArrays(ProperLayeredGraph properLayeredGraph, LayerArrays layerArrays) {
            Dictionary<int, IntPair> nodePositions = BuildInitialNodePositions(properLayeredGraph, layerArrays);
            UpdateLayerArrays(properLayeredGraph, layerArrays, nodePositions);
        }

        private static Dictionary<int, IntPair> BuildInitialNodePositions(ProperLayeredGraph properLayeredGraph,
                                                                LayerArrays layerArrays) {
            var result = new Dictionary<int, IntPair>();
            for (int i = 0; i < layerArrays.Layers.Length; i++) {
                int prev = 0, curr = 0;
                var layer = layerArrays.Layers[i];
                while (curr < layer.Length) {
                    while (curr < layer.Length &&
                           properLayeredGraph.IsVirtualNode(layer[curr])) {
                        curr++;
                    }

                    for (int j = prev; j < curr; j++) {
                        result[layer[j]] = new IntPair(i, prev);
                    }

                    if (curr < layer.Length) {
                        result[layer[curr]] = new IntPair(i, curr);
                    }

                    curr++;
                    prev = curr;
                }
            }

            return result;
        }

        private void UpdateLayerArrays() {
            //algo stuff here
            Dictionary<IntPair, List<int>> ordering = this.CreateInitialOrdering();
            ordering = this.BuildOrdering(ordering);
            this.RestoreLayerArrays(ordering);
        }

        private Dictionary<IntPair, List<int>> CreateInitialOrdering() {
            var initialOrdering = new Dictionary<IntPair, List<int>>();
            foreach (var layer in this.layerArrays.Layers) {
                for (int j = 0; j < layer.Length; j++) {
                    int node = layer[j];
                    if (!initialOrdering.ContainsKey(this.nodePositions[node])) {
                        initialOrdering[this.nodePositions[node]] = new List<int>();
                    }

                    initialOrdering[this.nodePositions[node]].Add(node);
                }
            }
            return initialOrdering;
        }

        private Dictionary<IntPair, List<int>> BuildOrdering(Dictionary<IntPair, List<int>> initialOrdering) {
            //run through nodes points and build order
            var result = new Dictionary<IntPair, List<int>>();
            var reverseOrder = new Dictionary<int, int>();
            foreach (var layer in this.layerArrays.Layers) {
                for (int j = 0; j < layer.Length; j++) {
                    int node = layer[j];

                    //already processed
                    if (result.ContainsKey(this.nodePositions[node])) {
                        continue;
                    }

                    this.BuildNodeOrdering(initialOrdering[this.nodePositions[node]], reverseOrder);
                    result[this.nodePositions[node]] = initialOrdering[this.nodePositions[node]];
                }
            }

            return result;
        }

        private void BuildNodeOrdering(List<int> nodeOrdering, Dictionary<int, int> inverseToOrder) {
            nodeOrdering.Sort(this.Comparison(inverseToOrder));

            for (int i = 0; i < nodeOrdering.Count; i++) {
                inverseToOrder[nodeOrdering[i]] = i;
            }
        }

        private Comparison<int> Comparison(Dictionary<int, int> inverseToOrder) {
            return delegate(int node1, int node2) {
                       Debug.Assert(this.properLayeredGraph.IsVirtualNode(node1) &&
                                    this.properLayeredGraph.IsVirtualNode(node2));

                       int succ1 = this.properLayeredGraph.Succ(node1).ElementAt(0);
                       int succ2 = this.properLayeredGraph.Succ(node2).ElementAt(0);
                       int pred1 = this.properLayeredGraph.Pred(node1).ElementAt(0);
                       int pred2 = this.properLayeredGraph.Pred(node2).ElementAt(0);

                       IntPair succIntPair1 = this.nodePositions[succ1];
                       IntPair succIntPair2 = this.nodePositions[succ2];
                       IntPair predIntPair1 = this.nodePositions[pred1];
                       IntPair predIntPair2 = this.nodePositions[pred2];

                       if (succIntPair1 != succIntPair2) {
                           if (predIntPair1 != predIntPair2) {
                        return predIntPair1.CompareTo(predIntPair2);
                    }

                    return succIntPair1.CompareTo(succIntPair2);
                       }
                       if (this.properLayeredGraph.IsVirtualNode(succ1)) {
                           if (predIntPair1 != predIntPair2) {
                        return predIntPair1.CompareTo(predIntPair2);
                    }

                    int o1 = inverseToOrder[succ1];
                           int o2 = inverseToOrder[succ2];
                           Debug.Assert(o1 != -1 && o2 != -1);
                           return (o1.CompareTo(o2));
                       }
                       while (this.nodePositions[pred1] == this.nodePositions[pred2] &&
                              this.properLayeredGraph.IsVirtualNode(pred1)) {
                           pred1 = this.properLayeredGraph.Pred(pred1).ElementAt(0);
                           pred2 = this.properLayeredGraph.Pred(pred2).ElementAt(0);
                       }

                       if (this.nodePositions[pred1] == this.nodePositions[pred2]) {
                    return node1.CompareTo(node2);
                }

                return this.nodePositions[pred1].CompareTo(this.nodePositions[pred2]);
                   };
        }

        private void RestoreLayerArrays(Dictionary<IntPair, List<int>> ordering) {
            foreach (var layer in this.layerArrays.Layers) {
                int pred = 0, tec = 0;
                while (tec < layer.Length) {
                    while (tec < layer.Length &&
                           this.nodePositions[layer[pred]] == this.nodePositions[layer[tec]]) {
                        tec++;
                    }

                    var t = ordering[this.nodePositions[layer[pred]]];
                    //System.Diagnostics.Debug.Assert(t.Count == tec - pred);
                    for (int j = pred; j < tec; j++) {
                        layer[j] = t[j - pred];
                    }
                    pred = tec;
                }
            }

            this.layerArrays.UpdateXFromLayers();
        }
    }
}