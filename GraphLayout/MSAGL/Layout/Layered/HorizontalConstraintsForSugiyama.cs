using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    internal class HorizontalConstraintsForSugiyama {
        private readonly Set<Tuple<Node, Node>> leftRightConstraints = new Set<Tuple<Node, Node>>();
        private readonly Set<Tuple<Node, Node>> leftRightNeighbors = new Set<Tuple<Node, Node>>();

        /// <summary>
        /// node is mapped to the block root
        /// </summary>
        private readonly Dictionary<int, int> nodeToBlockRoot = new Dictionary<int, int>();
        private readonly Set<Tuple<Node, Node>> upDownVerticalConstraints = new Set<Tuple<Node, Node>>();

        /// <summary>
        /// The right most node to the left of the  block is called a block root. The root does not belong to its block.
        /// </summary>
        internal Dictionary<int, List<int>> BlockRootToBlock = new Dictionary<int, List<int>>();

        internal Set<Tuple<int, int>> LeftRighInts;

        /// <summary>
        /// the set of integer pairs (i,j) such that i is a left neighbor of j
        /// </summary>
        internal Set<Tuple<int, int>> LeftRightIntNeibs;

        internal Set<Tuple<int, int>> VerticalInts;
        private Dictionary<Node, int> nodeIdToIndex;

        internal Set<Tuple<Node, Node>> LeftRightNeighbors {
            get { return this.leftRightNeighbors; }
        }

        internal Set<Tuple<Node, Node>> UpDownVerticalConstraints {
            get { return this.upDownVerticalConstraints; }
        }

        internal Set<Tuple<Node, Node>> LeftRightConstraints {
            get { return this.leftRightConstraints; }
        }

        internal bool IsEmpty {
            get {
                return this.LeftRightNeighbors.Count == 0 && this.UpDownVerticalConstraints.Count == 0 &&
                       this.LeftRightConstraints.Count == 0;
            }
        }

        [SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        internal void Clear() {}


        internal void AddSameLayerNeighbors(List<Node> neighbors) {
            for (int i = 0; i < neighbors.Count - 1; i++) {
                this.AddSameLayerNeighborsPair(neighbors[i], neighbors[i + 1]);
            }
        }

        internal void AddSameLayerNeighborsPair(Node leftNode, Node rightNode) {
            this.LeftRightNeighbors.Insert(new Tuple<Node, Node>(leftNode, rightNode));
        }

        private int NodeToBlockRootSoft(int i) {
            int blockRoot;
            if (this.nodeToBlockRoot.TryGetValue(i, out blockRoot)) {
                return blockRoot;
            }

            return i;
        }

        private void CreateMappingOfNeibBlocks() {
            BasicGraphOnEdges<IntPair> graph = this.BasicGraphFromLeftRightIntNeibs();
            for (int root = 0; root < graph.NodeCount; root++) {
                if (graph.InEdges(root).Count == 0 && !this.nodeToBlockRoot.ContainsKey(root)) {
                    var block = new List<int>();
                    int current = root;
                    for (IList<IntPair> outEdges = graph.OutEdges(current); outEdges.Count > 0;
                         outEdges = graph.OutEdges(current)) {
                        current = outEdges[0].Second;
                        block.Add(current);
                        this.nodeToBlockRoot[current] = root;
                    }
                    if (block.Count > 0) {
                        this.BlockRootToBlock[root] = block;
                    }
                }
            }
        }

        private BasicGraphOnEdges<IntPair> BasicGraphFromLeftRightIntNeibs() {
            return new BasicGraphOnEdges<IntPair>(from p in this.LeftRightIntNeibs select new IntPair(p.Item1, p.Item2));
        }

        private int NodeIndex(Node node) {
            int index;
            if (this.nodeIdToIndex.TryGetValue(node, out index)) {
                return index;
            }

            return -1;
        }

        internal void PrepareForOrdering(Dictionary<Node, int> nodeToIndexParameter, int[] yLayers) {
            this.nodeIdToIndex = nodeToIndexParameter;
            this.MapNodesToToIntegers(yLayers);

            this.CreateMappingOfNeibBlocks();
            this.LiftLeftRightRelationsToNeibBlocks();
            //MakeUpDownRelationsMonotone(yLayers);
        }

        //see UpDownMonotone.png
        //        void MakeUpDownRelationsMonotone(int[] yLayers) {
        //            BasicGraph<IntPair> upDownGraph = new BasicGraph<IntPair>(from c in this.verticalInts select new IntPair(c.First,c.Second));
        //            List<Tuple<int, int>> upDownToRemove = new List<Tuple<int, int>>();
        //            foreach (IEnumerable<int> componentNodes in ConnectedComponentCalculator<IntPair>.GetComponents(GraphOfLeftRightRelations())) {
        //                ResolveConflictsUboveComponent(upDownGraph, componentNodes, upDownToRemove, yLayers);
        //                ResolveConflictsBelowComponent(upDownGraph, componentNodes, upDownToRemove, yLayers);
        //            }
        //
        //            foreach (var v in upDownToRemove)
        //                this.verticalInts.Remove(v);
        //        }
        //makes left-right relations to be between neighb blocks and removes cycles in these relations
        private void LiftLeftRightRelationsToNeibBlocks() {
            this.LeftRighInts = new Set<Tuple<int, int>>(from p in this.leftRightConstraints
                                                     let ip =
                                                         new Tuple<int, int>(this.NodeIndex(p.Item1), this.NodeIndex(p.Item2))
                                                     where ip.Item1 != -1 && ip.Item2 != -1
                                                     let ipb =
                                                         new Tuple<int, int>(this.NodeToBlockRootSoft(ip.Item1),
                                                                              this.NodeToBlockRootSoft(ip.Item2))
                                                     where ipb.Item1 != ipb.Item2
                                                     select ipb);
            IEnumerable<IEdge> feedbackSet =
                CycleRemoval<IntPair>.GetFeedbackSet(
                    new BasicGraphOnEdges<IntPair>(from p in this.LeftRighInts select new IntPair(p.Item1, p.Item2)));
            foreach (IntPair ip in feedbackSet) {
                this.LeftRighInts.Remove(new Tuple<int, int>(ip.First, ip.Second));
            }
        }

        private void MapNodesToToIntegers(int[] yLayers) {
            this.LeftRightIntNeibs = new Set<Tuple<int, int>>(from p in this.LeftRightNeighbors
                                                          let left = this.NodeIndex(p.Item1)
                                                          where left != -1
                                                          let right = this.NodeIndex(p.Item2)
                                                          where right != -1
                                                          select new Tuple<int, int>(left, right));

            //as we follow yLayers there will not be cycles in verticalIntConstraints
            this.VerticalInts = new Set<Tuple<int, int>>(from p in this.UpDownVerticalConstraints
                                                     let upper = this.NodeIndex(p.Item1)
                                                     where upper != -1
                                                     let lower = this.NodeIndex(p.Item2)
                                                     where lower != -1
                                                     where yLayers[upper] > yLayers[lower]
                                                     select new Tuple<int, int>(upper, lower));
        }
    }
}