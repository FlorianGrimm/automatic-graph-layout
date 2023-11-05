using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// vertical constraints for Suquiyama scheme
    /// </summary>
    internal class VerticalConstraintsForSugiyama {
        private readonly Set<Node> _maxLayerOfGeomGraph = new Set<Node>();
        /// <summary>
        /// nodes that are pinned to the max layer
        /// </summary>
        internal Set<Node> MaxLayerOfGeomGraph {
            get { return this._maxLayerOfGeomGraph; }
        }

        private readonly Set<Node> _minLayerOfGeomGraph = new Set<Node>();
        /// <summary>
        /// nodes that are pinned to the min layer
        /// </summary>
        internal Set<Node> MinLayerOfGeomGraph
        {
            get { return this._minLayerOfGeomGraph; }
        }

        private Set<Tuple<Node, Node>> sameLayerConstraints = new Set<Tuple<Node, Node>>();
        /// <summary>
        /// set of couple of nodes belonging to the same layer
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        internal Set<Tuple<Node, Node>> SameLayerConstraints
        {
            get { return this.sameLayerConstraints; }
        }

        private Set<Tuple<Node, Node>> upDownConstraints = new Set<Tuple<Node, Node>>();

        /// <summary>
        /// set of node couples such that the first node of the couple is above the second one
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        internal Set<Tuple<Node, Node>> UpDownConstraints
        {
            get { return this.upDownConstraints; }
        }
        /// <summary>
        /// pins a node to max layer
        /// </summary>
        /// <param name="node"></param>
        internal void PinNodeToMaxLayer(Node node)
        {
            this.MaxLayerOfGeomGraph.Insert(node);
        }

        /// <summary>
        /// pins a node to min layer
        /// </summary>
        /// <param name="node"></param>
        internal void PinNodeToMinLayer(Node node)
        {
            System.Diagnostics.Debug.Assert(node != null);
            this.MinLayerOfGeomGraph.Insert(node);
        }

        internal bool IsEmpty {
            get { return this.MaxLayerOfGeomGraph.Count == 0 && this.MinLayerOfGeomGraph.Count == 0 && this.SameLayerConstraints.Count == 0 && this.UpDownConstraints.Count == 0; }
        }


      
        

        internal void Clear() {
            this.MaxLayerOfGeomGraph.Clear(); 
            this.MinLayerOfGeomGraph.Clear();
            this.SameLayerConstraints.Clear();
            this.UpDownConstraints.Clear();
        }

        private Set<IntPair> gluedUpDownIntConstraints = new Set<IntPair>();

        internal Set<IntPair> GluedUpDownIntConstraints {
            get { return this.gluedUpDownIntConstraints; }
            set { this.gluedUpDownIntConstraints = value; }
        }

        private Dictionary<Node, int> nodeIdToIndex;
        private BasicGraph<Node, PolyIntEdge> intGraph;

        /// <summary>
        /// this graph is obtained from intGraph by glueing together same layer vertices
        /// </summary>
        private BasicGraphOnEdges<IntPair> gluedIntGraph;
        private int maxRepresentative;
        private int minRepresentative;

        /// <summary>
        /// Maps each node participating in same layer relation its representative on the layer.
        /// </summary>
        private Dictionary<int, int> sameLayerDictionaryOfRepresentatives = new Dictionary<int, int>();
        private Dictionary<int, IEnumerable<int>> representativeToItsLayer = new Dictionary<int, IEnumerable<int>>();
        internal IEnumerable<IEdge> GetFeedbackSet(BasicGraph<Node, PolyIntEdge> intGraphPar, Dictionary<Node, int> nodeIdToIndexPar) {
            this.nodeIdToIndex = nodeIdToIndexPar;
            this.intGraph = intGraphPar;
            this.maxRepresentative = -1;
            this.minRepresentative = -1;
            this.CreateIntegerConstraints();
            this.GlueTogetherSameConstraintsMaxAndMin();
            this.AddMaxMinConstraintsToGluedConstraints();
            this.RemoveCyclesFromGluedConstraints();
            return this.GetFeedbackSet();
        }

        private void RemoveCyclesFromGluedConstraints() {
            var feedbackSet= CycleRemoval<IntPair>.
                GetFeedbackSetWithConstraints(new BasicGraphOnEdges<IntPair>(this.GluedUpDownIntConstraints, this.intGraph.NodeCount), null);
            //feedbackSet contains all glued constraints making constraints cyclic
            foreach (IntPair p in feedbackSet) {
                this.GluedUpDownIntConstraints.Remove(p);
            }
        }

        private void AddMaxMinConstraintsToGluedConstraints() {
            if (this.maxRepresentative != -1) {
                for (int i = 0; i < this.intGraph.NodeCount; i++) {
                    int j = this.NodeToRepr(i);
                    if (j != this.maxRepresentative) {
                        this.GluedUpDownIntConstraints.Insert(new IntPair(this.maxRepresentative, j));
                    }
                }
            }

            if (this.minRepresentative != -1) {
                for (int i = 0; i < this.intGraph.NodeCount; i++) {
                    int j = this.NodeToRepr(i);
                    if (j != this.minRepresentative) {
                        this.GluedUpDownIntConstraints.Insert(new IntPair(j, this.minRepresentative));
                    }
                }
            }
        }

        private void GlueTogetherSameConstraintsMaxAndMin() {
            this.CreateDictionaryOfSameLayerRepresentatives();
            this.GluedUpDownIntConstraints = new Set<IntPair>(from p in this.UpDownInts select this.GluedIntPair(p));
        }

        internal IntPair GluedIntPair(Tuple<int, int> p) {
            return new IntPair(this.NodeToRepr(p.Item1), this.NodeToRepr(p.Item2));
        }
     
        private IntPair GluedIntPair(PolyIntEdge p) {
            return new IntPair(this.NodeToRepr(p.Source), this.NodeToRepr(p.Target));
        }

        internal IntPair GluedIntPair(IntPair p) {
            return new IntPair(this.NodeToRepr(p.First), this.NodeToRepr(p.Second));
        }

        internal PolyIntEdge GluedIntEdge(PolyIntEdge intEdge) {
            int sourceRepr = this.NodeToRepr(intEdge.Source);
            int targetRepr = this.NodeToRepr(intEdge.Target);
            PolyIntEdge ie = new PolyIntEdge(sourceRepr, targetRepr);
            ie.Separation = intEdge.Separation;
            ie.Weight = 0;
            ie.Edge = intEdge.Edge;
            return ie;
        }
        

        internal int NodeToRepr(int node) {
            int repr;
            if (this.sameLayerDictionaryOfRepresentatives.TryGetValue(node, out repr)) {
                return repr;
            }

            return node;
        }

        private void CreateDictionaryOfSameLayerRepresentatives() {
            BasicGraphOnEdges<IntPair> graphOfSameLayers = this.CreateGraphOfSameLayers();
            foreach (var comp in ConnectedComponentCalculator<IntPair>.GetComponents(graphOfSameLayers)) {
                this.GlueSameLayerNodesOfALayer(comp);
            }
        }

        private BasicGraphOnEdges<IntPair> CreateGraphOfSameLayers() {
            return new BasicGraphOnEdges<IntPair>(this.CreateEdgesOfSameLayers(), this.intGraph.NodeCount);
        }

        private IEnumerable<IntPair> CreateEdgesOfSameLayers() {
            List<IntPair> ret = new List<IntPair>();
            if (this.maxRepresentative != -1) {
                ret.AddRange(from v in this.maxLayerInt where v != this.maxRepresentative select new IntPair(this.maxRepresentative, v));
            }

            if (this.minRepresentative != -1) {
                ret.AddRange(from v in this.minLayerInt where v != this.minRepresentative select new IntPair(this.minRepresentative, v));
            }

            ret.AddRange(from couple in this.SameLayerInts select new IntPair(couple.Item1, couple.Item2));
            return ret;
        }
        /// <summary>
        /// maps all nodes of the component to one random representative
        /// </summary>
        /// <param name="sameLayerNodes"></param>
        private void GlueSameLayerNodesOfALayer(IEnumerable<int> sameLayerNodes) {
            if (sameLayerNodes.Count<int>() > 1) {
                int representative = -1;
                if (this.ComponentsIsMaxLayer(sameLayerNodes)) {
                    foreach (int v in sameLayerNodes) {
                        this.sameLayerDictionaryOfRepresentatives[v] = representative = this.maxRepresentative;
                    }
                } else if (this.ComponentIsMinLayer(sameLayerNodes)) {
                    foreach (int v in sameLayerNodes) {
                        this.sameLayerDictionaryOfRepresentatives[v] = representative = this.minRepresentative;
                    }
                } else {
                    foreach (int v in sameLayerNodes) {
                        if (representative == -1) {
                            representative = v;
                        }

                        this.sameLayerDictionaryOfRepresentatives[v] = representative;
                    }
                }
                this.representativeToItsLayer[representative] = sameLayerNodes;
            }
        }

        private bool ComponentIsMinLayer(IEnumerable<int> component) {
            return component.Contains<int>(this.minRepresentative);
        }

        private bool ComponentsIsMaxLayer(IEnumerable<int> component) {
            return component.Contains<int>(this.maxRepresentative);
        }

        private List<int> maxLayerInt = new List<int>();
        private List<int> minLayerInt = new List<int>();
        private List<Tuple<int, int>> sameLayerInts = new List<Tuple<int, int>>();

        /// <summary>
        /// contains also pinned max and min pairs
        /// </summary>
        internal List<Tuple<int, int>> SameLayerInts {
            get { return this.sameLayerInts; }
            set { this.sameLayerInts = value; }
        }

        private List<Tuple<int, int>> upDownInts = new List<Tuple<int, int>>();

        internal List<Tuple<int, int>> UpDownInts {
            get { return this.upDownInts; }
            set { this.upDownInts = value; }
        }

        private void CreateIntegerConstraints() {
            this.CreateMaxIntConstraints();
            this.CreateMinIntConstraints();
            this.CreateUpDownConstraints();
            this.CreateSameLayerConstraints();
        }

        private void CreateSameLayerConstraints() {
            this.SameLayerInts = this.CreateIntConstraintsFromStringCouples(this.SameLayerConstraints);
        }

        private void CreateUpDownConstraints() {
            this.UpDownInts = this.CreateIntConstraintsFromStringCouples(this.UpDownConstraints);
        }

        private List<Tuple<int, int>> CreateIntConstraintsFromStringCouples(Set<Tuple<Node, Node>> set)
        {
            return new List<Tuple<int, int>>(from couple in set
                                              let t = new Tuple<int, int>(this.NodeIndex(couple.Item1), this.NodeIndex(couple.Item2))
                                              where t.Item1 != -1 && t.Item2 != -1
                                              select t);
        }

        private void CreateMinIntConstraints() {
            this.minLayerInt = this.CreateIntConstraintsFromExtremeLayer(this.MinLayerOfGeomGraph);
            if (this.minLayerInt.Count > 0) {
                this.minRepresentative = this.minLayerInt[0];
            }
        }

        private void CreateMaxIntConstraints() {
            this.maxLayerInt = this.CreateIntConstraintsFromExtremeLayer(this.MaxLayerOfGeomGraph);
            if (this.maxLayerInt.Count > 0) {
                this.maxRepresentative = this.maxLayerInt[0];
            }
        }

        private List<int> CreateIntConstraintsFromExtremeLayer(Set<Node> setOfNodes) {
            return new List<int>(from node in setOfNodes let index = this.NodeIndex(node) where index != -1 select index);   
        }

        private int NodeIndex(Node node) {
            int index;
            if (this.nodeIdToIndex.TryGetValue(node, out index)) {
                return index;
            }

            return -1;
        }
        private IEnumerable<IEdge> GetFeedbackSet() {
            this.gluedIntGraph = this.CreateGluedGraph();
            return this.UnglueIntPairs(CycleRemoval<IntPair>.GetFeedbackSetWithConstraints(this.gluedIntGraph, this.GluedUpDownIntConstraints));//avoiding lazy evaluation
        }

        private IEnumerable<IEdge> UnglueIntPairs(IEnumerable<IEdge> gluedEdges) {
            foreach (IEdge gluedEdge in gluedEdges) {
                foreach (IEdge ungluedEdge in this.UnglueEdge(gluedEdge)) {
                    yield return ungluedEdge;
                }
            }
        }

        private IEnumerable<IEdge> UnglueEdge(IEdge gluedEdge) {
            foreach (int source in this.UnglueNode(gluedEdge.Source)) {
                foreach (PolyIntEdge edge in this.intGraph.OutEdges(source)) {
                    if (this.NodeToRepr(edge.Target) == gluedEdge.Target) {
                        yield return edge;
                    }
                }
            }
        }

        private BasicGraphOnEdges<IntPair> CreateGluedGraph() {
            return new BasicGraphOnEdges<IntPair>(new Set<IntPair>(from edge in this.intGraph.Edges select this.GluedIntPair(edge)), this.intGraph.NodeCount);
        }

        private IEnumerable<int> UnglueNode(int node) {
            IEnumerable<int> layer;
            if (this.representativeToItsLayer.TryGetValue(node, out layer)) {
                return layer;
            }

            return new int[] { node };
        }


        internal int[] GetGluedNodeCounts() {
            int[] ret = new int[this.nodeIdToIndex.Count];
            for (int node = 0; node < ret.Length; node++) {
                ret[this.NodeToRepr(node)]++;
            }

            return ret;
        }
    }
}
