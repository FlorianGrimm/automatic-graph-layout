using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// a class representing a graph where every edge goes down only one layer
    /// </summary>
    internal class ProperLayeredGraph {
        /// <summary>
        /// the underlying basic graph
        /// </summary>
        internal BasicGraph<Node, PolyIntEdge>  BaseGraph;
        private LayerEdge[] virtualNodesToInEdges;
        private LayerEdge[] virtualNodesToOutEdges;
        private int totalNumberOfNodes;
        private int firstVirtualNode;

        private int FirstVirtualNode {
            get {
                return this.firstVirtualNode;
            }
        }


        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        internal ProperLayeredGraph(BasicGraph<Node, PolyIntEdge> intGraph) {
            this.Initialize(intGraph);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        internal void Initialize(BasicGraph<Node, PolyIntEdge> intGraph)
        {
            this.BaseGraph = intGraph;

            if (this.BaseGraph.Edges.Count > 0) {
                var edgesGoingDown = from edge in this.BaseGraph.Edges
                                     where edge.LayerEdges != null
                                     select edge;
                if(edgesGoingDown.Any() ) {
                    this.totalNumberOfNodes = (from edge in edgesGoingDown from layerEdge in edge.LayerEdges
                                           select Math.Max(layerEdge.Source, layerEdge.Target) + 1).Max();
                } else {
                    this.totalNumberOfNodes = intGraph.NodeCount;
                }
            } else {
                this.totalNumberOfNodes = intGraph.NodeCount;
            }

            if (ExistVirtualNodes(this.BaseGraph.Edges)) {
                this.firstVirtualNode = (from edge in this.BaseGraph.Edges
                                    where edge.LayerEdges != null && edge.LayerEdges.Count > 1
                                    let source = edge.Source
                                    from layerEdge in edge.LayerEdges
                                    let layerEdgeSource = layerEdge.Source
                                    where layerEdge.Source != source
                                    select layerEdgeSource).Min();
            } else {
                this.firstVirtualNode = this.BaseGraph.NodeCount;
                this.totalNumberOfNodes = this.BaseGraph.NodeCount;
            }

            this.virtualNodesToInEdges = new LayerEdge[this.totalNumberOfNodes - this.FirstVirtualNode];
            this.virtualNodesToOutEdges = new LayerEdge[this.totalNumberOfNodes - this.FirstVirtualNode];
            foreach (PolyIntEdge e in this.BaseGraph.Edges) {
                if (e.LayerSpan > 0) {
                    foreach (LayerEdge le in e.LayerEdges) {
                        if (le.Target != e.Target) {
                            this.virtualNodesToInEdges[le.Target - this.FirstVirtualNode] = le;
                        }

                        if (le.Source != e.Source) {
                            this.virtualNodesToOutEdges[le.Source - this.FirstVirtualNode] = le;
                        }
                    }
                }
            }
        }

        private static bool ExistVirtualNodes(ICollection<PolyIntEdge> iCollection) {
            foreach (var edge in iCollection) {
                if (edge.LayerEdges != null && edge.LayerEdges.Count > 1) {
                    return true;
                }
            }

            return false;
        }

        //internal ProperLayeredGraph(BasicGraph<IntEdge> intGraph, int[]layering) {
        //    this.baseGraph = intGraph;
        //    this.totalNumberOfNodes = intGraph.Nodes.Count + VirtualNodeCount(layering);
            

        //    virtualNodesToInEdges = new LayerEdge[totalNumberOfNodes - this.baseGraph.Nodes.Count];
        //    virtualNodesToOutEdges = new LayerEdge[totalNumberOfNodes - this.baseGraph.Nodes.Count];
        //    int currentVirtNode = intGraph.Nodes.Count;
        //    foreach (IntEdge e in baseGraph.Edges) {
        //        CreateLayerEdgesPath(e, layering, ref currentVirtNode);
        //        if (e.LayerSpan > 1)
        //            foreach (LayerEdge le in e.LayerEdges) {
        //                if (le.Target != e.Target)
        //                    virtualNodesToInEdges[le.Target - NumOfOriginalNodes] = le;
        //                if (le.Source != e.Source)
        //                    virtualNodesToOutEdges[le.Source - NumOfOriginalNodes] = le;
        //            }
        //    }
        //}


      
        /// <summary>
        /// enumerates over the graph edges
        /// </summary>
        public IEnumerable<LayerEdge> Edges {
            get {
                foreach (PolyIntEdge ie in this.BaseGraph.Edges) {
                    if (ie.LayerSpan > 0) {
                        foreach (LayerEdge le in ie.LayerEdges) {
                            yield return le;
                        }
                    }
                }
            }
        }
        /// <summary>
        /// enumerates over edges of a node
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        public IEnumerable<LayerEdge> InEdges(int node) {
            if (node < this.BaseGraph.NodeCount)//original node
{
                foreach (PolyIntEdge e in this.BaseGraph.InEdges(node)) {
                    if (e.Source != e.Target && e.LayerEdges != null) {
                        yield return LastEdge(e);
                    }
                }
            } else if (node >= this.firstVirtualNode) {
                yield return this.InEdgeOfVirtualNode(node);
            }
        }

        private static LayerEdge LastEdge(PolyIntEdge e) {
            return e.LayerEdges[e.LayerEdges.Count - 1];

        }

        internal LayerEdge InEdgeOfVirtualNode(int node) {
            return this.virtualNodesToInEdges[node - this.FirstVirtualNode];
        }
/// <summary>
/// enumerates over the node outcoming edges
/// </summary>
/// <param name="node"></param>
/// <returns></returns>
        public IEnumerable<LayerEdge> OutEdges(int node) {
            if (node < this.BaseGraph.NodeCount)//original node
{
                foreach (PolyIntEdge e in this.BaseGraph.OutEdges(node)) {
                    if (e.Source != e.Target && e.LayerEdges!=null) {
                        yield return FirstEdge(e);
                    }
                }
            } else if (node >= this.FirstVirtualNode) {
                yield return this.OutEdgeOfVirtualNode(node);
            }
        }
        public bool OutDegreeIsMoreThanOne(int node) {
            if (node < this.BaseGraph.NodeCount)//original node
{
                return this.BaseGraph.OutEdges(node).Count() > 1;
            } else {
                return false;
            }
        }
        public bool InDegreeIsMoreThanOne(int node) {
            if (node < this.BaseGraph.NodeCount)//original node
{
                return this.BaseGraph.InEdges(node).Count() > 1;
            } else {
                return false;
            }
        }
        internal LayerEdge OutEdgeOfVirtualNode(int node) {
            return this.virtualNodesToOutEdges[node - this.FirstVirtualNode];
        }

        private static LayerEdge FirstEdge(PolyIntEdge e) {
            return e.LayerEdges[0];
        }
        /// <summary>
        /// returns the number of incoming edges for an edge
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        public int InEdgesCount(int node) {
            return this.RealInEdgesCount(node);
        }

        private int RealInEdgesCount(int node) {
            return node < this.BaseGraph.NodeCount ? this.BaseGraph.InEdges(node).Count(e=>e.LayerEdges!=null): 1;
        }


        /// <summary>
        /// returns the number of outcoming edges for an edge
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        public int OutEdgesCount(int node) {
            return this.RealOutEdgesCount(node);
        }

        private int RealOutEdgesCount(int node) {
            return node < this.BaseGraph.NodeCount ? this.BaseGraph.OutEdges(node).Count(l=>l.LayerEdges!=null):  1;
        }

        /// <summary>
        /// returns the node count
        /// </summary>
        public int NodeCount {
            get { return this.totalNumberOfNodes; }
        }

        public bool IsRealNode(int node) {
            return (node < this.BaseGraph.NodeCount);
        }

        public bool IsVirtualNode(int node) {
            return !this.IsRealNode(node);
        }

        internal ProperLayeredGraph ReversedClone() {
            List<PolyIntEdge> reversedEdges = this.CreateReversedEdges();
            return new ProperLayeredGraph(new BasicGraph<Node, PolyIntEdge>(reversedEdges, this.BaseGraph.NodeCount));
        }

        private List<PolyIntEdge> CreateReversedEdges() {
            List<PolyIntEdge> ret = new List<PolyIntEdge>();
            foreach (PolyIntEdge e in this.BaseGraph.Edges) {
                if (!e.SelfEdge()) {
                    ret.Add(e.ReversedClone());
                }
            }

            return ret;
        }

        internal IEnumerable<int> Succ(int node) {
            foreach (LayerEdge le in this.OutEdges(node)) {
                yield return le.Target;
            }
        }

        internal IEnumerable<int> Pred(int node) {
            foreach (LayerEdge le in this.InEdges(node)) {
                yield return le.Source;
            }
        }

    }
}
