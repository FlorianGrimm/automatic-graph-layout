using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;

namespace Microsoft.Msagl.Core.GraphAlgorithms {
    /// <summary>
    /// 
    /// </summary>
    public class MinimumSpanningTreeByPrim {
        private readonly BasicGraphOnEdges<IEdge> graph;
        private readonly Func<IEdge, double> weight;
        private readonly int root;
        private readonly BinaryHeapPriorityQueue q;
        private Set<int> treeNodes = new Set<int>();

        //map of neighbors of the tree to the edges connected them to the tree
        private Dictionary<int, IEdge> hedgehog = new Dictionary<int, IEdge>(); 
        
        /// <summary>
        /// 
        /// </summary>
        public static void Test() {

        }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="graph"></param>
        /// <param name="weight"></param>
        /// <param name="root">the node we start building the tree</param>
        internal MinimumSpanningTreeByPrim(BasicGraphOnEdges<IEdge> graph, Func<IEdge, double> weight, int root) {
            this.graph = graph;
            this.weight = weight;
            this.root = root;
            this.q =new BinaryHeapPriorityQueue(graph.NodeCount);
        }

        private bool NodeIsInTree(int i) {
            return this.treeNodes.Contains(i);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public IList<IEdge> GetTreeEdges()
        {
            var ret = new List<IEdge>(this.graph.NodeCount - 1);
            this.Init();
            while (ret.Count < this.graph.NodeCount - 1 && this.q.Count > 0) //some nodes might have no edges
{
                this.AddEdgeToTree(ret);
            }

            return ret;
        }

        private void AddEdgeToTree(List<IEdge> ret) {
            var v = this.q.Dequeue();
            var e = this.hedgehog[v];
            this.treeNodes.Insert(v);
            ret.Add(e);
            this.UpdateOutEdgesOfV(v);
            this.UpdateInEdgesOfV(v);
        }

        private void UpdateOutEdgesOfV(int v) {
            foreach (var outEdge in this.graph.OutEdges(v)) {
                var u = outEdge.Target;
                if (this.NodeIsInTree(u)) {
                    continue;
                }

                IEdge oldEdge;
                if (this.hedgehog.TryGetValue(u, out oldEdge)) {
                    var oldWeight = this.weight(oldEdge);
                    var newWeight = this.weight(outEdge);
                    if (newWeight < oldWeight) {
                        this.q.DecreasePriority(u, newWeight);
                        this.hedgehog[u] = outEdge;
                    }
                } else {
                    this.q.Enqueue(u, this.weight(outEdge));
                    this.hedgehog[u] = outEdge;
                }
            }
        }

        private void UpdateInEdgesOfV(int v)
        {
            foreach (var inEdge in this.graph.InEdges(v))
            {
                var u = inEdge.Source;
                if (this.NodeIsInTree(u)) {
                    continue;
                }

                IEdge oldEdge;
                if (this.hedgehog.TryGetValue(u, out oldEdge))
                {
                    var oldWeight = this.weight(oldEdge);
                    var newWeight = this.weight(inEdge);
                    if (newWeight < oldWeight)
                    {
                        this.q.DecreasePriority(u, newWeight);
                        this.hedgehog[u] = inEdge;
                    }
                }
                else
                {
                    this.q.Enqueue(u, this.weight(inEdge));
                    this.hedgehog[u] = inEdge;
                }
            }
        }

        private void Init() {
            this.treeNodes.Insert(this.root);

            foreach (var outEdge in this.graph.OutEdges(this.root)) {
                var w = this.weight(outEdge);
                this.q.Enqueue(outEdge.Target, w);
                this.hedgehog[outEdge.Target] = outEdge;
            }

            foreach (var inEdge in this.graph.InEdges(this.root)) {
                var w = this.weight(inEdge);
                this.q.Enqueue(inEdge.Source, w);
                this.hedgehog[inEdge.Source] = inEdge;
            }
        }
    }
}
