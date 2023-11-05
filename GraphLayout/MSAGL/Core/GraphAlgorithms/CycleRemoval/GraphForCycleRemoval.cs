using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;

namespace Microsoft.Msagl.Core.GraphAlgorithms {

    internal class GraphForCycleRemoval {
        /// <summary>
        /// this dictionary contains only buckets with nodes which are sources in the graph of constrained edges
        /// </summary>
        private SortedDictionary<int, Set<int>> deltaDegreeBucketsForSourcesInConstrainedSubgraph = new SortedDictionary<int, Set<int>>();
        private Dictionary<int, NodeInfo> nodeInfoDictionary = new Dictionary<int, NodeInfo>();
        private Set<int> sources=new Set<int>();
        private Set<int> sinks=new Set<int>();
        private Set<IntPair> edgesToKeep = new Set<IntPair>();

        internal void AddEdge(IntPair edge) {
            this.edgesToKeep.Insert(edge);
            int source = edge.First; int target = edge.Second;
            this.GetOrCreateNodeInfo(source).AddOutEdge(target);
            this.GetOrCreateNodeInfo(target).AddInEdge(source);
        }

        private NodeInfo GetOrCreateNodeInfo(int node) {
            NodeInfo nodeInfo;
            if (!this.nodeInfoDictionary.TryGetValue(node, out nodeInfo)) {
                nodeInfo = new NodeInfo();
                this.nodeInfoDictionary[node] = nodeInfo;
            }
            return nodeInfo;
        }

        internal void AddConstraintEdge(IntPair intPair) {
            int source = intPair.First; int target = intPair.Second;
            this.GetOrCreateNodeInfo(source).AddOutConstrainedEdge(target);
            this.GetOrCreateNodeInfo(target).AddInConstrainedEdge(source);
        }

        internal bool IsEmpty() {
            return this.nodeInfoDictionary.Count == 0;
        }

        internal void RemoveNode(int u) {
            this.sources.Remove(u);
            this.sinks.Remove(u);
            this.RemoveNodeFromItsBucket(u);
            NodeInfo uNodeInfo=this.nodeInfoDictionary[u];
            Set<int> allNbs = new Set<int>(uNodeInfo.AllNeighbors);
            foreach(int v in allNbs) {
                this.RemoveNodeFromItsBucket(v);
            }

            this.DisconnectNodeFromGraph(u, uNodeInfo);

            foreach (int v in allNbs) {
                this.AddNodeToBucketsSourcesAndSinks(v, this.nodeInfoDictionary[v]);
            }
        }

        private void DisconnectNodeFromGraph(int u, NodeInfo uNodeInfo) {
            foreach (int v in uNodeInfo.OutEdges) {
                this.nodeInfoDictionary[v].RemoveInEdge(u);
            }

            foreach (int v in uNodeInfo.OutConstrainedEdges) {
                this.nodeInfoDictionary[v].RemoveInConstrainedEdge(u);
            }

            foreach (int v in uNodeInfo.InEdges) {
                this.nodeInfoDictionary[v].RemoveOutEdge(u);
            }

            foreach (int v in uNodeInfo.InConstrainedEdges) {
                this.nodeInfoDictionary[v].RemoveOutConstrainedEdge(u);
            }

            this.nodeInfoDictionary.Remove(u);
        }

        private void RemoveNodeFromItsBucket(int v) {
            int delta = this.DeltaDegree(v);
            Set<int> bucket;
            if (this.deltaDegreeBucketsForSourcesInConstrainedSubgraph.TryGetValue(delta, out bucket)) {
                bucket.Remove(v);
                if (bucket.Count == 0) {
                    this.deltaDegreeBucketsForSourcesInConstrainedSubgraph.Remove(delta);
                }
            }
        }

        private int DeltaDegree(int v) {
            int delta = this.nodeInfoDictionary[v].DeltaDegree;
            return delta;
        }



        internal bool TryFindVertexWithNoIncomingConstrainedEdgeAndMaximumOutDegreeMinusInDedree(out int u) {
            var enumerator = this.deltaDegreeBucketsForSourcesInConstrainedSubgraph.GetEnumerator();
            if (enumerator.MoveNext()) {
                var bucketSet = enumerator.Current.Value;
                System.Diagnostics.Debug.Assert(bucketSet.Count > 0);
                var nodeEnumerator = bucketSet.GetEnumerator();
                nodeEnumerator.MoveNext();
                u = nodeEnumerator.Current;
                return true;
            }
            u = -1;
            return false;
        }

        internal bool TryGetSource(out int u) {
            var enumerator = this.sources.GetEnumerator();
            if (enumerator.MoveNext()) {
                u = enumerator.Current;
                return true;
            }
            u = -1;
            return false;
        }

        internal bool TryGetSink(out int u) {
            var enumerator = this.sinks.GetEnumerator();
            if (enumerator.MoveNext()) {
                u = enumerator.Current;
                return true;
            }
            u = -1;
            return false;
        }

        internal IEnumerable<IntPair> GetOriginalIntPairs() {
            return this.edgesToKeep;
        }

        internal void Initialize() {
            foreach (var p in this.nodeInfoDictionary) {
                this.AddNodeToBucketsSourcesAndSinks(p.Key, p.Value);
            }
        }

        private void AddNodeToBucketsSourcesAndSinks(int v, NodeInfo nodeInfo) {
            if (nodeInfo.InDegree == 0) {
                this.sources.Insert(v);
            } else if (nodeInfo.OutDegree == 0) {
                this.sinks.Insert(v);
            } else if (nodeInfo.InDegreeOfConstrainedEdges == 0) {
                this.GetOrCreateBucket(nodeInfo.DeltaDegree).Insert(v);
            }
        }

        private Set<int> GetOrCreateBucket(int delta) {
            Set<int> ret;
            if (this.deltaDegreeBucketsForSourcesInConstrainedSubgraph.TryGetValue(delta, out ret)) {
                return ret;
            }

            return this.deltaDegreeBucketsForSourcesInConstrainedSubgraph[delta] = new Set<int>();
        }
    }
}
