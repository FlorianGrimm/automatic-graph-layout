using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;

namespace Microsoft.Msagl.Core.GraphAlgorithms {
    internal class NodeInfo {
        private Set<int> outEdges = new Set<int>();

        internal Set<int> OutEdges {
            get { return this.outEdges; }
        }

        private Set<int> inEdges = new Set<int>();

        internal Set<int> InEdges {
            get { return this.inEdges; }
        }

        private Set<int> outConstrainedEdges = new Set<int>();

        internal Set<int> OutConstrainedEdges {
            get { return this.outConstrainedEdges; }
        }

        private Set<int> inConstrainedEdges = new Set<int>();

        public Set<int> InConstrainedEdges {
            get { return this.inConstrainedEdges; }
        }
        /// <summary>
        /// it is the out degree without the in degree
        /// </summary>
        internal int DeltaDegree {
            get { return this.InDegree - this.OutDegree; }
        }
        internal void AddOutEdge(int v) {
            this.outEdges.Insert(v);
        }
        internal void RemoveOutEdge(int v) {
            this.outEdges.Remove(v);
        }

        internal void AddInEdge(int v) {
            this.inEdges.Insert(v);
        }
        internal void RemoveInEdge(int v) {
            this.inEdges.Remove(v);
        }
        internal void AddOutConstrainedEdge(int v) {
            this.outConstrainedEdges.Insert(v);
        }
        internal void RemoveOutConstrainedEdge(int v) {
            this.outConstrainedEdges.Remove(v);
        }

        internal void AddInConstrainedEdge(int v) {
            this.inConstrainedEdges.Insert(v);
        }
        internal void RemoveInConstrainedEdge(int v) {
            this.inConstrainedEdges.Remove(v);
        }

        internal int OutDegree {
            get {
                return this.outEdges.Count+ this.outConstrainedEdges.Count;
            }
        }
        internal int InDegreeOfConstrainedEdges {
            get {
                return this.inConstrainedEdges.Count;
            }
        }
        internal int InDegree {
            get { return this.inEdges.Count+ this.inConstrainedEdges.Count; }
        }


        /// <summary>
        /// including constrained neighbors
        /// </summary>
        internal IEnumerable<int> AllNeighbors {
            get {
                foreach (int v in this.OutConstrainedEdges) {
                    yield return v;
                }

                foreach (int v in this.InConstrainedEdges) {
                    yield return v;
                }

                foreach (int v in this.OutEdges) {
                    yield return v;
                }

                foreach (int v in this.InEdges) {
                    yield return v;
                }
            }
        }
    }
}
