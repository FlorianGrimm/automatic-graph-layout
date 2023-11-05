using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// An edge with source and target represented as integers, they point to an array of Nodes of the graph
    /// </summary>
    [SuppressMessage("Microsoft.Naming", "CA1710:IdentifiersShouldHaveCorrectSuffix")]
#if TEST_MSAGL
    public
#else
    internal
#endif
        class PolyIntEdge : IEnumerable<int>, IEdge {
        private int source;

        /// <summary>
        /// the source
        /// </summary>
        public int Source {
            get { return this.source; }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            set { source = value; UpdateHashKey(); }
#else
            set { this.source = value; }
#endif
        }

        private int target;

        /// <summary>
        /// the edge target
        /// </summary>
        public int Target {
            get { return this.target; }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            set { target = value; UpdateHashKey(); }
#else
            set { this.target = value; }
#endif
        }

        private bool reversed;

        /// <summary>
        /// An edge can be reversed to keep the graph acyclic
        /// </summary>
        public bool Reversed {
            get { return this.reversed; }
//            set { reversed = value; }
        }

        /// <summary>
        /// A dummy edge that will not be drawn; serves just as a place holder.
        /// </summary>
        public bool IsVirtualEdge { get; set; }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="source"></param>
        /// <param name="target"></param>
        internal PolyIntEdge(int source, int target) {
            this.source = source;
            this.target = target;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            UpdateHashKey();
#endif
        }

        /// <summary>
        /// Returns true if the edge has label
        /// </summary>
        internal bool HasLabel {
            get { return this.Edge.Label != null; }
        }

        /// <summary>
        /// Label width
        /// </summary>
        internal double LabelWidth {
            get { return this.Edge.Label.Width; }
        }

        /// <summary>
        /// Label height
        /// </summary>
        internal double LabelHeight {
            get { return this.Edge.Label.Height; }
        }

        /// <summary>
        /// This function changes the edge by swapping 
        /// source and target. However Revert(Revert) does not change it.
        /// </summary>
        internal void Revert() {
            int t = this.source;
            this.source = this.target;
            this.target = t;
            this.reversed = !this.reversed;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            UpdateHashKey();
#endif
        }

        /// <summary>
        /// The original edge corresponding to the IntEdge
        /// </summary>
        public Edge Edge { get; set; }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="source"></param>
        /// <param name="target"></param>
        /// <param name="edge"></param>
        internal PolyIntEdge(int source, int target, Edge edge) {
            this.source = source;
            this.target = target;
            this.Edge = edge;
            if (edge != null) {
                this.Separation = edge.Separation;
                this.Weight = edge.Weight;
            }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            UpdateHashKey();
#endif
        }

        /// <summary>
        /// compares only source and target
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj) {
            var ie = obj as PolyIntEdge;
            if (ie == null) {
                return false;
            }

            return ie.source == this.source &&
                   ie.target == this.target;
        }

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
        private SharpKit.JavaScript.JsString _hashKey;
        private void UpdateHashKey()
        {
            _hashKey = ""+source+","+target;
        }
#endif

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode() {
            var hc = (uint)this.source.GetHashCode();
            return (int) ((hc << 5 | hc >> 27) + (uint)this.target);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return "Edge(" + this.source + "->" + this.target + ")";
        }

        internal ICurve Curve {
            get { return this.Edge.Curve; }
            set { this.Edge.Curve = value; }
        }


        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal SmoothedPolyline UnderlyingPolyline {
            get { return this.Edge.UnderlyingPolyline; }
            set { this.Edge.UnderlyingPolyline = value; }
        }

        private int weight = 1;

        internal int Weight {
            get { return this.weight; }
            set { this.weight = value; }
        }

        
        internal int CrossingWeight {
            get { return 1; }
        }

        private int separation;

        /// <summary>
        /// the distance between the source and the target in the number of layers
        /// </summary>
        public int Separation {
            get { return this.separation; }
            set { this.separation = value; }
        }

        /// <summary>
        /// the edge span in layers
        /// </summary>
        public int LayerSpan {
            get {
                return this.layerEdges != null ? this.layerEdges.Length : 0;
                // return virtualStart == -1 ? 1 : VirtualEnd - VirtualStart + 2;
            }
        }

        private LayerEdge[] layerEdges;
        /// <summary>
        /// 
        /// </summary>
#if TEST_MSAGL
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA2227:CollectionPropertiesShouldBeReadOnly")]
        public
#else
        internal
#endif
            IList<LayerEdge> LayerEdges {
            get { return this.layerEdges; }
            set { this.layerEdges = (LayerEdge[]) value; }
        }


        internal bool SelfEdge() {
            return this.source == this.target;
        }

        internal PolyIntEdge ReversedClone() {
            var ret = new PolyIntEdge(this.target, this.source, this.Edge);
            if (this.layerEdges != null) {
                int len = this.layerEdges.Length;
                ret.layerEdges = new LayerEdge[len];
                for (int i = 0; i < len; i++) {
                    LayerEdge le = this.layerEdges[len - 1 - i];
                    ret.layerEdges[i] = new LayerEdge(le.Target, le.Source, le.CrossingWeight);
                }
                ret.layerEdges[0].Source = this.target;
                ret.layerEdges[this.layerEdges.Length - 1].Target = this.source;
            }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            ret.UpdateHashKey();
#endif
            return ret;
        }

        internal LayerEdge this[int i] {
            get { return this.layerEdges[i]; }
        }

        internal int Count {
            get { return this.layerEdges.Length; }
        }


        internal void UpdateEdgeLabelPosition(Anchor[] anchors) {
            if (this.Edge.Label != null) {
                int m = this.layerEdges.Length/2;
                LayerEdge layerEdge = this.layerEdges[m];
                Routing.UpdateLabel(this.Edge, anchors[layerEdge.Source]);
            }
        }

        #region IEnumerable<int> Members

        /// <summary>
        /// enumerates over virtual virtices corresponding to the original edge
        /// </summary>
        /// <returns></returns>
        public IEnumerator<int> GetEnumerator() {
            yield return this.layerEdges[0].Source;
            foreach (LayerEdge le in this.layerEdges) {
                yield return le.Target;
            }
        }

        #endregion

        #region IEnumerable Members

        IEnumerator IEnumerable.GetEnumerator() {
            yield return this.layerEdges[0].Source;
            foreach (LayerEdge le in this.layerEdges) {
                yield return le.Target;
            }
        }

        #endregion

        /// <summary>
        /// The function returns an array arr such that
        /// arr is a permutation of the graph vertices,
        /// and for any edge e in graph if e.Source=arr[i]
        /// e.Target=arr[j], then i is less than j
        /// </summary>
        /// <param name="graph"></param>
        /// <returns></returns>
        internal static int[] GetOrder(BasicGraphOnEdges<PolyIntEdge> graph){
            var visited = new bool[graph.NodeCount];

            //no recursion! So we have to organize a stack
            var sv = new Stack<int>();
            var se = new Stack<IEnumerator<int>>();

            var order = new List<int>();

            IEnumerator<int> en;
            for (int u = 0; u < graph.NodeCount; u++){
                if (visited[u]) {
                    continue;
                }

                int cu = u;
                visited[cu] = true;
                en = new Succ(graph, u).GetEnumerator();

                do{
                    while (en.MoveNext()){
                        int v = en.Current;
                        if (!visited[v]){
                            visited[v] = true;
                            sv.Push(cu);
                            se.Push(en);
                            cu = v;
                            en = new Succ(graph, cu).GetEnumerator();
                        }
                    }
                    order.Add(cu);


                    if (sv.Count > 0){
                        en = se.Pop();
                        cu = sv.Pop();
                    }
                    else {
                        break;
                    }
                } while (true);
            }

            order.Reverse();

            return order.ToArray();
        }
        }
}