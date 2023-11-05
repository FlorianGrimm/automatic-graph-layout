using System;
using System.Collections.Generic;
using System.Collections;

using Microsoft.Msagl.Core.GraphAlgorithms;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Enumerator of the vertex successors
    /// </summary>
    internal class SuccEnumerator : IEnumerator<int> {
        private IEnumerator edges;

        public void Reset() {
            this.edges.Reset();
        }

        public void Dispose() {
            GC.SuppressFinalize(this);
        }


        public bool MoveNext() {
            return this.edges.MoveNext();
        }

        internal SuccEnumerator(IEnumerator edges) {
            this.edges = edges;
        }

        #region IEnumerator<int> Members

        public int Current {
            get {
                return((PolyIntEdge)this.edges.Current).Target;
            }

        }

        #endregion

        #region IEnumerator Members

        object IEnumerator.Current {
            get {
                throw new NotImplementedException();
            }
        }

        #endregion
    }

    /// <summary>
    /// Enumeration of the vertex predecessors
    /// </summary>
    internal class PredEnumerator : IEnumerator<int> {
        private IEnumerator edges;

        public void Dispose() { GC.SuppressFinalize(this); }

        public void Reset() {
            this.edges.Reset();
        }

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203
        public int Current {
#else
        int IEnumerator<int>.Current {
#endif
            get {
                return ((PolyIntEdge)this.edges.Current).Source;
            }
        }

        object IEnumerator.Current {
            get {
                PolyIntEdge l = this.edges.Current as PolyIntEdge;
                return l.Source;
            }
        }

        public bool MoveNext() {
            return this.edges.MoveNext();
        }

        internal PredEnumerator(IEnumerator edges) {
            this.edges = edges;
        }
    }

    internal class EmptyEnumerator : IEnumerator<int> {
        public bool MoveNext() { return false; }

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203
        public int Current { get { return 0; } }
#else
        int IEnumerator<int>.Current { get { return 0; } }
#endif
        object IEnumerator.Current { get { return 0; } }

        void IEnumerator.Reset() { }


        public void Dispose() { GC.SuppressFinalize(this); }

    }

    internal class Pred : IEnumerable<int> {
        #region IEnumerable Members

        private BasicGraphOnEdges<PolyIntEdge> graph;
        private int vert;

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203
        public IEnumerator<int> GetEnumerator() {
#else
        IEnumerator<int> IEnumerable<int>.GetEnumerator() {
#endif
        IEnumerable e = this.graph.InEdges(this.vert);
            if (e == null) {
                return new EmptyEnumerator();
            } else {
                return new PredEnumerator(e.GetEnumerator());
            }
        }

        IEnumerator IEnumerable.GetEnumerator() {
            IEnumerable e = this.graph.InEdges(this.vert);
            if (e == null) {
                return new EmptyEnumerator();
            } else {
                return new PredEnumerator(e.GetEnumerator());
            }
        }

        internal Pred(BasicGraphOnEdges<PolyIntEdge> g, int v) {
            this.graph = g;
            this.vert = v;
        }

        #endregion


    }


    internal class Succ {
        #region IEnumerable Members

        private BasicGraphOnEdges<PolyIntEdge> graph;
        private int vert;

        public IEnumerator<int> GetEnumerator() {
            return new SuccEnumerator(this.graph.OutEdges(this.vert).GetEnumerator());
        }

        internal Succ(BasicGraphOnEdges<PolyIntEdge> g, int v) {
            this.graph = g;
            this.vert = v;
        }



        #endregion
    }

}
