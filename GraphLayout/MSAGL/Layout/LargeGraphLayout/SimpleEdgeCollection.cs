using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    internal class SimpleEdgeCollection : EdgeCollection {
        private readonly List<Edge> edges;

        public SimpleEdgeCollection(IEnumerable<Edge> collectionEdges) : base(null) {
            this.edges = collectionEdges as List<Edge>?? 
                collectionEdges.ToList();
        }
        public override IEnumerator<Edge> GetEnumerator() {
            return this.edges.GetEnumerator();
        }

        public override int Count {
            get { return this.edges.Count; }
        }

        public override void Add(Edge item) {
            throw new NotImplementedException();
        }

        public override void Clear() {
            throw new NotImplementedException();
        }

        public override bool Remove(Edge item) {
            throw new NotImplementedException();
        }
    }
}