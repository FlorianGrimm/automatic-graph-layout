using System.Collections.Generic;

namespace Microsoft.Msagl.Core.GraphAlgorithms {
    internal class BasicGraph<TNode, TEdge> : BasicGraphOnEdges<TEdge> where TEdge : IEdge
    {
        
        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="edges"></param>
        /// <param name="numberOfVerts"></param>
        internal BasicGraph(IEnumerable<TEdge> edges, int numberOfVerts) : base(edges, numberOfVerts)
        {
        }

        private IList<TNode> nodes;

        /// <summary>
        /// array of nodes
        /// </summary>
        public IList<TNode> Nodes {
            get { return this.nodes; }
            set { this.nodes = value; }
        }
    }
}
