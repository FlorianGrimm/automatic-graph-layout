using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// An edge is a pair of nodes and an ideal length required between them
    /// </summary>
    internal class FiEdge : IEdge {
        internal Edge mEdge;
        public FiNode source;
        public FiNode target;

        public FiEdge(Edge mEdge) {
            this.mEdge = mEdge;
            this.source = (FiNode) mEdge.Source.AlgorithmData;
            this.target = (FiNode) mEdge.Target.AlgorithmData;
        }
        #region IEdge Members

        public int Source {
            get { return this.source.index; }
            set { }
        }

        public int Target {
            get { return this.target.index; }
            set { }
        }

        #endregion

        internal Point vector() {
            return this.source.mNode.Center - this.target.mNode.Center;
        }
    }
}