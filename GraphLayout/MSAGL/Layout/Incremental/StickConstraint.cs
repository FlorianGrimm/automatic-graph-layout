using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// A stick constraint requires a fixed separation between two nodes
    /// </summary>
    public class StickConstraint : IConstraint {
        /// <summary>
        /// 
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1051:DoNotDeclareVisibleInstanceFields")]
        protected Node u;
        /// <summary>
        /// 
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1051:DoNotDeclareVisibleInstanceFields")]
        protected Node v;
        /// <summary>
        /// 
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1051:DoNotDeclareVisibleInstanceFields")]
        protected double separation;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="separation"></param>
        public StickConstraint(Node u, Node v, double separation) {
            this.u = u;
            this.v = v;
            this.separation = separation;
        }
        /// <summary>
        /// 
        /// </summary>
        public virtual double Project() {
            Point uv = this.v.Center - this.u.Center;
            double d = this.separation - uv.Length,
                   wu = ((FiNode)this.u.AlgorithmData).stayWeight,
                   wv = ((FiNode)this.v.AlgorithmData).stayWeight;
            Point f = d * uv.Normalize() / (wu + wv);
            this.u.Center -= wv * f;
            this.v.Center += wu * f;
            return Math.Abs(d);
        }
        /// <summary>
        /// StickConstraints are usually structural and therefore default to level 0
        /// </summary>
        /// <returns>0</returns>
        public int Level { get { return 0; } }
        
        /// <summary>
        /// Get the list of nodes involved in the constraint
        /// </summary>
        public IEnumerable<Node> Nodes { get { return new Node[]{ this.u, this.v }; } }
    }
}