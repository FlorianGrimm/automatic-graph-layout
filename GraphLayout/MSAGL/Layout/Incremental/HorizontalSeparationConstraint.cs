using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// A horizontal separation constraint requires a minimum separation between x coordinates of two nodes,
    /// i.e. u.X + separation less or equal v.X
    /// </summary>
    public class HorizontalSeparationConstraint : IConstraint {
        private bool equality;
        /// <summary>
        /// 
        /// </summary>
        public bool IsEquality {
            get { return this.equality; }
        }
        private Node u;
        /// <summary>
        /// Constrained to be vertically above the BottomNode
        /// </summary>
        public Node LeftNode {
            get { return this.u; }
        }
        private Node v;
        /// <summary>
        /// Constrained to be vertically below the TopNode
        /// </summary>
        public Node RightNode {
            get { return this.v; }
        }
        private double separation;
        /// <summary>
        /// We allow the separation of existing constraints to be modified by the user.
        /// </summary>
        public double Separation {
            get { return this.separation; }
            set { this.separation = value; }
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="separation"></param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704")]
        public HorizontalSeparationConstraint(Node u, Node v, double separation)
        {
            this.u = u;
            this.v = v;
            this.separation = separation;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="separation"></param>
        /// <param name="equality"></param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704")]
        public HorizontalSeparationConstraint(Node u, Node v, double separation, bool equality)
        {
            this.equality = equality;
            this.u = u;
            this.v = v;
            this.separation = separation;
        }
        /// <summary>
        /// 
        /// </summary>
        public virtual double Project() {
            double uv = this.v.Center.X - this.u.Center.X;
            double d = this.separation - uv;
            if (d > 0) {
                double
                    wu = ((FiNode)this.u.AlgorithmData).stayWeight,
                    wv = ((FiNode)this.v.AlgorithmData).stayWeight;
                double f = d / (wu + wv);
                this.u.Center = new Point(this.u.Center.X - wv * f, this.u.Center.Y);
                this.v.Center = new Point(this.v.Center.X + wu * f, this.v.Center.Y);
                return d;
            } else {
                return 0;
            }
        }
        /// <summary>
        /// HorizontalSeparationConstraint are usually structural and therefore default to level 0
        /// </summary>
        /// <returns>0</returns>
        public int Level { get { return 0; } }
        /// <summary>
        /// Get the list of nodes involved in the constraint
        /// </summary>
        public IEnumerable<Node> Nodes { get { return new Node[] { this.u, this.v }; } }
    }
}