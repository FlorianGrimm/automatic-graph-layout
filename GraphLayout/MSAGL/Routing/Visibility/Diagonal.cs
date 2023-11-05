using System;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Visibility {
    internal class Diagonal {

        public override string ToString() {
            return String.Format(System.Globalization.CultureInfo.InvariantCulture, "{0},{1}", this.Start, this.End);
        }
        internal Point Start {
            get { return this.leftTangent.End.Point; }
        }

        internal Point End {
            get { return this.rightTangent.End.Point; }
        }

        internal Diagonal(Tangent leftTangent, Tangent rightTangent) {
            this.LeftTangent = leftTangent;
            this.RightTangent = rightTangent;
        }

        private Tangent leftTangent;

        internal Tangent LeftTangent {
            get { return this.leftTangent; }
            set { this.leftTangent = value; }
        }

        private Tangent rightTangent;

        internal Tangent RightTangent {
            get { return this.rightTangent; }
            set { this.rightTangent = value; }
        }

        private RBNode<Diagonal> rbNode;

        internal RBNode<Diagonal> RbNode {
            get { return this.rbNode; }
            set { this.rbNode = value; }
        }
    }
}
