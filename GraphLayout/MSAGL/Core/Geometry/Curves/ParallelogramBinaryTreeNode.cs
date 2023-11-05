namespace Microsoft.Msagl.Core.Geometry.Curves {
    /// <summary>
    /// Keeps left and right sons of the node. Is used in curve intersections routines.
    /// </summary>
    internal class ParallelogramBinaryTreeNode:ParallelogramNode {
        private ParallelogramNode leftSon;
        public ParallelogramNode LeftSon {
            get {
                return this.leftSon;
            }
            set {
                this.leftSon = value;
            }
        }

        private ParallelogramNode rightSon;

        public ParallelogramNode RightSon {
            get {
                return this.rightSon;
            }
            set {
                this.rightSon = value;
            }
        }
    }
}
