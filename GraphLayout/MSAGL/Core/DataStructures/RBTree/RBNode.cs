using System;
namespace Microsoft.Msagl.Core.DataStructures {
    

#if TEST_MSAGL
    [Serializable]
#endif
    internal class RBNode<T>
        where T : notnull {

        internal RBColor color;
        internal T Item;
        internal RBNode<T> parent, left, right;

        internal RBNode(RBColor color) { 
            this.color = color;
            this.Item = default!;
            this.parent = null!;
            this.left = null!;
            this.right= null!;
        }
        internal RBNode(RBColor color, T item, RBNode<T> p, RBNode<T> left, RBNode<T> right) {
            this.color = color;
            this.parent = p;
            this.left = left;
            this.right = right;
            this.Item = item;
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        public override string? ToString() {
            return this.Item.ToString();
        }
    }
}
