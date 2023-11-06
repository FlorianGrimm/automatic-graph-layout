using System;
namespace Microsoft.Msagl.Core.DataStructures {

    //public class RBNodeBase<T>
    //    where T : notnull {
    //    //public RBColor color;
    //    //public T Item;
    //    //public RBNode<T> Parent, Left, Right;
    //}
    //public class RBNodeNil<T>
    //    : RBNodeBase<T>
    //    where T : notnull {
    //    //public RBColor color;
    //    //public T Item;
    //    //public RBNode<T> Parent, Left, Right;
    //}

    //public class RBNodeRoot<T>
    //    : RBNodeBase<T>
    //    where T : notnull {
    //    //public RBColor color;
    //    //public T Item;
    //    //public RBNode<T> Parent, Left, Right;
    //}

#if TEST_MSAGL
    [Serializable]
#endif
    public class RBNode<T>
        //: RBNodeRoot<T>
        where T : notnull {

        public RBColor color;
        public T Item;
        public RBNode<T> Parent;
        public RBNode<T> Left;
        public RBNode<T> Right;

        public RBNode(RBColor color) {
            this.color = color;
            this.Item = default!;
            this.Parent = null!;
            this.Left = null!;
            this.Right = null!;
        }
        public RBNode(RBColor color, T item, RBNode<T> p, RBNode<T> left, RBNode<T> right) {
            this.color = color;
            this.Parent = p;
            this.Left = left;
            this.Right = right;
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
