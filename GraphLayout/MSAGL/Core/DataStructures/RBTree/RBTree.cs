using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Text;

namespace Microsoft.Msagl.Core.DataStructures {

#if TEST_MSAGL
    [Serializable]
#endif
    public class RbTree<T> : IEnumerable<T>
        where T : notnull {

        /// <summary>
        /// find the first, minimal, node in the tree such that predicate holds
        /// </summary>
        /// <param name="predicate">Has to be monotone in the sense that if it holds for t then it holds for any t' greater or equal than t
        /// so the predicate values have a form (false, false, ..., false, true, true, ..., true)
        /// </param>
        /// <returns>the first node where predicate holds or null</returns>
        public RBNode<T>? FindFirst(Func<T, bool> predicate)
            => this.FindFirst(this._Root, predicate);

        private RBNode<T>? FindFirst(RBNode<T> n, Func<T, bool> p) {
            if (ReferenceEquals(n, this._Nil)) {
                return null;
            }

            RBNode<T>? good = null;
            while (!ReferenceEquals(n, this._Nil)) {
                n = p(n.Item) ? (good = n).Left : n.Right;
            }

            return good;
        }

        /// <summary>
        /// find the last, maximal, node in the tree such that predicate holds
        /// </summary>
        /// <param name="predicate">Has to be monotone in the sense that if it holds for t then it holds for any t' less or equal than t
        /// so the predicate values on the tree have a form (true, true, ..., true, false, false, ..., false)
        /// </param>
        /// <returns>the last node where predicate holds or null</returns>
        public RBNode<T>? FindLast(Func<T, bool> predicate)
            => this.FindLast(this._Root, predicate);

        private RBNode<T>? FindLast(RBNode<T> n, Func<T, bool> p) {
            if (ReferenceEquals(n, this._Nil)) {
                return null;
            }

            RBNode<T>? good = null;
            while (!ReferenceEquals(n, this._Nil)) {
                n = p(n.Item) ? (good = n).Right : n.Left;
            }

            return good;
        }

        private readonly IComparer<T> _Comparer;

        private IComparer<T> Comparer => this._Comparer;

        public IEnumerator<T> GetEnumerator() { return new RBTreeEnumerator<T>(this); }

        private RBNode<T> _Nil;
        public RBNode<T> Nil { get { return this._Nil; } }

        private RBNode<T> _Root;
        public RBNode<T> Root { get { return this._Root; } }

        public RBNode<T>? Next(RBNode<T>? x) {
            if (x is null) { return default; }

            if (!ReferenceEquals(x.Right , this._Nil)) {
                return this.TreeMinimum(x.Right);
            }

            RBNode<T> y = x.Parent;
            while (!ReferenceEquals(y, this._Nil) && x == y.Right) {
                x = y;
                y = y.Parent;
            }
            return this.ToNull(y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private RBNode<T>? ToNull(RBNode<T>? y) => (y is null || ReferenceEquals(y, this._Nil)) ? null : y;

        public RBNode<T>? Previous(RBNode<T>? x) {
            if (x is null) { return default; }

            if (!ReferenceEquals(x.Left, this._Nil)) {
                return this.TreeMaximum(x.Left);
            }

            RBNode<T> y = x.Parent;
            while (!ReferenceEquals(y, this._Nil) && x == y.Left) {
                x = y;
                y = y.Parent;
            }
            return this.ToNull(y);
        }

        private RBNode<T>? TreeMinimum(RBNode<T> x) {
            while (!ReferenceEquals(x.Left, this._Nil)) {
                x = x.Left;
            }

            return this.ToNull(x);
        }

        public RBNode<T>? TreeMinimum() => this.TreeMinimum(this._Root);

        private RBNode<T>? TreeMaximum(RBNode<T> x) {
            while (!ReferenceEquals(x.Right, this._Nil)) {
                x = x.Right;
            }

            return this.ToNull(x);
        }

        public RBNode<T>? TreeMaximum() => this.TreeMaximum(this._Root);

        public override string ToString() {
            var result = new StringBuilder();
            result.Append('{');
            foreach (T p in this) {
                if (p is null) { continue; }
                if (result.Length > 1) { result.Append(','); }
                result.Append(p.ToString());
            }
            result.Append('}');
            return result.ToString();
        }


        public RBNode<T>? DeleteSubtree(RBNode<T> z) {
            System.Diagnostics.Debug.Assert(!ReferenceEquals(z, this._Nil));

            RBNode<T> y;
            if (ReferenceEquals(z.Left, this._Nil) || ReferenceEquals(z.Right, this._Nil)) {
                /* y has a nil node as a child */
                y = z;
            } else {
                /* find tree successor with a nil node as a child */
                y = z.Right;
                while (!ReferenceEquals(y.Left, this._Nil)) {
                    y = y.Left;
                }
            }

            /* x is y's only child */
            RBNode<T> x = !ReferenceEquals(y.Left, this._Nil) ? y.Left : y.Right;

            x.Parent = y.Parent;
            if (ReferenceEquals(y.Parent, this._Nil)) {
                this._Root = x;
            } else {
                if (y == y.Parent.Left) {
                    y.Parent.Left = x;
                } else {
                    y.Parent.Right = x;
                }
            }
            if (y != z) {
                z.Item = y.Item;
            }

            if (y.color == RBColor.Black) {
                this.DeleteFixup(x);
            }

            //	checkTheTree();

            return this.ToNull(z);

        }

        private int _Count;
        public int Count { get { return this._Count; } }

        public RBNode<T>? Remove(T i) {
            RBNode<T>? n = this.Find(i);
            if (n == null) {
                return null;
            }
            this._Count--;
            return this.DeleteSubtree(n);
        }

        public void DeleteNodeInternal(RBNode<T> x) {
            this._Count--;
            this.DeleteSubtree(x);
        }

        private RBNode<T>? Find(RBNode<T> x, T i) {
            int compareResult;
            while (!ReferenceEquals(x, this._Nil) && (compareResult = this.Comparer.Compare(i, x.Item)) != 0) {
                x = compareResult < 0 ? x.Left : x.Right;
            }

            return this.ToNull(x);
        }

        public RBNode<T>? Find(T i)
            => this.Find(this._Root, i);

        private void DeleteFixup(RBNode<T> x) {
            while (x != this._Root && x.color == RBColor.Black) {
                if (x == x.Parent.Left) {
                    RBNode<T> w = x.Parent.Right;
                    if (w.color == RBColor.Red) {
                        w.color = RBColor.Black;
                        x.Parent.color = RBColor.Red;
                        this.LeftRotate(x.Parent);
                        w = x.Parent.Right;
                    }
                    if (w.Left.color == RBColor.Black && w.Right.color == RBColor.Black) {
                        w.color = RBColor.Red;
                        x = x.Parent;
                    } else {
                        if (w.Right.color == RBColor.Black) {
                            w.Left.color = RBColor.Black;
                            w.color = RBColor.Red;
                            this.RightRotate(w);
                            w = x.Parent.Right;
                        }
                        w.color = x.Parent.color;
                        x.Parent.color = RBColor.Black;
                        w.Right.color = RBColor.Black;
                        this.LeftRotate(x.Parent);
                        x = this._Root;
                    }
                } else {
                    RBNode<T> w = x.Parent.Left;
                    if (w.color == RBColor.Red) {
                        w.color = RBColor.Black;
                        x.Parent.color = RBColor.Red;
                        this.RightRotate(x.Parent);
                        w = x.Parent.Left;
                    }
                    if (w.Right.color == RBColor.Black && w.Left.color == RBColor.Black) {
                        w.color = RBColor.Red;
                        x = x.Parent;
                    } else {
                        if (w.Left.color == RBColor.Black) {
                            w.Right.color = RBColor.Black;
                            w.color = RBColor.Red;
                            this.LeftRotate(w);
                            w = x.Parent.Left;
                        }
                        w.color = x.Parent.color;
                        x.Parent.color = RBColor.Black;
                        w.Left.color = RBColor.Black;
                        this.RightRotate(x.Parent);
                        x = this._Root;
                    }
                }
            }
            x.color = RBColor.Black;
        }

        public bool IsEmpty() { return ReferenceEquals(this._Root, this._Nil); }

        private RBNode<T> TreeInsert(T z) {
            var y = this._Nil;
            var x = this._Root;
            var compareRes = 0;
            while (!ReferenceEquals(x, this._Nil)) {
                y = x;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=368
                compareRes = Comparer.Compare(z, x.Item);
                x = compareRes < 0 ? x.left : x.right;
#else
                x = (compareRes = this.Comparer.Compare(z, x.Item)) < 0 ? x.Left : x.Right;
#endif
            }

            var nz = new RBNode<T>(RBColor.Black, z, y, this._Nil, this._Nil);

            if (ReferenceEquals(y, this._Nil)) {
                this._Root = nz;
            } else if (compareRes < 0) {
                y.Left = nz;
            } else {
                y.Right = nz;
            }

            // nz cannot be null
            // return this.ToNull(nz);
            return nz;
        }

        private void InsertPrivate(RBNode<T> x) {
            this._Count++;
            x.color = RBColor.Red;
            while (x != this._Root && x.Parent.color == RBColor.Red) {
                if (x.Parent == x.Parent.Parent.Left) {
                    RBNode<T> y = x.Parent.Parent.Right;
                    if (y.color == RBColor.Red) {
                        x.Parent.color = RBColor.Black;
                        y.color = RBColor.Black;
                        x.Parent.Parent.color = RBColor.Red;
                        x = x.Parent.Parent;
                    } else {
                        if (x == x.Parent.Right) {
                            x = x.Parent;
                            this.LeftRotate(x);
                        }
                        x.Parent.color = RBColor.Black;
                        x.Parent.Parent.color = RBColor.Red;
                        this.RightRotate(x.Parent.Parent);
                    }
                } else {
                    RBNode<T> y = x.Parent.Parent.Left;
                    if (y.color == RBColor.Red) {
                        x.Parent.color = RBColor.Black;
                        y.color = RBColor.Black;
                        x.Parent.Parent.color = RBColor.Red;
                        x = x.Parent.Parent;
                    } else {
                        if (x == x.Parent.Left) {
                            x = x.Parent;
                            this.RightRotate(x);
                        }
                        x.Parent.color = RBColor.Black;
                        x.Parent.Parent.color = RBColor.Red;
                        this.LeftRotate(x.Parent.Parent);
                    }
                }

            }

            this._Root.color = RBColor.Black;
        }

        public RBNode<T> Insert(T v) {
            RBNode<T> result = this.TreeInsert(v);
            this.InsertPrivate(result);
            // x cannot be null
            // return this.ToNull(result);
            return result;
        }

        private void LeftRotate(RBNode<T> x) {
            RBNode<T> y = x.Right;
            x.Right = y.Left;
            if (!ReferenceEquals(y.Left, this._Nil)) {
                y.Left.Parent = x;
            }

            y.Parent = x.Parent;
            if (ReferenceEquals(x.Parent, this._Nil)) {
                this._Root = y;
            } else if (x == x.Parent.Left) {
                x.Parent.Left = y;
            } else {
                x.Parent.Right = y;
            }

            y.Left = x;
            x.Parent = y;
        }

        private void RightRotate(RBNode<T> x) {
            RBNode<T> y = x.Left;
            x.Left = y.Right;
            if (!ReferenceEquals(y.Right, this._Nil)) {
                y.Right.Parent = x;
            }

            y.Parent = x.Parent;
            if (ReferenceEquals(x.Parent, this._Nil)) {
                this._Root = y;
            } else if (x == x.Parent.Right) {
                x.Parent.Right = y;
            } else {
                x.Parent.Left = y;
            }

            y.Right = x;
            x.Parent = y;

        }

        public RbTree(Func<T?, T?, int> func) : this(new ComparerOnDelegate<T>(func)) { }


        public RbTree(IComparer<T> comparer) {
            this._Root = this._Nil = new RBNode<T>(RBColor.Black);
            this._Comparer = comparer;
        }

        public void Clear() {
            this._Root = this._Nil = new RBNode<T>(RBColor.Black);
        }


        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return new RBTreeEnumerator<T>(this);
        }
    }
}
