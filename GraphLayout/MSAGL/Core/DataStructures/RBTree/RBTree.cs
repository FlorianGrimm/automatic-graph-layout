using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace Microsoft.Msagl.Core.DataStructures {

#if TEST_MSAGL
    [Serializable]
#endif
    internal class RbTree<T> : IEnumerable<T>
        where T : notnull {

        /// <summary>
        /// find the first, minimal, node in the tree such that predicate holds
        /// </summary>
        /// <param name="predicate">Has to be monotone in the sense that if it holds for t then it holds for any t' greater or equal than t
        /// so the predicate values have a form (false, false, ..., false, true, true, ..., true)
        /// </param>
        /// <returns>the first node where predicate holds or null</returns>
        internal RBNode<T>? FindFirst(Func<T, bool> predicate)
            => this.FindFirst(this._Root, predicate);

        private RBNode<T>? FindFirst(RBNode<T> n, Func<T, bool> p) {
            if (n == this._Nil) {
                return null;
            }

            RBNode<T>? good = null;
            while (n != this._Nil) {
                n = p(n.Item) ? (good = n).left : n.right;
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
        internal RBNode<T>? FindLast(Func<T, bool> predicate)
            => this.FindLast(this._Root, predicate);

        private RBNode<T>? FindLast(RBNode<T> n, Func<T, bool> p) {
            if (n == this._Nil) {
                return null;
            }

            RBNode<T>? good = null;
            while (n != this._Nil) {
                n = p(n.Item) ? (good = n).right : n.left;
            }

            return good;
        }

        private readonly IComparer<T> _Comparer;

        private IComparer<T> Comparer => this._Comparer;

        public IEnumerator<T> GetEnumerator() { return new RBTreeEnumerator<T>(this); }

        private RBNode<T> _Nil;
        internal RBNode<T> Nil { get { return this._Nil; } }

        private RBNode<T> _Root;
        internal RBNode<T> Root { get { return this._Root; } }

        internal RBNode<T>? Next(RBNode<T> x) {
            if (x.right != this._Nil) {
                return this.TreeMinimum(x.right);
            }

            RBNode<T> y = x.parent;
            while (y != this._Nil && x == y.right) {
                x = y;
                y = y.parent;
            }
            return this.ToNull(y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private RBNode<T>? ToNull(RBNode<T>? y) => (y is null || y == this._Nil) ? null : y;

        internal RBNode<T>? Previous(RBNode<T> x) {
            if (x.left != this._Nil) {
                return this.TreeMaximum(x.left);
            }

            RBNode<T> y = x.parent;
            while (y != this._Nil && x == y.left) {
                x = y;
                y = y.parent;
            }
            return this.ToNull(y);
        }

        private RBNode<T>? TreeMinimum(RBNode<T> x) {
            while (x.left != this._Nil) {
                x = x.left;
            }

            return this.ToNull(x);
        }

        internal RBNode<T>? TreeMinimum() => this.TreeMinimum(this._Root);

        private RBNode<T>? TreeMaximum(RBNode<T> x) {
            while (x.right != this._Nil) {
                x = x.right;
            }

            return this.ToNull(x);
        }

        internal RBNode<T>? TreeMaximum() => this.TreeMaximum(this._Root);

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


        internal RBNode<T>? DeleteSubtree(RBNode<T> z) {
            System.Diagnostics.Debug.Assert(z != this._Nil);

            RBNode<T> y;
            if (z.left == this._Nil || z.right == this._Nil) {
                /* y has a nil node as a child */
                y = z;
            } else {
                /* find tree successor with a nil node as a child */
                y = z.right;
                while (y.left != this._Nil) {
                    y = y.left;
                }
            }

            /* x is y's only child */
            RBNode<T> x = y.left != this._Nil ? y.left : y.right;

            x.parent = y.parent;
            if (y.parent == this._Nil) {
                this._Root = x;
            } else {
                if (y == y.parent.left) {
                    y.parent.left = x;
                } else {
                    y.parent.right = x;
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

        internal RBNode<T>? Remove(T i) {
            RBNode<T>? n = this.Find(i);
            if (n == null) {
                return null;
            }
            this._Count--;
            return this.DeleteSubtree(n);
        }

        internal void DeleteNodeInternal(RBNode<T> x) {
            this._Count--;
            this.DeleteSubtree(x);
        }

        private RBNode<T>? Find(RBNode<T> x, T i) {
            int compareResult;
            while (x != this._Nil && (compareResult = this.Comparer.Compare(i, x.Item)) != 0) {
                x = compareResult < 0 ? x.left : x.right;
            }

            return this.ToNull(x);
        }

        internal RBNode<T>? Find(T i)
            => this.Find(this._Root, i);

        private void DeleteFixup(RBNode<T> x) {
            while (x != this._Root && x.color == RBColor.Black) {
                if (x == x.parent.left) {
                    RBNode<T> w = x.parent.right;
                    if (w.color == RBColor.Red) {
                        w.color = RBColor.Black;
                        x.parent.color = RBColor.Red;
                        this.LeftRotate(x.parent);
                        w = x.parent.right;
                    }
                    if (w.left.color == RBColor.Black && w.right.color == RBColor.Black) {
                        w.color = RBColor.Red;
                        x = x.parent;
                    } else {
                        if (w.right.color == RBColor.Black) {
                            w.left.color = RBColor.Black;
                            w.color = RBColor.Red;
                            this.RightRotate(w);
                            w = x.parent.right;
                        }
                        w.color = x.parent.color;
                        x.parent.color = RBColor.Black;
                        w.right.color = RBColor.Black;
                        this.LeftRotate(x.parent);
                        x = this._Root;
                    }
                } else {
                    RBNode<T> w = x.parent.left;
                    if (w.color == RBColor.Red) {
                        w.color = RBColor.Black;
                        x.parent.color = RBColor.Red;
                        this.RightRotate(x.parent);
                        w = x.parent.left;
                    }
                    if (w.right.color == RBColor.Black && w.left.color == RBColor.Black) {
                        w.color = RBColor.Red;
                        x = x.parent;
                    } else {
                        if (w.left.color == RBColor.Black) {
                            w.right.color = RBColor.Black;
                            w.color = RBColor.Red;
                            this.LeftRotate(w);
                            w = x.parent.left;
                        }
                        w.color = x.parent.color;
                        x.parent.color = RBColor.Black;
                        w.left.color = RBColor.Black;
                        this.RightRotate(x.parent);
                        x = this._Root;
                    }
                }
            }
            x.color = RBColor.Black;
        }

        internal bool IsEmpty() { return this._Root == this._Nil; }

        private RBNode<T>? TreeInsert(T z) {
            var y = this._Nil;
            var x = this._Root;
            var compareRes = 0;
            while (x != this._Nil) {
                y = x;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=368
                compareRes = Comparer.Compare(z, x.Item);
                x = compareRes < 0 ? x.left : x.right;
#else
                x = (compareRes = this.Comparer.Compare(z, x.Item)) < 0 ? x.left : x.right;
#endif
            }

            var nz = new RBNode<T>(RBColor.Black, z, y, this._Nil, this._Nil);

            if (y == this._Nil) {
                this._Root = nz;
            } else if (compareRes < 0) {
                y.left = nz;
            } else {
                y.right = nz;
            }

            return this.ToNull(nz);
        }

        private void InsertPrivate(RBNode<T> x) {
            this._Count++;
            x.color = RBColor.Red;
            while (x != this._Root && x.parent.color == RBColor.Red) {
                if (x.parent == x.parent.parent.left) {
                    RBNode<T> y = x.parent.parent.right;
                    if (y.color == RBColor.Red) {
                        x.parent.color = RBColor.Black;
                        y.color = RBColor.Black;
                        x.parent.parent.color = RBColor.Red;
                        x = x.parent.parent;
                    } else {
                        if (x == x.parent.right) {
                            x = x.parent;
                            this.LeftRotate(x);
                        }
                        x.parent.color = RBColor.Black;
                        x.parent.parent.color = RBColor.Red;
                        this.RightRotate(x.parent.parent);
                    }
                } else {
                    RBNode<T> y = x.parent.parent.left;
                    if (y.color == RBColor.Red) {
                        x.parent.color = RBColor.Black;
                        y.color = RBColor.Black;
                        x.parent.parent.color = RBColor.Red;
                        x = x.parent.parent;
                    } else {
                        if (x == x.parent.left) {
                            x = x.parent;
                            this.RightRotate(x);
                        }
                        x.parent.color = RBColor.Black;
                        x.parent.parent.color = RBColor.Red;
                        this.LeftRotate(x.parent.parent);
                    }
                }

            }

            this._Root.color = RBColor.Black;
        }

        internal RBNode<T>? Insert(T v) {
            RBNode<T>? x = this.TreeInsert(v);
            if (x is not null) {
                this.InsertPrivate(x);
            }
            return this.ToNull(x);
        }

        private void LeftRotate(RBNode<T> x) {
            RBNode<T> y = x.right;
            x.right = y.left;
            if (y.left != this._Nil) {
                y.left.parent = x;
            }

            y.parent = x.parent;
            if (x.parent == this._Nil) {
                this._Root = y;
            } else if (x == x.parent.left) {
                x.parent.left = y;
            } else {
                x.parent.right = y;
            }

            y.left = x;
            x.parent = y;
        }

        private void RightRotate(RBNode<T> x) {
            RBNode<T> y = x.left;
            x.left = y.right;
            if (y.right != this._Nil) {
                y.right.parent = x;
            }

            y.parent = x.parent;
            if (x.parent == this._Nil) {
                this._Root = y;
            } else if (x == x.parent.right) {
                x.parent.right = y;
            } else {
                x.parent.left = y;
            }

            y.right = x;
            x.parent = y;

        }

        internal RbTree(Func<T?, T?, int> func) : this(new ComparerOnDelegate<T>(func)) { }


        internal RbTree(IComparer<T> comparer) {
            this._Root = this._Nil = new RBNode<T>(RBColor.Black);
            this._Comparer = comparer;
        }

        internal void Clear() {
            this._Root = this._Nil = new RBNode<T>(RBColor.Black);
        }


        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return new RBTreeEnumerator<T>(this);
        }
    }
}
