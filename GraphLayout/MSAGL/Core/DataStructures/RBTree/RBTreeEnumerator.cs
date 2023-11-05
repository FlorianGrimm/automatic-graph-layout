using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.DataStructures {
    internal class RBTreeEnumerator<T> : IEnumerator<T>
        where T : notnull {
        private readonly RbTree<T> _Tree;
        private bool _InitialState;
        private RBNode<T>? _Current;
                
        public T Current {
            get {
                if (_Current == null) {
                    throw new InvalidOperationException();
                } else { 
                    return this._Current.Item;
                }
            }
        }

        public void Reset() {
            this._InitialState = true;
        }

        public bool MoveNext() {
            if (this._Tree.IsEmpty()) {
                return false;
            }

            if (this._InitialState == true) {
                this._InitialState = false;
                this._Current = this._Tree.TreeMinimum();
            } else if (this._Current is not null) {
                this._Current = this._Tree.Next(this._Current);
            } else {
                this._Current = null;
            }
            return this._Current != null;
        }

        internal RBTreeEnumerator(RbTree<T> tree) {
            this._Tree = tree;
            this.Reset();

        }


        public void Dispose() {
            GC.SuppressFinalize(this);
        }


        object System.Collections.IEnumerator.Current {
            get {
                if (this._Current is null) { 
                    throw new InvalidOperationException();
                } else {
                    return this._Current.Item; 
                }
            }
        }
    }
}
