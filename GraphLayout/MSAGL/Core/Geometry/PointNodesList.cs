using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.Geometry {
    internal class PointNodesList : IEnumerator<Point>, IEnumerable<Point> {
        private CornerSite current, head;

        private CornerSite Head {
            get { return this.head; }
        }

        internal PointNodesList(CornerSite pointNode) {
            this.head = pointNode;
        }

        internal PointNodesList() { }
        #region IEnumerator<Point> Members

        public Point Current {
            get { return this.current.Point; }
        }
     
        #endregion

        #region IDisposable Members

        public void Dispose() { GC.SuppressFinalize(this); }

        #endregion

        #region IEnumerator Members

        object System.Collections.IEnumerator.Current {
            get { return this.current.Point; }
        }

        public bool MoveNext() {
            if (this.current != null) {
                if (this.current.Next != null) {
                    this.current = this.current.Next;
                    return true;
                } else {
                    return false;
                }
            } else {
                this.current = this.Head;
                return true;
            }

        }

        public void Reset() {
            this.current = null;
        }

        #endregion

        #region IEnumerable<Point> Members

        public IEnumerator<Point> GetEnumerator() {
            return this;
        }

        #endregion

        #region IEnumerable Members

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return this;
        }

        #endregion
    }
}
