using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.Geometry.Curves
{
    internal class PolylineIterator:IEnumerator<Point>
    {
        private Polyline polyline;
        private PolylinePoint currentPolyPoint;

        internal PolylineIterator(Polyline poly)
        {
            this.polyline = poly;
        }

        #region IEnumerator<Point> Members

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=332
        public Point Current
#else
        Point IEnumerator<Point>.Current
#endif
        {
            get { return this.currentPolyPoint.Point; }
        }

        #endregion

        #region IDisposable Members

#if !SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=71
        public void Dispose()
#else
        void IDisposable.Dispose()
#endif
        {
            GC.SuppressFinalize(this);
        }

        #endregion

        #region IEnumerator Members

        object System.Collections.IEnumerator.Current
        {
            get { return this.currentPolyPoint.Point; }
        }

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=332&thanks=332
        public bool MoveNext()
#else
        bool System.Collections.IEnumerator.MoveNext()
#endif
        {
            if (this.currentPolyPoint == null)
            {
                this.currentPolyPoint = this.polyline.StartPoint;
                return this.currentPolyPoint != null;
            }
            if(this.currentPolyPoint == this.polyline.EndPoint) {
                return false;
            }

            this.currentPolyPoint = this.currentPolyPoint.Next;
            return true;
        }

        void System.Collections.IEnumerator.Reset()
        {
            this.currentPolyPoint = null;
        }

        #endregion
    }
}
