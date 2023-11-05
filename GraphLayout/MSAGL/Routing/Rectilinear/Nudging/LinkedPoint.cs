using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;

#if TEST_MSAGL
using System.Linq;
#endif

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// represents a segment of a path
    /// </summary>
    internal class LinkedPoint : IEnumerable<Point> {
        internal Point Point { get; set; }

        internal LinkedPoint Next { get; set; }

        internal LinkedPoint(Point point) {
            this.Point = point;
        }

        public IEnumerator<Point> GetEnumerator() {
            for (var p = this; p != null; p = p.Next) {
                yield return p.Point;
            }
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return this.GetEnumerator();
        }

        internal double X { get { return this.Point.X; } }
 
        internal double Y { get { return this.Point.Y; } }

        internal void InsertVerts(int i, int j, Point[] points) {
            for (j--; i < j; j--) {
                this.SetNewNext(points[j]);
            }
        }

        public void InsertVertsInReverse(int i, int j, Point[] points) {
            for (i++; i < j; i++) {
                this.SetNewNext(points[i]);
            }
        }

        internal void SetNewNext(Point p) {
            var nv = new LinkedPoint(p);
            var tmp = this.Next;
            this.Next = nv;
            nv.Next = tmp;            
            Debug.Assert( CompassVector.IsPureDirection(this.Point, this.Next.Point) );
        }

#if TEST_MSAGL
        public override string ToString() {
            return this.Point.ToString();
        }
#endif


    }
}
