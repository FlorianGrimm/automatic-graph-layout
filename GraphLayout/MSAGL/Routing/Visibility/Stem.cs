using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Visibility {
    /// <summary>
    /// represents a chunk of a hole boundary
    /// </summary>
    internal class Stem {
        private PolylinePoint start;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal PolylinePoint Start {
            get { return this.start; }
            set { this.start = value; }
        }

        private PolylinePoint end;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal PolylinePoint End {
            get { return this.end; }
            set { this.end = value; }
        }
     
        
        internal Stem(PolylinePoint start, PolylinePoint end) {
            System.Diagnostics.Debug.Assert(start.Polyline==end.Polyline);
            this.start = start;
            this.end = end;
        }

        internal IEnumerable<PolylinePoint> Sides {
            get {
                PolylinePoint v= this.start;
                
                while(v!= this.end ) {
                    PolylinePoint side = v;
                    yield return side;
                    v = side.NextOnPolyline;
                }
            }
        }

        internal bool MoveStartClockwise() {
            if (this.Start != this.End) {
                this.Start = this.Start.NextOnPolyline;
                return true;
            }
            return false;
        }

        public override string ToString() {
            return String.Format(System.Globalization.CultureInfo.InvariantCulture, "Stem({0},{1})", this.Start, this.End);
        }

    }
}
