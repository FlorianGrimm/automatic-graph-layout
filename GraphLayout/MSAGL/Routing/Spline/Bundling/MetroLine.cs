using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// holds the data of a path
    /// </summary>
#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203
    //SharpKit/Colin - Interface implementations
    // FP: this needs to be public because it is referenced by interfaces elsewhere. It's not public in the .NET version because that version can use explicitly-defined interfaces.
    public class Metroline {
#else
    internal class Metroline {
#endif
        internal double Width;
        internal double Length { get; set; }

        internal double IdealLength { get; set; }

        internal Polyline Polyline { get; set; }
        public int Index { get; set; }

        internal Metroline(Polyline polyline, double width, Func<Tuple<Polyline, Polyline>> sourceAndTargetLoosePolys, int index) {
            this.Width = width;
            this.Polyline = polyline;
            this.sourceAndTargetLoosePolylines = sourceAndTargetLoosePolys;
            this.Index = index;
        }

        internal void UpdateLengths() {
            var l = 0.0;
            for (var p = this.Polyline.StartPoint; p.Next != null; p = p.Next) {
                l += (p.Next.Point - p.Point).Length;
            }
            this.Length = l;
            this.IdealLength = (this.Polyline.End - this.Polyline.Start).Length;
        }

        internal Func<Tuple<Polyline, Polyline>> sourceAndTargetLoosePolylines;
    }
}