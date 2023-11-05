using System;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing {
    /// <summary>
    /// an utility class to keep different polylines created around a shape
    /// </summary>
    internal class TightLooseCouple {
        internal Polyline TightPolyline { get; set; }
        internal Shape LooseShape { get; set; }

        internal TightLooseCouple() { }

        public TightLooseCouple(Polyline tightPolyline, Shape looseShape, double distance) {
            this.TightPolyline = tightPolyline;
            this.LooseShape = looseShape;
            this.Distance = distance;
        }
        /// <summary>
        /// the loose polyline has been created with this distance
        /// </summary>
        internal double Distance { get; set; }
        public override string ToString() {
            return (this.TightPolyline == null ? "null" : this.TightPolyline.ToString().Substring(0, 5)) + "," + (this.LooseShape == null ? "null" : this.LooseShape.ToString().Substring(0, 5));
        }
    }
}