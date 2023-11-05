using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner{
    /// <summary>
    /// represents a cone side that is broken by the obstacle 
    /// </summary>
    internal class BrokenConeSide:ConeSide {
        /// <summary>
        /// point where it starts
        /// </summary>
        internal Point start;

        internal override Point Start {
            get { return this.start; }
        }

        /// <summary>
        /// it is the side of the cone that intersects the obstacle side
        /// </summary>
        internal ConeSide ConeSide { get; set; }

        internal PolylinePoint EndVertex { get; set; }

        internal Point End {
            get { return this.EndVertex.Point; }
        }
      

        internal BrokenConeSide(Point start, PolylinePoint end, ConeSide coneSide) {
            this.start = start;
            this.EndVertex = end;
            this.ConeSide = coneSide;
        }


        internal override Point Direction {
            get { return this.End - this.Start; }
        }

        public override string ToString() {
            return "BrokenConeSide: " + this.Start + "," + this.End;
        }
    }
}
