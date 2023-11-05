using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    internal class AxisEdgeHighPointEvent : SweepEvent {
        private Point site;
        internal AxisEdge AxisEdge { get; set; }

        public AxisEdgeHighPointEvent(AxisEdge edge, Point point) {
            this.site = point;
            this.AxisEdge = edge;
        }

        internal override Point Site {
            get { return this.site; }
        }


    }
}