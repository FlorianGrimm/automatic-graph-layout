using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    /// <summary>
    /// left here means an intersection of a left cone side with an obstacle edge
    /// </summary>
    internal class LeftIntersectionEvent : SweepEvent {
        internal ConeLeftSide coneLeftSide;
        private Point intersectionPoint;
        private PolylinePoint endVertex;

        internal PolylinePoint EndVertex {
            get { return this.endVertex; }
        }

        internal LeftIntersectionEvent(ConeLeftSide coneLeftSide,
            Point intersectionPoint,
            PolylinePoint endVertex) {
            this.coneLeftSide = coneLeftSide;
            this.intersectionPoint = intersectionPoint;
            this.endVertex = endVertex;
        }

        internal override Point Site {
            get { return this.intersectionPoint; }
        }

        public override string ToString() {
            return "LeftIntersectionEvent " + this.intersectionPoint;
        }
    }
}
