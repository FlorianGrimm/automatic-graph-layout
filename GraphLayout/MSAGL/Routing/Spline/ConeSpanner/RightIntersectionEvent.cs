using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    /// <summary>
    /// right here means an intersection of a right cone side with an obstacle edge
    /// </summary>
    internal class RightIntersectionEvent: SweepEvent {
        internal ConeRightSide coneRightSide;
        private Point intersectionPoint;
        private PolylinePoint endVertex;

        internal PolylinePoint EndVertex {
            get { return this.endVertex; }
            set { this.endVertex = value; }
        }

        internal RightIntersectionEvent(ConeRightSide coneRightSide,
            Point intersectionPoint, PolylinePoint endVertex) {
            this.coneRightSide = coneRightSide;
            this.intersectionPoint = intersectionPoint;
            this.endVertex = endVertex;
        }

        internal override Point Site {
            get { return this.intersectionPoint; }
        }

        public override string ToString() {
            return "RightIntersectionEvent "+ this.intersectionPoint;
        }
    }
}
