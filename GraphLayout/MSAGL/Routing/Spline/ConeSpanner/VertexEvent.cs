using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    abstract internal class VertexEvent: SweepEvent {
        private PolylinePoint vertex;

        internal PolylinePoint Vertex {
            get { return this.vertex; }
            set { this.vertex = value; }
        }

        internal override Point Site {
            get { return this.vertex.Point; }
        }

        internal VertexEvent(PolylinePoint p) { this.vertex = p; }
        internal Polyline Polyline { get { return this.vertex.Polyline; } }
    }
}
