using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    internal class LeftObstacleSide : ObstacleSide {
        private readonly Point end;
        internal LeftObstacleSide(PolylinePoint startVertex)
            : base(startVertex) {
            this.end = startVertex.NextOnPolyline.Point;
        }
        internal override Point End {
            get { return this.end; }
        }

        internal override PolylinePoint EndVertex {
            get { return this.StartVertex.NextOnPolyline; }
        }

    }
}
