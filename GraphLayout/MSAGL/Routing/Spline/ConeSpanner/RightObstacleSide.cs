using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner
{
    internal class RightObstacleSide : ObstacleSide
    {
        private Point end;
        internal RightObstacleSide(PolylinePoint startVertex)
            : base(startVertex)
        {
            this.end = startVertex.PrevOnPolyline.Point;
        }
        internal override Point End
        {
            get { return this.end; }
        }

        internal override PolylinePoint EndVertex
        {
            get { return this.StartVertex.PrevOnPolyline; }
        }

    }
}