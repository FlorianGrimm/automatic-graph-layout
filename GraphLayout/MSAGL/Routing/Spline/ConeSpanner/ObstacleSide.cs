using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner
{
    internal abstract class ObstacleSide : SegmentBase
    {
        internal PolylinePoint StartVertex { get; private set; }
        internal ObstacleSide(PolylinePoint startVertex)
        {
            this.StartVertex = startVertex;
        }

        internal abstract PolylinePoint EndVertex { get; }

        internal Polyline Polyline { get { return this.StartVertex.Polyline; } }

        internal override Point Start
        {
            get { return this.StartVertex.Point; }
        }

        internal override Point End
        {
            get { return this.EndVertex.Point; }
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            string typeString = this.GetType().ToString();
            int lastDotLoc = typeString.LastIndexOf('.');
            if (lastDotLoc >= 0) {
                typeString = typeString.Substring(lastDotLoc + 1);
            }
            return typeString + " [" + this.Start.ToString() + " -> " + this.End.ToString() + "]";
        }
    }
}
