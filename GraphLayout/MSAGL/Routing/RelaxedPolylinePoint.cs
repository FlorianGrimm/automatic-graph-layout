using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing {
    internal class RelaxedPolylinePoint  {
        private PolylinePoint polylinePoint;

        internal PolylinePoint PolylinePoint {
            get { return this.polylinePoint; }
            set { this.polylinePoint = value; }
        }
        private Point originalPosition;

        internal Point OriginalPosition {
            get { return this.originalPosition; }
            set { this.originalPosition = value; }
        }
        
        internal RelaxedPolylinePoint(PolylinePoint polylinePoint, Point originalPosition) {
            this.PolylinePoint = polylinePoint;
            this.OriginalPosition = originalPosition;
        }

        private RelaxedPolylinePoint next;

        internal RelaxedPolylinePoint Next {
            get { return this.next; }
            set { this.next = value; }
        }

        private RelaxedPolylinePoint prev;

        internal RelaxedPolylinePoint Prev {
            get { return this.prev; }
            set { this.prev = value; }
        }
    }
}
