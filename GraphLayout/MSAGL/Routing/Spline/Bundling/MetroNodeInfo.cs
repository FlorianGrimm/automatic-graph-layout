using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class MetroNodeInfo {
        private Metroline metroline;
        private Station station;
        private PolylinePoint polyPoint;

        internal MetroNodeInfo(Metroline metroline, Station station, PolylinePoint polyPoint) {
            this.metroline = metroline;
            this.station = station;
            this.polyPoint = polyPoint;
        }

        internal Metroline Metroline {
            get { return this.metroline; }
        }

        internal PolylinePoint PolyPoint {
            get { return this.polyPoint; }
        }
    }
}